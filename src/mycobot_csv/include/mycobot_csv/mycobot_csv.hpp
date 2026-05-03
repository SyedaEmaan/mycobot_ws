/**
 * mycobot_csv.hpp — Layer 4 refactor (organizational).
 *
 * L4 changes:
 *   - enable_drive() factored: three near-identical state-transition blocks
 *     replaced by calls to a new helper cia402_transition()
 *   - slave_csv_config() factored into named static helpers (one per
 *     numbered step from simple_ng.c)
 *   - File-header overview comment added to mycobot_csv.cpp
 *
 * What did NOT change:
 *   - Public ros2_control SystemInterface contract (on_init / on_activate / etc.)
 *   - PDO struct layouts
 *   - All constants in cw::, sw::, brake::, timing::
 *   - BringupResult enum and result_str()
 *
 * Verification: behavior is identical to L3. The diff is purely about
 * code organization — every SDO write happens at the same logical moment
 * for the same reason as before.
 *
 * == File overview ==
 *
 *   Public API: hardware_interface::SystemInterface
 *     ↳ on_init / on_activate / on_deactivate
 *     ↳ export_state_interfaces / export_command_interfaces
 *     ↳ read / write
 *
 *   Bring-up sequence (called from on_activate):
 *     init_soem()           — ec_init, config_init, PDO mapping
 *     reach_safe_op()       — EtherCAT state PRE_OP → SAFE_OP
 *     reach_operational()   — prime PDOs, SAFE_OP → OPERATIONAL
 *     enable_drive(s)       — CiA 402: SWITCH_ON_DISABLED → OPERATION_ENABLED
 *
 *   Per-cycle (called from ControllerManager update loop):
 *     read()                — copy txpdo into hw_positions_/hw_velocities_
 *     write()               — copy hw_velocity_commands_ into rxpdo + roundtrip
 *
 *   Static SOEM callback (invoked by SOEM during PRE_OP → SAFE_OP):
 *     slave_csv_config(s)   — orchestrates the configure_* helpers below
 *       configure_csv_mode
 *       configure_interpolation_period
 *       configure_txpdo
 *       configure_rxpdo
 *       configure_brake_output
 *       configure_sync_managers
 *       configure_free_run_sync
 */

#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>
#include <cstdint>

extern "C" {
#include "soem/soem.h"
}

namespace mycobot_csv {

/* ================================================================== */
/* PDO structures — verbatim from simple_ng.c                         */
/* ================================================================== */
typedef struct __attribute__((packed)) {
    uint16_t controlword;
    int32_t  target_velocity;
    uint16_t touch_probe_func;
    uint32_t physical_output;
} csv_rxpdo_t;

typedef struct __attribute__((packed)) {
    uint16_t statusword;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    uint16_t touch_probe_stat;
    int32_t  touch_probe_pos1;
    int32_t  touch_probe_pos2;
    uint32_t physical_inputs;
} csv_txpdo_t;

/* ================================================================== */
/* CiA 402 controlword bits & composed values                         */
/* ================================================================== */
namespace cw {
    constexpr uint16_t SWITCH_ON         = 1u << 0;
    constexpr uint16_t ENABLE_VOLTAGE    = 1u << 1;
    constexpr uint16_t QUICK_STOP        = 1u << 2;
    constexpr uint16_t ENABLE_OPERATION  = 1u << 3;
    constexpr uint16_t FAULT_RESET       = 1u << 7;

    constexpr uint16_t SHUTDOWN          = QUICK_STOP | ENABLE_VOLTAGE;
    constexpr uint16_t SWITCH_ON_CMD     = QUICK_STOP | ENABLE_VOLTAGE | SWITCH_ON;
    constexpr uint16_t ENABLE_OP_CMD     = QUICK_STOP | ENABLE_VOLTAGE | SWITCH_ON | ENABLE_OPERATION;
}

/* ================================================================== */
/* CiA 402 statusword patterns                                        */
/* ================================================================== */
namespace sw {
    constexpr uint16_t READY_TO_SWITCH_ON = 0x0021;
    constexpr uint16_t SWITCHED_ON        = 0x0023;
    constexpr uint16_t OPERATION_ENABLED  = 0x0027;
    constexpr uint16_t FAULT_BIT          = 0x0008;
    constexpr uint16_t STATE_MASK         = 0x006F;
}

/* ================================================================== */
/* Brake control                                                      */
/* ================================================================== */
namespace brake {
    constexpr uint32_t RELEASED = 0x00000001u;
    constexpr uint32_t ENGAGED  = 0x00000000u;
}

/* ================================================================== */
/* Timing constants                                                   */
/* ================================================================== */
namespace timing {
    constexpr int POLL_CYCLE_US        =   5'000;
    constexpr int MODE_SETTLE_US       =  50'000;
    constexpr int ERROR_ACK_SETTLE_US  = 100'000;
    constexpr int DRIVE_DISABLE_US     =  50'000;

    constexpr int FAULT_RESET_RETRIES        = 100;
    constexpr int CIA402_TRANSITION_RETRIES  = 200;
    constexpr int BRAKE_RELEASE_CYCLES       =  20;

    constexpr int SAFE_OP_RETRIES   = 20;
    constexpr int OPERATIONAL_RETRIES = 40;

    constexpr int PRIMING_INITIAL_CYCLES = 5;
    constexpr int PRIMING_FINAL_CYCLES   = 10;

    constexpr int RECEIVE_TIMEOUT_US = 10'000;
}

/* ================================================================== */
/* BringupResult                                                       */
/* ================================================================== */
enum class BringupResult {
    OK,
    EC_INIT_FAILED,
    NO_SLAVES_FOUND,
    INVALID_SLAVE_INDEX,
    PDO_SIZE_MISMATCH,
    NULL_PDO_BUFFER,
    SAFE_OP_TIMEOUT,
    OPERATIONAL_TIMEOUT,
    FAULT_NOT_CLEARED,
    SHUTDOWN_FAILED,
    SWITCH_ON_FAILED,
    ENABLE_OPERATION_FAILED,
};

const char * result_str(BringupResult r);

/* ================================================================== */
/* Plugin class                                                        */
/* ================================================================== */
class MyCobotCSV : public hardware_interface::SystemInterface
{
public:
  /* ros2_control public API */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /* ros2_control buffers */
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_velocity_commands_;

  std::vector<int>     joint_to_slave_;
  std::vector<double>  counts_per_rad_;
  std::vector<int32_t> last_position_counts_;

  std::string ifname_;
  size_t      n_joints_{0};

  /* SOEM state */
  ecx_contextt context_;
  uint8_t      io_map_[4096]{};
  uint8_t      group_{0};
  bool         soem_running_{false};

  std::vector<csv_rxpdo_t *> rxpdo_;
  std::vector<csv_txpdo_t *> txpdo_;

  rclcpp::Logger logger_{rclcpp::get_logger("MyCobotCSV")};

  /* ============================================================
   * Bring-up sequence (called by on_activate, in this order)
   * ============================================================ */
  BringupResult init_soem();
  BringupResult reach_safe_op();
  BringupResult reach_operational();
  BringupResult enable_drive(int slave_idx);

  /* L4: factored from enable_drive's three repeated state-transition blocks.
     Writes `controlword`, polls statusword for `expected_state` (under
     `state_mask`), returns true on success or false if retry budget exhausted.
     `retries_used_out` (optional) receives how many cycles were actually used. */
  bool cia402_transition(int slave_idx,
                         uint16_t controlword,
                         uint16_t expected_state,
                         uint16_t state_mask,
                         int max_retries,
                         int * retries_used_out = nullptr);

  /* ============================================================
   * Tear-down (called by on_deactivate)
   * ============================================================ */
  void disable_drive(int slave_idx);
  void close_soem();

  /* ============================================================
   * Per-cycle helper
   * ============================================================ */
  int  fieldbus_roundtrip();

  /* ============================================================
   * Statics — invoked by SOEM during PRE_OP → SAFE_OP transition
   *
   * slave_csv_config orchestrates the configure_* helpers below.
   * Each configure_* corresponds to one numbered step in the
   * original simple_ng.c laifual_csv_config.
   * ============================================================ */
  static const char * cia402_state_str(uint16_t status);

  static int slave_csv_config(ecx_contextt * context, uint16_t slave);

  /* L4: factored from slave_csv_config — each helper writes a
     coherent group of SDOs corresponding to one config step.
     Return 1 on success, -1 on SDO failure (matches SOEM convention). */
  static int configure_csv_mode(ecx_contextt * context, uint16_t slave);
  static int configure_interpolation_period(ecx_contextt * context, uint16_t slave);
  static int configure_txpdo(ecx_contextt * context, uint16_t slave);
  static int configure_rxpdo(ecx_contextt * context, uint16_t slave);
  static int configure_brake_output(ecx_contextt * context, uint16_t slave);
  static int configure_sync_managers(ecx_contextt * context, uint16_t slave);
  static int configure_free_run_sync(ecx_contextt * context, uint16_t slave);
};

}  // namespace mycobot_csv
