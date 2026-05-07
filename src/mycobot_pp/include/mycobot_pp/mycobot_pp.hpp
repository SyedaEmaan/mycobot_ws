/**
 * mycobot_pp.hpp — CiA 402 Profile Position mode (mode 1) over EtherCAT/SOEM.
 *
 * Sibling of mycobot_csp. Same EtherCAT bring-up + CiA 402 state machine,
 * but uses Profile Position mode because this drive's firmware does NOT
 * implement CSP (mode 8) despite the ESI XML claiming so.
 *
 * Key differences from CSP:
 *   - Mode-of-operation (0x6060) = 1 (PP) instead of 8 (CSP).
 *   - Drive does the trajectory generation, master just supplies endpoints.
 *   - Each new target_position must be committed by toggling bit 4
 *     ("New set-point") of the controlword. Bit 5 ("Change set
 *     immediately") is held = 1 so a new target overrides any in-progress
 *     motion. Bit 6 = 0 (absolute targets).
 *   - Profile velocity (0x6081), profile acceleration (0x6083), and
 *     profile deceleration (0x6084) shape the trajectory; the plugin
 *     writes generous defaults during slave_pp_config.
 *
 * == File overview ==
 *
 *   Public API: hardware_interface::SystemInterface
 *     ↳ on_init / on_activate / on_deactivate
 *     ↳ export_state_interfaces / export_command_interfaces
 *     ↳ read / write
 *
 *   Bring-up sequence (called from on_activate):
 *     init_soem()           — ec_init, config_init, PDO mapping; one
 *                             early roundtrip seeds last_position_counts_
 *     reach_safe_op()       — EtherCAT state PRE_OP → SAFE_OP
 *     reach_operational()   — prime PDOs (target = current count), SAFE_OP → OPERATIONAL
 *     enable_drive(s)       — CiA 402: SWITCH_ON_DISABLED → OPERATION_ENABLED;
 *                             also re-writes 0x6060=1 post-enable and verifies
 *                             0x6061 == 1.
 *
 *   Per-cycle (called from ControllerManager update loop):
 *     read()                — copy txpdo into hw_positions_/hw_velocities_
 *     write()               — copy hw_position_commands_ into rxpdo +
 *                             toggle controlword bit 4 on target change +
 *                             roundtrip
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

namespace mycobot_pp {

/* ================================================================== */
/* PDO structures — same byte layout as CSP variant                    */
/* ================================================================== */
typedef struct __attribute__((packed)) {
    uint16_t controlword;
    int32_t  target_position;     /* 0x607A — same in PP and CSP */
    uint16_t touch_probe_func;
    uint32_t physical_output;
} pp_rxpdo_t;

typedef struct __attribute__((packed)) {
    uint16_t statusword;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    uint16_t touch_probe_stat;
    int32_t  touch_probe_pos1;
    int32_t  touch_probe_pos2;
    uint32_t physical_inputs;
} pp_txpdo_t;

/* ================================================================== */
/* CiA 402 controlword bits & composed values                         */
/* ================================================================== */
namespace cw {
    constexpr uint16_t SWITCH_ON          = 1u << 0;
    constexpr uint16_t ENABLE_VOLTAGE     = 1u << 1;
    constexpr uint16_t QUICK_STOP         = 1u << 2;
    constexpr uint16_t ENABLE_OPERATION   = 1u << 3;
    constexpr uint16_t NEW_SETPOINT       = 1u << 4;  /* PP-specific bit 4 */
    constexpr uint16_t CHANGE_IMMEDIATE   = 1u << 5;  /* PP-specific bit 5 */
    constexpr uint16_t ABS_REL            = 1u << 6;  /* PP-specific bit 6 (0 = absolute) */
    constexpr uint16_t FAULT_RESET        = 1u << 7;

    constexpr uint16_t SHUTDOWN           = QUICK_STOP | ENABLE_VOLTAGE;                      /* 0x06 */
    constexpr uint16_t SWITCH_ON_CMD      = QUICK_STOP | ENABLE_VOLTAGE | SWITCH_ON;          /* 0x07 */
    constexpr uint16_t ENABLE_OP_CMD      = QUICK_STOP | ENABLE_VOLTAGE | SWITCH_ON | ENABLE_OPERATION; /* 0x0F */

    /* PP base = OP enabled + bit 5 (change immediate) + bit 6=0 (absolute). */
    constexpr uint16_t PP_BASE            = ENABLE_OP_CMD | CHANGE_IMMEDIATE; /* 0x2F */
    /* PP with new-setpoint pulse = base | bit 4. */
    constexpr uint16_t PP_NEW_SETPOINT    = PP_BASE | NEW_SETPOINT;           /* 0x3F */
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
    /* PP-specific: bit 12 = "set-point acknowledge" */
    constexpr uint16_t SETPOINT_ACK       = 1u << 12;
    /* Bit 10 = "target reached" (also in CSV/CSP) */
    constexpr uint16_t TARGET_REACHED     = 1u << 10;
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
class MyCobotPP : public hardware_interface::SystemInterface
{
public:
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
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_position_commands_;

  std::vector<int>     joint_to_slave_;
  std::vector<double>  counts_per_rad_;
  std::vector<int32_t> last_position_counts_;
  std::vector<int32_t> last_target_counts_;     /* PP: track target changes for bit-4 toggle */
  std::vector<bool>    setpoint_acked_;         /* PP: setpoint handshake state */

  std::string ifname_;
  size_t      n_joints_{0};

  ecx_contextt context_;
  uint8_t      io_map_[4096]{};
  uint8_t      group_{0};
  bool         soem_running_{false};

  std::vector<pp_rxpdo_t *> rxpdo_;
  std::vector<pp_txpdo_t *> txpdo_;

  rclcpp::Logger logger_{rclcpp::get_logger("MyCobotPP")};

  BringupResult init_soem();
  BringupResult reach_safe_op();
  BringupResult reach_operational();
  BringupResult enable_drive(int slave_idx);

  bool cia402_transition(int slave_idx,
                         uint16_t controlword,
                         uint16_t expected_state,
                         uint16_t state_mask,
                         int max_retries,
                         int * retries_used_out = nullptr);

  void disable_drive(int slave_idx);
  void close_soem();

  int  fieldbus_roundtrip();

  static const char * cia402_state_str(uint16_t status);

  static int slave_pp_config(ecx_contextt * context, uint16_t slave);

  static int configure_pp_mode(ecx_contextt * context, uint16_t slave);
  static int configure_interpolation_period(ecx_contextt * context, uint16_t slave);
  static int configure_txpdo(ecx_contextt * context, uint16_t slave);
  static int configure_rxpdo(ecx_contextt * context, uint16_t slave);
  static int configure_brake_output(ecx_contextt * context, uint16_t slave);
  static int configure_sync_managers(ecx_contextt * context, uint16_t slave);
  static int configure_free_run_sync(ecx_contextt * context, uint16_t slave);
  static int configure_pp_motion_limits(ecx_contextt * context, uint16_t slave);
};

}  // namespace mycobot_pp
