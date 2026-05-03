/**
 * mycobot_csv.cpp — Layer 4 refactor (organizational).
 *
 * == Roadmap ==
 *
 *   1. Forward-declared SDO write helper used by all configure_* (file-local).
 *   2. result_str()             — BringupResult to string for logging.
 *   3. cia402_state_str()       — statusword to human-readable state name.
 *   4. configure_*              — one helper per slave_csv_config step.
 *      slave_csv_config()       — orchestrates the configure_* helpers.
 *   5. fieldbus_roundtrip()     — one PDO send+receive cycle.
 *   6. init_soem()              — ec_init, config_init, PDO mapping.
 *   7. reach_safe_op()          — drive bus to SAFE_OP.
 *   8. reach_operational()      — prime PDOs, drive bus to OPERATIONAL.
 *   9. cia402_transition()      — write CW, poll for expected SW.
 *      enable_drive()           — uses cia402_transition for the 3 transitions.
 *  10. disable_drive() / close_soem()
 *  11. ros2_control SystemInterface lifecycle methods.
 *  12. ros2_control read() / write() per-cycle methods.
 *  13. PLUGINLIB_EXPORT_CLASS at end.
 *
 * == Verification protocol ==
 *
 *   Behavior at runtime is identical to L3. Every SDO write happens at
 *   the same logical moment for the same reason. The diff is purely
 *   about where lines of code live (which function), not what they do.
 */

#include "mycobot_csv/mycobot_csv.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace mycobot_csv {

/* ============================================================================
 * BringupResult → string
 * ========================================================================== */
const char * result_str(BringupResult r)
{
    switch (r) {
        case BringupResult::OK:                       return "OK";
        case BringupResult::EC_INIT_FAILED:           return "EC_INIT_FAILED";
        case BringupResult::NO_SLAVES_FOUND:          return "NO_SLAVES_FOUND";
        case BringupResult::INVALID_SLAVE_INDEX:      return "INVALID_SLAVE_INDEX";
        case BringupResult::PDO_SIZE_MISMATCH:        return "PDO_SIZE_MISMATCH";
        case BringupResult::NULL_PDO_BUFFER:          return "NULL_PDO_BUFFER";
        case BringupResult::SAFE_OP_TIMEOUT:          return "SAFE_OP_TIMEOUT";
        case BringupResult::OPERATIONAL_TIMEOUT:      return "OPERATIONAL_TIMEOUT";
        case BringupResult::FAULT_NOT_CLEARED:        return "FAULT_NOT_CLEARED";
        case BringupResult::SHUTDOWN_FAILED:          return "SHUTDOWN_FAILED";
        case BringupResult::SWITCH_ON_FAILED:         return "SWITCH_ON_FAILED";
        case BringupResult::ENABLE_OPERATION_FAILED:  return "ENABLE_OPERATION_FAILED";
    }
    return "UNKNOWN";
}

/* ============================================================================
 * CiA 402 state decoder (verbatim from simple_ng.c:53-67)
 * ========================================================================== */
const char * MyCobotCSV::cia402_state_str(uint16_t status)
{
    uint16_t masked = status & sw::STATE_MASK;
    if ((masked & 0x004F) == 0x0040)               return "NOT_READY_TO_SWITCH_ON";
    if ((masked & 0x006F) == 0x0060)               return "SWITCH_ON_DISABLED";
    if ((masked & 0x006F) == sw::READY_TO_SWITCH_ON) return "READY_TO_SWITCH_ON";
    if ((masked & 0x006F) == sw::SWITCHED_ON)        return "SWITCHED_ON";
    if ((masked & 0x006F) == sw::OPERATION_ENABLED)  return "OPERATION_ENABLED";
    if ((masked & 0x006F) == 0x0007)               return "QUICK_STOP_ACTIVE";
    if ((masked & 0x004F) == 0x000F)               return "FAULT_REACTION_ACTIVE";
    if ((masked & 0x006F) == 0x0028)               return "FAULT";
    if (status & sw::FAULT_BIT)                    return "FAULT";
    return "UNKNOWN";
}

/* ============================================================================
 * configure_* — slave_csv_config factored into per-step helpers
 *
 * Each helper corresponds to one numbered step in simple_ng.c's
 * laifual_csv_config (lines 107-254). Splitting them out makes:
 *   - the orchestrator (slave_csv_config) read like a step list
 *   - each helper independently testable and replaceable
 *   - the boundary between steps explicit (was just code-comments before)
 *
 * All helpers return 1 on success, -1 on SDO failure (matching the SOEM
 * PO2SOconfig convention). They do NOT log directly because they run
 * inside SOEM's callback path where rclcpp may not be safe.
 * ========================================================================== */

int MyCobotCSV::configure_csv_mode(ecx_contextt * context, uint16_t slave)
{
    /* simple_ng.c:114-126 — set 0x6060 = 9 (CSV) and verify */
    int8_t mode = 9;
    int wc = ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE,
                          sizeof(mode), &mode, EC_TIMEOUTSAFE);
    if (wc <= 0) return -1;
    osal_usleep(timing::MODE_SETTLE_US);

    int8_t mode_rb = 0;
    int sz = sizeof(mode_rb);
    ecx_SDOread(context, slave, 0x6060, 0x00, FALSE, &sz, &mode_rb, EC_TIMEOUTSAFE);
    if (mode_rb != 9) return -1;
    return 1;
}

int MyCobotCSV::configure_interpolation_period(ecx_contextt * context, uint16_t slave)
{
    /* simple_ng.c:128-143 — 5 ms = 5 × 10^-3 */
    int8_t interp_val = 5;
    int8_t interp_idx = -3;
    if (ecx_SDOwrite(context, slave, 0x60C2, 0x01, FALSE,
                     sizeof(interp_val), &interp_val, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x60C2, 0x02, FALSE,
                     sizeof(interp_idx), &interp_idx, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSV::configure_txpdo(ecx_contextt * context, uint16_t slave)
{
    /* simple_ng.c:145-174 — TxPDO 0x1A02 (26 bytes, 8 entries) */
    uint32_t tx_objs[] = {
        0x60410010,  /* Statusword       16-bit */
        0x60640020,  /* Position Actual  32-bit */
        0x60770010,  /* Torque Actual    16-bit */
        0x60F40020,  /* Following Error  32-bit */
        0x60B90010,  /* Touch Probe Stat 16-bit */
        0x60BA0020,  /* Touch Probe Pos1 32-bit */
        0x60BC0020,  /* Touch Probe Pos2 32-bit */
        0x60FD0020   /* Physical Inputs  32-bit */
    };
    const int tx_count = 8;

    uint8_t zero = 0;
    if (ecx_SDOwrite(context, slave, 0x1A02, 0x00, FALSE,
                     sizeof(zero), &zero, EC_TIMEOUTSAFE) <= 0) return -1;
    for (int i = 0; i < tx_count; i++) {
        if (ecx_SDOwrite(context, slave, 0x1A02, i + 1, FALSE,
                         sizeof(tx_objs[i]), &tx_objs[i], EC_TIMEOUTSAFE) <= 0) return -1;
    }
    uint8_t eight = 8;
    if (ecx_SDOwrite(context, slave, 0x1A02, 0x00, FALSE,
                     sizeof(eight), &eight, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSV::configure_rxpdo(ecx_contextt * context, uint16_t slave)
{
    /* simple_ng.c:176-199 — RxPDO 0x1602 (12 bytes, 4 entries, CSV variant) */
    uint32_t rx_objs[] = {
        0x60400010,  /* Controlword      16-bit */
        0x60FF0020,  /* Target Velocity  32-bit  (CSV) */
        0x60B80010,  /* Touch Probe Func 16-bit */
        0x60FE0120   /* Physical Output  32-bit  (subindex 0x01) */
    };
    const int rx_count = 4;

    uint8_t zero = 0;
    if (ecx_SDOwrite(context, slave, 0x1602, 0x00, FALSE,
                     sizeof(zero), &zero, EC_TIMEOUTSAFE) <= 0) return -1;
    for (int i = 0; i < rx_count; i++) {
        if (ecx_SDOwrite(context, slave, 0x1602, i + 1, FALSE,
                         sizeof(rx_objs[i]), &rx_objs[i], EC_TIMEOUTSAFE) <= 0) return -1;
    }
    uint8_t four = 4;
    if (ecx_SDOwrite(context, slave, 0x1602, 0x00, FALSE,
                     sizeof(four), &four, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSV::configure_brake_output(ecx_contextt * context, uint16_t slave)
{
    /* simple_ng.c:200-209 — output mask 0x60FE:02 = 0x01 (allow bit 0) */
    uint32_t output_mask = brake::RELEASED;
    if (ecx_SDOwrite(context, slave, 0x60FE, 0x02, FALSE,
                     sizeof(output_mask), &output_mask, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSV::configure_sync_managers(ecx_contextt * context, uint16_t slave)
{
    /* simple_ng.c:210-235 — assign 0x1602 to SM2 and 0x1A02 to SM3 */
    uint16_t rx_map = 0x1602;
    uint16_t tx_map = 0x1A02;
    uint8_t  zero   = 0;
    uint8_t  one    = 1;

    if (ecx_SDOwrite(context, slave, 0x1C12, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x1C12, 0x01, FALSE, sizeof(rx_map), &rx_map, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x1C12, 0x00, FALSE, sizeof(one), &one, EC_TIMEOUTSAFE) <= 0) return -1;

    if (ecx_SDOwrite(context, slave, 0x1C13, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x1C13, 0x01, FALSE, sizeof(tx_map), &tx_map, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x1C13, 0x00, FALSE, sizeof(one), &one, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSV::configure_free_run_sync(ecx_contextt * context, uint16_t slave)
{
    /* simple_ng.c:237-250 — sync mode 0 (free-run) for both SMs */
    uint16_t sync_free = 0;
    if (ecx_SDOwrite(context, slave, 0x1C32, 0x01, FALSE,
                     sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x1C33, 0x01, FALSE,
                     sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

/* slave_csv_config — orchestrator. Body is now a step list. */
int MyCobotCSV::slave_csv_config(ecx_contextt * context, uint16_t slave)
{
    if (configure_csv_mode(context, slave)              < 0) return -1;
    if (configure_interpolation_period(context, slave)  < 0) return -1;
    if (configure_txpdo(context, slave)                 < 0) return -1;
    if (configure_rxpdo(context, slave)                 < 0) return -1;
    if (configure_brake_output(context, slave)          < 0) return -1;
    if (configure_sync_managers(context, slave)         < 0) return -1;
    if (configure_free_run_sync(context, slave)         < 0) return -1;
    return 1;
}

/* ============================================================================
 * fieldbus_roundtrip — one PDO send/receive cycle (simple_ng.c:88-102)
 * ========================================================================== */
int MyCobotCSV::fieldbus_roundtrip()
{
    ecx_send_processdata(&context_);
    return ecx_receive_processdata(&context_, timing::RECEIVE_TIMEOUT_US);
}

/* ============================================================================
 * init_soem — ec_init + config_init + PDO mapping
 * ========================================================================== */
BringupResult MyCobotCSV::init_soem()
{
    std::memset(&context_, 0, sizeof(context_));
    std::memset(io_map_, 0, sizeof(io_map_));

    RCLCPP_INFO(logger_, "EtherCAT bring-up: ec_init on '%s'", ifname_.c_str());
    if (!ecx_init(&context_, ifname_.c_str())) {
        RCLCPP_ERROR(logger_,
            "ec_init failed on '%s'. Check interface name and CAP_NET_RAW.",
            ifname_.c_str());
        return BringupResult::EC_INIT_FAILED;
    }

    if (ecx_config_init(&context_) <= 0) {
        RCLCPP_ERROR(logger_, "No EtherCAT slaves found on '%s'.", ifname_.c_str());
        ecx_close(&context_);
        return BringupResult::NO_SLAVES_FOUND;
    }
    RCLCPP_INFO(logger_, "EtherCAT: %d slave(s) discovered.", context_.slavecount);

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        if (s < 1 || s > context_.slavecount) {
            RCLCPP_ERROR(logger_,
                "Joint '%s' references slave %d but bus has %d slaves.",
                info_.joints[i].name.c_str(), s, context_.slavecount);
            ecx_close(&context_);
            return BringupResult::INVALID_SLAVE_INDEX;
        }
        context_.slavelist[s].PO2SOconfig = &MyCobotCSV::slave_csv_config;
        RCLCPP_DEBUG(logger_, "Attached PO2SOconfig hook to slave %d (joint '%s').",
                     s, info_.joints[i].name.c_str());
    }

    ecx_config_map_group(&context_, io_map_, group_);

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        uint16_t wd_off = 0;
        ecx_FPWR(&context_.port, context_.slavelist[s].configadr,
                 0x0422, sizeof(wd_off), &wd_off, EC_TIMEOUTRET);
    }

    ec_groupt * grp = context_.grouplist + group_;
    if (grp->Obytes != sizeof(csv_rxpdo_t)) {
        RCLCPP_ERROR(logger_,
            "Output PDO size mismatch: mapped %d bytes, expected %zu (csv_rxpdo_t). "
            "Did slave_csv_config() succeed on every slave?",
            grp->Obytes, sizeof(csv_rxpdo_t));
        ecx_close(&context_);
        return BringupResult::PDO_SIZE_MISMATCH;
    }
    if (grp->Ibytes != sizeof(csv_txpdo_t)) {
        RCLCPP_ERROR(logger_,
            "Input PDO size mismatch: mapped %d bytes, expected %zu (csv_txpdo_t). "
            "Did slave_csv_config() succeed on every slave?",
            grp->Ibytes, sizeof(csv_txpdo_t));
        ecx_close(&context_);
        return BringupResult::PDO_SIZE_MISMATCH;
    }
    RCLCPP_DEBUG(logger_, "PDO sizes verified: out=%zu bytes, in=%zu bytes.",
                 sizeof(csv_rxpdo_t), sizeof(csv_txpdo_t));

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        rxpdo_[s] = reinterpret_cast<csv_rxpdo_t *>(context_.slavelist[s].outputs);
        txpdo_[s] = reinterpret_cast<csv_txpdo_t *>(context_.slavelist[s].inputs);
        if (!rxpdo_[s] || !txpdo_[s]) {
            RCLCPP_ERROR(logger_, "Slave %d: null PDO buffer after mapping.", s);
            ecx_close(&context_);
            return BringupResult::NULL_PDO_BUFFER;
        }
    }

    return BringupResult::OK;
}

/* ============================================================================
 * reach_safe_op / reach_operational
 * ========================================================================== */
BringupResult MyCobotCSV::reach_safe_op()
{
    context_.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&context_, 0);

    for (int retries = 0; retries < timing::SAFE_OP_RETRIES; ++retries) {
        ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        ecx_readstate(&context_);
        if (context_.slavelist[0].state >= EC_STATE_SAFE_OP) {
            RCLCPP_DEBUG(logger_, "SAFE_OP reached after %d retries.", retries);
            return BringupResult::OK;
        }
        for (int s = 1; s <= context_.slavecount; ++s) {
            if (context_.slavelist[s].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR) ||
                context_.slavelist[s].state == (EC_STATE_PRE_OP  + EC_STATE_ERROR)) {
                RCLCPP_WARN(logger_,
                    "Slave %d in error state during SAFE_OP transition (state=0x%02X), acknowledging.",
                    s, context_.slavelist[s].state);
                context_.slavelist[s].state =
                    (context_.slavelist[s].state & 0x0F) + EC_STATE_ACK;
                ecx_writestate(&context_, s);
                osal_usleep(timing::ERROR_ACK_SETTLE_US);
            }
        }
    }
    return (context_.slavelist[0].state >= EC_STATE_SAFE_OP)
         ? BringupResult::OK
         : BringupResult::SAFE_OP_TIMEOUT;
}

BringupResult MyCobotCSV::reach_operational()
{
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        rxpdo_[s]->controlword      = cw::SHUTDOWN;
        rxpdo_[s]->target_velocity  = 0;
        rxpdo_[s]->touch_probe_func = 0;
        rxpdo_[s]->physical_output  = brake::ENGAGED;
    }
    for (int i = 0; i < timing::PRIMING_INITIAL_CYCLES; ++i) fieldbus_roundtrip();
    for (int i = 0; i < timing::PRIMING_FINAL_CYCLES;   ++i) fieldbus_roundtrip();
    RCLCPP_DEBUG(logger_, "PDO buffer primed (%d initial + %d final cycles).",
                 timing::PRIMING_INITIAL_CYCLES, timing::PRIMING_FINAL_CYCLES);

    context_.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&context_, 0);

    for (int retries = 0; retries < timing::OPERATIONAL_RETRIES; ++retries) {
        fieldbus_roundtrip();
        ecx_statecheck(&context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
        ecx_readstate(&context_);
        bool all_op = true;
        for (size_t j = 0; j < n_joints_; ++j) {
            if (context_.slavelist[joint_to_slave_[j]].state != EC_STATE_OPERATIONAL) {
                all_op = false;
                break;
            }
        }
        if (all_op) {
            RCLCPP_DEBUG(logger_, "OPERATIONAL reached after %d retries.", retries);
            return BringupResult::OK;
        }
    }
    return BringupResult::OPERATIONAL_TIMEOUT;
}

/* ============================================================================
 * cia402_transition — one CiA 402 state transition
 *
 * Factored out of enable_drive's three near-identical blocks. Writes
 * `controlword`, polls statusword for `expected_state` (after masking
 * with `state_mask`), returns true on success.
 *
 * `retries_used_out` (optional) is filled with the number of poll cycles
 * actually used — useful for log messages distinguishing "timed out
 * instantly" from "almost made it".
 * ========================================================================== */
bool MyCobotCSV::cia402_transition(int slave_idx,
                                   uint16_t controlword,
                                   uint16_t expected_state,
                                   uint16_t state_mask,
                                   int max_retries,
                                   int * retries_used_out)
{
    rxpdo_[slave_idx]->controlword = controlword;

    int retries;
    uint16_t status = 0;
    for (retries = 0; retries < max_retries; ++retries) {
        fieldbus_roundtrip();
        status = txpdo_[slave_idx]->statusword;
        if ((status & state_mask) == expected_state) break;
        osal_usleep(timing::POLL_CYCLE_US);
    }
    if (retries_used_out) *retries_used_out = retries;
    return (status & state_mask) == expected_state;
}

/* ============================================================================
 * enable_drive — CiA 402 SWITCH_ON_DISABLED → OPERATION_ENABLED
 *
 * Now structured as four discrete steps:
 *   (1) optional fault reset
 *   (2) Shutdown            (CW=0x06 → READY_TO_SWITCH_ON)
 *   (3) Switch On           (CW=0x07 → SWITCHED_ON)
 *   (4) Enable Operation    (CW=0x0F → OPERATION_ENABLED)
 *   (5) brake release stream
 * ========================================================================== */
BringupResult MyCobotCSV::enable_drive(int s)
{
    fieldbus_roundtrip();
    uint16_t status = txpdo_[s]->statusword;
    RCLCPP_DEBUG(logger_, "Slave %d enable entry: sw=0x%04X (%s)",
                 s, status, cia402_state_str(status));

    /* (1) Fault reset — only if drive is currently in FAULT */
    if (status & sw::FAULT_BIT) {
        RCLCPP_WARN(logger_, "Slave %d entered enable() in FAULT state (sw=0x%04X), resetting.",
                    s, status);
        rxpdo_[s]->controlword = cw::FAULT_RESET;
        int retries;
        for (retries = 0; retries < timing::FAULT_RESET_RETRIES; ++retries) {
            fieldbus_roundtrip();
            status = txpdo_[s]->statusword;
            if (!(status & sw::FAULT_BIT)) break;
            osal_usleep(timing::POLL_CYCLE_US);
        }
        if (status & sw::FAULT_BIT) {
            RCLCPP_ERROR(logger_,
                "Slave %d: fault did not clear after %d retries (sw=0x%04X). Check drive's error register (0x603F).",
                s, timing::FAULT_RESET_RETRIES, status);
            return BringupResult::FAULT_NOT_CLEARED;
        }
    }

    int retries = 0;

    /* (2) Shutdown → READY_TO_SWITCH_ON */
    if (!cia402_transition(s, cw::SHUTDOWN, sw::READY_TO_SWITCH_ON, sw::STATE_MASK,
                           timing::CIA402_TRANSITION_RETRIES, &retries)) {
        status = txpdo_[s]->statusword;
        RCLCPP_ERROR(logger_,
            "Slave %d: failed Shutdown after %d/%d retries. sw=0x%04X (%s)",
            s, retries, timing::CIA402_TRANSITION_RETRIES, status, cia402_state_str(status));
        return BringupResult::SHUTDOWN_FAILED;
    }
    RCLCPP_DEBUG(logger_, "Slave %d: Shutdown OK after %d retries.", s, retries);

    /* (3) Switch On → SWITCHED_ON */
    if (!cia402_transition(s, cw::SWITCH_ON_CMD, sw::SWITCHED_ON, sw::STATE_MASK,
                           timing::CIA402_TRANSITION_RETRIES, &retries)) {
        status = txpdo_[s]->statusword;
        RCLCPP_ERROR(logger_,
            "Slave %d: failed Switch On after %d/%d retries. sw=0x%04X (%s)",
            s, retries, timing::CIA402_TRANSITION_RETRIES, status, cia402_state_str(status));
        return BringupResult::SWITCH_ON_FAILED;
    }
    RCLCPP_DEBUG(logger_, "Slave %d: Switch On OK after %d retries.", s, retries);

    /* (4) Enable Operation → OPERATION_ENABLED */
    if (!cia402_transition(s, cw::ENABLE_OP_CMD, sw::OPERATION_ENABLED, sw::STATE_MASK,
                           timing::CIA402_TRANSITION_RETRIES, &retries)) {
        status = txpdo_[s]->statusword;
        RCLCPP_ERROR(logger_,
            "Slave %d: failed Enable Operation after %d/%d retries. sw=0x%04X (%s)",
            s, retries, timing::CIA402_TRANSITION_RETRIES, status, cia402_state_str(status));
        return BringupResult::ENABLE_OPERATION_FAILED;
    }
    RCLCPP_DEBUG(logger_, "Slave %d: Enable Operation OK after %d retries.", s, retries);

    /* (5) Brake release — stream PDOs for ~100 ms with brake bit set */
    for (int i = 0; i < timing::BRAKE_RELEASE_CYCLES; ++i) {
        rxpdo_[s]->controlword     = cw::ENABLE_OP_CMD;
        rxpdo_[s]->target_velocity = 0;
        rxpdo_[s]->physical_output = brake::RELEASED;
        fieldbus_roundtrip();
        osal_usleep(timing::POLL_CYCLE_US);
    }

    RCLCPP_INFO(logger_,
        "Slave %d: OPERATION_ENABLED, brake released. position_actual=%d counts.",
        s, txpdo_[s]->position_actual);
    return BringupResult::OK;
}

/* ============================================================================
 * disable_drive / close_soem
 * ========================================================================== */
void MyCobotCSV::disable_drive(int s)
{
    if (!txpdo_[s] || !rxpdo_[s]) return;

    RCLCPP_DEBUG(logger_, "Slave %d: disable_drive() entry (sw=0x%04X / %s)",
                 s, txpdo_[s]->statusword, cia402_state_str(txpdo_[s]->statusword));

    rxpdo_[s]->target_velocity = 0;
    rxpdo_[s]->controlword     = cw::ENABLE_OP_CMD;
    rxpdo_[s]->physical_output = brake::RELEASED;
    fieldbus_roundtrip();
    osal_usleep(timing::DRIVE_DISABLE_US);

    rxpdo_[s]->physical_output = brake::ENGAGED;
    rxpdo_[s]->controlword     = cw::SHUTDOWN;
    fieldbus_roundtrip();
    fieldbus_roundtrip();
    osal_usleep(timing::DRIVE_DISABLE_US);

    RCLCPP_INFO(logger_, "Slave %d: disabled, brake engaged.", s);
}

void MyCobotCSV::close_soem()
{
    if (!soem_running_) return;
    context_.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&context_, 0);
    ecx_close(&context_);
    soem_running_ = false;
    RCLCPP_INFO(logger_, "EtherCAT socket closed.");
}

/* ============================================================================
 * ros2_control SystemInterface lifecycle methods
 * ========================================================================== */
hardware_interface::CallbackReturn MyCobotCSV::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto it = info_.hardware_parameters.find("ifname");
    if (it == info_.hardware_parameters.end()) {
        RCLCPP_ERROR(logger_, "Missing required <param name=\"ifname\"> in URDF.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    ifname_ = it->second;

    n_joints_ = info_.joints.size();
    hw_positions_.assign(n_joints_, 0.0);
    hw_velocities_.assign(n_joints_, 0.0);
    hw_velocity_commands_.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
    joint_to_slave_.assign(n_joints_, 0);
    counts_per_rad_.assign(n_joints_, 0.0);
    last_position_counts_.assign(n_joints_, 0);

    int max_slave = 0;
    for (size_t i = 0; i < n_joints_; ++i) {
        const auto & j = info_.joints[i];

        auto it_s = j.parameters.find("slave_index");
        auto it_c = j.parameters.find("counts_per_rad");
        if (it_s == j.parameters.end() || it_c == j.parameters.end()) {
            RCLCPP_ERROR(logger_,
                "Joint '%s' is missing required <param>: slave_index AND counts_per_rad",
                j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        joint_to_slave_[i] = std::stoi(it_s->second);
        counts_per_rad_[i] = std::stod(it_c->second);
        if (joint_to_slave_[i] > max_slave) max_slave = joint_to_slave_[i];

        bool has_vel_cmd = false;
        for (const auto & ci : j.command_interfaces) {
            if (ci.name == hardware_interface::HW_IF_VELOCITY) has_vel_cmd = true;
        }
        if (!has_vel_cmd) {
            RCLCPP_ERROR(logger_,
                "Joint '%s' must declare a 'velocity' command interface (CSV mode).",
                j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(logger_, "Joint '%s' configured: slave=%d, counts_per_rad=%.1f",
                     j.name.c_str(), joint_to_slave_[i], counts_per_rad_[i]);
    }

    rxpdo_.assign(max_slave + 1, nullptr);
    txpdo_.assign(max_slave + 1, nullptr);

    RCLCPP_DEBUG(logger_,
        "MyCobotCSV initialised: ifname='%s', %zu joint(s), max slave %d.",
        ifname_.c_str(), n_joints_, max_slave);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyCobotCSV::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    for (size_t i = 0; i < n_joints_; ++i) {
        ifaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        ifaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    return ifaces;
}

std::vector<hardware_interface::CommandInterface>
MyCobotCSV::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    for (size_t i = 0; i < n_joints_; ++i) {
        ifaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY,
                            &hw_velocity_commands_[i]);
    }
    return ifaces;
}

hardware_interface::CallbackReturn MyCobotCSV::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Activating MyCobotCSV (%zu joint(s) on '%s')...",
                n_joints_, ifname_.c_str());

    if (auto r = init_soem(); r != BringupResult::OK) {
        RCLCPP_ERROR(logger_, "Activation failed in init_soem(): %s", result_str(r));
        return hardware_interface::CallbackReturn::ERROR;
    }
    soem_running_ = true;

    if (auto r = reach_safe_op(); r != BringupResult::OK) {
        RCLCPP_ERROR(logger_, "Activation failed reaching SAFE_OP: %s", result_str(r));
        close_soem();
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (auto r = reach_operational(); r != BringupResult::OK) {
        RCLCPP_ERROR(logger_, "Activation failed reaching OPERATIONAL: %s", result_str(r));
        close_soem();
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        if (auto r = enable_drive(s); r != BringupResult::OK) {
            RCLCPP_ERROR(logger_,
                "Drive enable failed for joint '%s' (slave %d): %s",
                info_.joints[i].name.c_str(), s, result_str(r));
            close_soem();
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        int32_t cnt = txpdo_[s]->position_actual;
        last_position_counts_[i]  = cnt;
        hw_positions_[i]          = static_cast<double>(cnt) / counts_per_rad_[i];
        hw_velocities_[i]         = 0.0;
        hw_velocity_commands_[i]  = 0.0;
    }

    RCLCPP_INFO(logger_, "MyCobotCSV ACTIVE — %zu drive(s) operating in CSV mode.",
                n_joints_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotCSV::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Deactivating MyCobotCSV...");
    for (size_t i = 0; i < n_joints_; ++i) {
        disable_drive(joint_to_slave_[i]);
    }
    close_soem();
    return hardware_interface::CallbackReturn::SUCCESS;
}

/* ============================================================================
 * ros2_control read / write — per-cycle methods
 * ========================================================================== */
hardware_interface::return_type MyCobotCSV::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    const double dt = period.seconds() > 1e-6 ? period.seconds() : 0.005;

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        int32_t cnt = txpdo_[s]->position_actual;

        hw_positions_[i]  = static_cast<double>(cnt) / counts_per_rad_[i];
        hw_velocities_[i] =
            (static_cast<double>(cnt - last_position_counts_[i]) / counts_per_rad_[i]) / dt;
        last_position_counts_[i] = cnt;
    }

    static rclcpp::Clock dbg_clock(RCL_STEADY_TIME);
    if (n_joints_ > 0) {
        RCLCPP_DEBUG_THROTTLE(logger_, dbg_clock, 500,
            "j0 pos=%.4f rad vel=%.4f rad/s cnt=%d",
            hw_positions_[0], hw_velocities_[0], last_position_counts_[0]);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobotCSV::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];

        double cmd_rad_per_sec = hw_velocity_commands_[i];
        if (!std::isfinite(cmd_rad_per_sec)) cmd_rad_per_sec = 0.0;

        int32_t cnt_per_sec = static_cast<int32_t>(
            std::lround(cmd_rad_per_sec * counts_per_rad_[i]));

        rxpdo_[s]->target_velocity  = cnt_per_sec;
        rxpdo_[s]->controlword      = cw::ENABLE_OP_CMD;
        rxpdo_[s]->physical_output  = brake::RELEASED;
        rxpdo_[s]->touch_probe_func = 0;
    }

    int wkc = fieldbus_roundtrip();
    if (wkc <= 0) {
        static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(logger_, throttle_clock, 1000,
            "PDO roundtrip wkc=%d (link issue or slave dropped from OP).", wkc);
    }
    return hardware_interface::return_type::OK;
}

}  // namespace mycobot_csv

PLUGINLIB_EXPORT_CLASS(mycobot_csv::MyCobotCSV,
                       hardware_interface::SystemInterface)
