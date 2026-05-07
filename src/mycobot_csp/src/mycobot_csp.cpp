/**
 * mycobot_csp.cpp — CiA 402 CSP plugin (sibling of the verified CSV baseline).
 *
 * == Roadmap ==
 *
 *   1. result_str()             — BringupResult to string for logging.
 *   2. cia402_state_str()       — statusword to human-readable state name.
 *   3. configure_*              — one helper per slave_csp_config step.
 *      slave_csp_config()       — orchestrates the configure_* helpers.
 *   4. fieldbus_roundtrip()     — one PDO send+receive cycle.
 *   5. init_soem()              — ec_init, config_init, PDO mapping; one
 *                                 early roundtrip seeds last_position_counts_
 *                                 so subsequent priming/brake-release/disable
 *                                 sites can write target_position = current
 *                                 count instead of 0 (which under CSP would
 *                                 race the joint to encoder origin).
 *   6. reach_safe_op()          — drive bus to SAFE_OP.
 *   7. reach_operational()      — prime PDOs (target = cached count), drive bus to OP.
 *   8. cia402_transition()      — write CW, poll for expected SW.
 *      enable_drive()           — uses cia402_transition for the 3 transitions.
 *   9. disable_drive() / close_soem()
 *  10. ros2_control SystemInterface lifecycle methods.
 *  11. ros2_control read() / write() per-cycle methods.
 *  12. PLUGINLIB_EXPORT_CLASS at end.
 */

#include "mycobot_csp/mycobot_csp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace mycobot_csp {

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
 * CiA 402 state decoder
 * ========================================================================== */
const char * MyCobotCSP::cia402_state_str(uint16_t status)
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
 * configure_* — slave_csp_config factored into per-step helpers
 * ========================================================================== */

int MyCobotCSP::configure_csp_mode(ecx_contextt * context, uint16_t slave)
{
    /* Set 0x6060 = 8 (CSP) and verify. CSV uses 9; this is the one-byte
     * difference that flips the drive into Cyclic Synchronous Position. */
    int8_t mode = 8;
    int wc = ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE,
                          sizeof(mode), &mode, EC_TIMEOUTSAFE);
    if (wc <= 0) return -1;
    osal_usleep(timing::MODE_SETTLE_US);

    int8_t mode_rb = 0;
    int sz = sizeof(mode_rb);
    ecx_SDOread(context, slave, 0x6060, 0x00, FALSE, &sz, &mode_rb, EC_TIMEOUTSAFE);
    if (mode_rb != 8) return -1;

    /* Diagnostic: read back several CSP-relevant objects so we can see
     * what the drive's effective configuration is. Failures here are
     * non-fatal — we still return success. */
    int8_t mode_display = -99;
    sz = sizeof(mode_display);
    ecx_SDOread(context, slave, 0x6061, 0x00, FALSE, &sz, &mode_display, EC_TIMEOUTSAFE);

    uint32_t max_motor_speed = 0xDEADBEEF;
    sz = sizeof(max_motor_speed);
    ecx_SDOread(context, slave, 0x6080, 0x00, FALSE, &sz, &max_motor_speed, EC_TIMEOUTSAFE);

    uint32_t pos_window = 0xDEADBEEF;
    sz = sizeof(pos_window);
    ecx_SDOread(context, slave, 0x6067, 0x00, FALSE, &sz, &pos_window, EC_TIMEOUTSAFE);

    uint32_t ferr_window = 0xDEADBEEF;
    sz = sizeof(ferr_window);
    ecx_SDOread(context, slave, 0x6065, 0x00, FALSE, &sz, &ferr_window, EC_TIMEOUTSAFE);

    uint16_t ferr_timeout = 0xDEAD;
    sz = sizeof(ferr_timeout);
    ecx_SDOread(context, slave, 0x6066, 0x00, FALSE, &sz, &ferr_timeout, EC_TIMEOUTSAFE);

    uint32_t supported_modes = 0xDEADBEEFu;
    sz = sizeof(supported_modes);
    ecx_SDOread(context, slave, 0x6502, 0x00, FALSE, &sz, &supported_modes, EC_TIMEOUTSAFE);

    uint16_t error_code = 0xDEAD;
    sz = sizeof(error_code);
    ecx_SDOread(context, slave, 0x603F, 0x00, FALSE, &sz, &error_code, EC_TIMEOUTSAFE);

    /* Cannot RCLCPP_INFO from a static SDO callback (no logger access),
     * so stash via printf to stderr. */
    fprintf(stderr,
        "[mycobot_csp slave=%u CSP cfg readback] "
        "mode_display(0x6061)=%d  "
        "max_motor_speed(0x6080)=%u  "
        "pos_window(0x6067)=%u  "
        "ferr_window(0x6065)=%u  "
        "ferr_timeout(0x6066)=%u  "
        "supported_modes(0x6502)=0x%08X  "
        "error_code(0x603F)=0x%04X\n",
        (unsigned)slave, (int)mode_display,
        max_motor_speed, pos_window, ferr_window, ferr_timeout,
        supported_modes, error_code);

    /* Write a generous max_motor_speed (counts/sec) to make sure CSP
     * isn't being silently clamped. ~5 rad/s × 2,085,932 counts/rad
     * = ~10,400,000 counts/sec. */
    uint32_t mms = 10000000;
    ecx_SDOwrite(context, slave, 0x6080, 0x00, FALSE, sizeof(mms), &mms, EC_TIMEOUTSAFE);

    /* Disable following-error trip (use max value). Some drives ship with
     * window=0 which can mean "no tolerance" → silent rejection. */
    uint32_t big_window = 0xFFFFFFFFu;
    ecx_SDOwrite(context, slave, 0x6065, 0x00, FALSE, sizeof(big_window), &big_window, EC_TIMEOUTSAFE);

    uint16_t big_timeout = 0xFFFFu;
    ecx_SDOwrite(context, slave, 0x6066, 0x00, FALSE, sizeof(big_timeout), &big_timeout, EC_TIMEOUTSAFE);

    return 1;
}

int MyCobotCSP::configure_interpolation_period(ecx_contextt * context, uint16_t slave)
{
    /* 5 ms = 5 × 10^-3 — same cadence as CSV. The XML auto-writes 2×10^-3
     * on PreOP→SafeOP, but we re-write 5 ms here so the controller_manager
     * update_rate (200 Hz / 5 ms) matches the drive's interpolation cadence. */
    int8_t interp_val = 5;
    int8_t interp_idx = -3;
    if (ecx_SDOwrite(context, slave, 0x60C2, 0x01, FALSE,
                     sizeof(interp_val), &interp_val, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x60C2, 0x02, FALSE,
                     sizeof(interp_idx), &interp_idx, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSP::configure_txpdo(ecx_contextt * context, uint16_t slave)
{
    /* TxPDO 0x1A02 — same 8 entries as CSV. Position feedback is the same
     * regardless of mode; the XML's CSP-default 0x1A02 also includes these. */
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

int MyCobotCSP::configure_rxpdo(ecx_contextt * context, uint16_t slave)
{
    /* RxPDO 0x1602 — CSP variant: entry 2 is 0x607A target_position
     * (CSV used 0x60FF target_velocity). Layout still 12 bytes, 4 entries. */
    uint32_t rx_objs[] = {
        0x60400010,  /* Controlword      16-bit */
        0x607A0020,  /* Target Position  32-bit  (CSP) */
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

int MyCobotCSP::configure_brake_output(ecx_contextt * context, uint16_t slave)
{
    uint32_t output_mask = brake::RELEASED;
    if (ecx_SDOwrite(context, slave, 0x60FE, 0x02, FALSE,
                     sizeof(output_mask), &output_mask, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSP::configure_sync_managers(ecx_contextt * context, uint16_t slave)
{
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

int MyCobotCSP::configure_free_run_sync(ecx_contextt * context, uint16_t slave)
{
    uint16_t sync_free = 0;
    if (ecx_SDOwrite(context, slave, 0x1C32, 0x01, FALSE,
                     sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x1C33, 0x01, FALSE,
                     sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotCSP::slave_csp_config(ecx_contextt * context, uint16_t slave)
{
    if (configure_csp_mode(context, slave)              < 0) return -1;
    if (configure_interpolation_period(context, slave)  < 0) return -1;
    if (configure_txpdo(context, slave)                 < 0) return -1;
    if (configure_rxpdo(context, slave)                 < 0) return -1;
    if (configure_brake_output(context, slave)          < 0) return -1;
    if (configure_sync_managers(context, slave)         < 0) return -1;
    if (configure_free_run_sync(context, slave)         < 0) return -1;
    return 1;
}

/* ============================================================================
 * fieldbus_roundtrip — one PDO send/receive cycle
 * ========================================================================== */
int MyCobotCSP::fieldbus_roundtrip()
{
    ecx_send_processdata(&context_);
    return ecx_receive_processdata(&context_, timing::RECEIVE_TIMEOUT_US);
}

/* ============================================================================
 * init_soem — ec_init + config_init + PDO mapping + seed last_position_counts_
 * ========================================================================== */
BringupResult MyCobotCSP::init_soem()
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
        context_.slavelist[s].PO2SOconfig = &MyCobotCSP::slave_csp_config;
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
    if (grp->Obytes != sizeof(csp_rxpdo_t)) {
        RCLCPP_ERROR(logger_,
            "Output PDO size mismatch: mapped %d bytes, expected %zu (csp_rxpdo_t). "
            "Did slave_csp_config() succeed on every slave?",
            grp->Obytes, sizeof(csp_rxpdo_t));
        ecx_close(&context_);
        return BringupResult::PDO_SIZE_MISMATCH;
    }
    if (grp->Ibytes != sizeof(csp_txpdo_t)) {
        RCLCPP_ERROR(logger_,
            "Input PDO size mismatch: mapped %d bytes, expected %zu (csp_txpdo_t). "
            "Did slave_csp_config() succeed on every slave?",
            grp->Ibytes, sizeof(csp_txpdo_t));
        ecx_close(&context_);
        return BringupResult::PDO_SIZE_MISMATCH;
    }
    RCLCPP_DEBUG(logger_, "PDO sizes verified: out=%zu bytes, in=%zu bytes.",
                 sizeof(csp_rxpdo_t), sizeof(csp_txpdo_t));

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        rxpdo_[s] = reinterpret_cast<csp_rxpdo_t *>(context_.slavelist[s].outputs);
        txpdo_[s] = reinterpret_cast<csp_txpdo_t *>(context_.slavelist[s].inputs);
        if (!rxpdo_[s] || !txpdo_[s]) {
            RCLCPP_ERROR(logger_, "Slave %d: null PDO buffer after mapping.", s);
            ecx_close(&context_);
            return BringupResult::NULL_PDO_BUFFER;
        }
    }

    /* CSP safety: seed last_position_counts_ with the live encoder count so
     * that priming, brake-release, and disable_drive can write the current
     * count into target_position rather than 0. Under CSP, target=0 means
     * "snap to encoder origin" — dangerous on a real arm. We do one early
     * roundtrip to populate txpdo->position_actual; the drive is still in
     * SAFE_OP at this point, so it accepts process data but won't act on
     * the controlword (which we've zeroed via memset). */
    fieldbus_roundtrip();
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        last_position_counts_[i] = txpdo_[s]->position_actual;
        RCLCPP_DEBUG(logger_,
            "Slave %d: seeded last_position_counts_=%d for CSP priming.",
            s, last_position_counts_[i]);
    }

    return BringupResult::OK;
}

/* ============================================================================
 * reach_safe_op / reach_operational
 * ========================================================================== */
BringupResult MyCobotCSP::reach_safe_op()
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

BringupResult MyCobotCSP::reach_operational()
{
    /* CSP safety: target_position = current encoder count (NOT 0).
     * Without this, the moment the drive transitions to OPERATIONAL it
     * sees a fresh target of 0 counts and will try to slam the joint to
     * encoder origin. */
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        rxpdo_[s]->controlword      = cw::SHUTDOWN;
        rxpdo_[s]->target_position  = last_position_counts_[i];
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
 * ========================================================================== */
bool MyCobotCSP::cia402_transition(int slave_idx,
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
 * ========================================================================== */
BringupResult MyCobotCSP::enable_drive(int s)
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

    /* (4) Enable Operation → OPERATION_ENABLED.
     *
     * CSP safety: refresh target_position to the LIVE position_actual right
     * before the drive transitions into OPERATION_ENABLED. The cached
     * last_position_counts_ from init_soem() may be stale by tens of ms;
     * using the freshest count keeps the initial position-error window
     * tight when the drive starts servoing. */
    for (size_t i = 0; i < n_joints_; ++i) {
        int sj = joint_to_slave_[i];
        if (sj == s) {
            int32_t cur = txpdo_[s]->position_actual;
            last_position_counts_[i] = cur;
            rxpdo_[s]->target_position = cur;
        }
    }

    if (!cia402_transition(s, cw::ENABLE_OP_CMD, sw::OPERATION_ENABLED, sw::STATE_MASK,
                           timing::CIA402_TRANSITION_RETRIES, &retries)) {
        status = txpdo_[s]->statusword;
        RCLCPP_ERROR(logger_,
            "Slave %d: failed Enable Operation after %d/%d retries. sw=0x%04X (%s)",
            s, retries, timing::CIA402_TRANSITION_RETRIES, status, cia402_state_str(status));
        return BringupResult::ENABLE_OPERATION_FAILED;
    }
    RCLCPP_DEBUG(logger_, "Slave %d: Enable Operation OK after %d retries.", s, retries);

    /* (5) Brake release — stream PDOs for ~100 ms with brake bit set.
     * target_position holds the current count so the drive doesn't move. */
    for (int i = 0; i < timing::BRAKE_RELEASE_CYCLES; ++i) {
        rxpdo_[s]->controlword     = cw::ENABLE_OP_CMD;
        rxpdo_[s]->target_position = txpdo_[s]->position_actual;
        rxpdo_[s]->physical_output = brake::RELEASED;
        fieldbus_roundtrip();
        osal_usleep(timing::POLL_CYCLE_US);
    }

    /* Cache the post-brake-release count for any joint whose slave matches s. */
    for (size_t i = 0; i < n_joints_; ++i) {
        if (joint_to_slave_[i] == s) {
            last_position_counts_[i] = txpdo_[s]->position_actual;
        }
    }

    /* Some Laifual firmware revisions accept the SDO write of 0x6060 = 8
     * during PRE_OP→SAFE_OP but never apply it: the modes_of_operation
     * actual value (0x6061) stays at the previous active mode (often 9
     * = CSV from a previous session). The drive only transitions modes
     * once the CiA 402 PDS is in OPERATION_ENABLED. So: re-write
     * 0x6060 = 8 here, then verify 0x6061 = 8.
     *
     * If 0x6061 still doesn't update, a stronger fallback (write 0 then
     * 8 to force a transition) is attempted. Failure to reach mode 8 is
     * treated as a soft warning, not an error — the user will see in
     * /joint_states whether motion actually works. */
    {
        int8_t target_mode = 8;
        ecx_SDOwrite(&context_, s, 0x6060, 0x00, FALSE,
                     sizeof(target_mode), &target_mode, EC_TIMEOUTSAFE);
        osal_usleep(timing::MODE_SETTLE_US);
        int8_t mode_display = -99;
        int sz = sizeof(mode_display);
        ecx_SDOread(&context_, s, 0x6061, 0x00, FALSE, &sz, &mode_display, EC_TIMEOUTSAFE);

        if (mode_display != 8) {
            RCLCPP_WARN(logger_,
                "Slave %d: mode_display=%d after first re-write of 0x6060=8. "
                "Trying 0→8 toggle to force transition.",
                s, (int)mode_display);
            int8_t zero = 0;
            ecx_SDOwrite(&context_, s, 0x6060, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);
            osal_usleep(timing::MODE_SETTLE_US);
            ecx_SDOwrite(&context_, s, 0x6060, 0x00, FALSE,
                         sizeof(target_mode), &target_mode, EC_TIMEOUTSAFE);
            osal_usleep(timing::MODE_SETTLE_US);
            sz = sizeof(mode_display);
            ecx_SDOread(&context_, s, 0x6061, 0x00, FALSE, &sz, &mode_display, EC_TIMEOUTSAFE);
        }

        uint32_t max_motor_speed = 0xDEADBEEF;
        sz = sizeof(max_motor_speed);
        ecx_SDOread(&context_, s, 0x6080, 0x00, FALSE, &sz, &max_motor_speed, EC_TIMEOUTSAFE);

        if (mode_display == 8) {
            RCLCPP_INFO(logger_,
                "Slave %d: CSP active (mode_display=8). max_motor_speed=%u counts/sec.",
                s, max_motor_speed);
        } else {
            RCLCPP_WARN(logger_,
                "Slave %d: mode_display=%d (expected 8). Drive may still be in CSV. "
                "max_motor_speed=%u",
                s, (int)mode_display, max_motor_speed);
        }
    }

    RCLCPP_INFO(logger_,
        "Slave %d: OPERATION_ENABLED, brake released. position_actual=%d counts.",
        s, txpdo_[s]->position_actual);
    return BringupResult::OK;
}

/* ============================================================================
 * disable_drive / close_soem
 * ========================================================================== */
void MyCobotCSP::disable_drive(int s)
{
    if (!txpdo_[s] || !rxpdo_[s]) return;

    RCLCPP_DEBUG(logger_, "Slave %d: disable_drive() entry (sw=0x%04X / %s)",
                 s, txpdo_[s]->statusword, cia402_state_str(txpdo_[s]->statusword));

    /* CSP safety: hold current position while disabling. Writing 0 here
     * would command the joint to encoder origin in the cycles before the
     * drive actually disables. */
    int32_t hold = txpdo_[s]->position_actual;
    rxpdo_[s]->target_position = hold;
    rxpdo_[s]->controlword     = cw::ENABLE_OP_CMD;
    rxpdo_[s]->physical_output = brake::RELEASED;
    fieldbus_roundtrip();
    osal_usleep(timing::DRIVE_DISABLE_US);

    rxpdo_[s]->target_position = txpdo_[s]->position_actual;
    rxpdo_[s]->physical_output = brake::ENGAGED;
    rxpdo_[s]->controlword     = cw::SHUTDOWN;
    fieldbus_roundtrip();
    fieldbus_roundtrip();
    osal_usleep(timing::DRIVE_DISABLE_US);

    RCLCPP_INFO(logger_, "Slave %d: disabled, brake engaged.", s);
}

void MyCobotCSP::close_soem()
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
hardware_interface::CallbackReturn MyCobotCSP::on_init(
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
    hw_position_commands_.assign(n_joints_, std::numeric_limits<double>::quiet_NaN());
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

        bool has_pos_cmd = false;
        for (const auto & ci : j.command_interfaces) {
            if (ci.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
        }
        if (!has_pos_cmd) {
            RCLCPP_ERROR(logger_,
                "Joint '%s' must declare a 'position' command interface (CSP mode).",
                j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(logger_, "Joint '%s' configured: slave=%d, counts_per_rad=%.1f",
                     j.name.c_str(), joint_to_slave_[i], counts_per_rad_[i]);
    }

    rxpdo_.assign(max_slave + 1, nullptr);
    txpdo_.assign(max_slave + 1, nullptr);

    RCLCPP_DEBUG(logger_,
        "MyCobotCSP initialised: ifname='%s', %zu joint(s), max slave %d.",
        ifname_.c_str(), n_joints_, max_slave);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyCobotCSP::export_state_interfaces()
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
MyCobotCSP::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    for (size_t i = 0; i < n_joints_; ++i) {
        ifaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION,
                            &hw_position_commands_[i]);
    }
    return ifaces;
}

hardware_interface::CallbackReturn MyCobotCSP::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Activating MyCobotCSP (%zu joint(s) on '%s')...",
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

    /* CSP safety: seed hw_position_commands_ from the live encoder reading
     * so the first scheduler-driven write() can never land on NaN (which
     * would otherwise force the fallback path). */
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        int32_t cnt = txpdo_[s]->position_actual;
        last_position_counts_[i]   = cnt;
        hw_positions_[i]           = static_cast<double>(cnt) / counts_per_rad_[i];
        hw_velocities_[i]          = 0.0;
        hw_position_commands_[i]   = hw_positions_[i];
    }

    RCLCPP_INFO(logger_, "MyCobotCSP ACTIVE — %zu drive(s) operating in CSP mode.",
                n_joints_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotCSP::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Deactivating MyCobotCSP...");
    for (size_t i = 0; i < n_joints_; ++i) {
        disable_drive(joint_to_slave_[i]);
    }
    close_soem();
    return hardware_interface::CallbackReturn::SUCCESS;
}

/* ============================================================================
 * ros2_control read / write — per-cycle methods
 * ========================================================================== */
hardware_interface::return_type MyCobotCSP::read(
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

hardware_interface::return_type MyCobotCSP::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];

        /* CSP safety: NaN → hold current position. CSV could safely
         * substitute 0 (= zero velocity = stand still); CSP cannot,
         * because 0 means "go to encoder origin". */
        double cmd_rad = hw_position_commands_[i];
        if (!std::isfinite(cmd_rad)) cmd_rad = hw_positions_[i];

        int32_t cnt = static_cast<int32_t>(
            std::lround(cmd_rad * counts_per_rad_[i]));

        rxpdo_[s]->target_position  = cnt;
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

    /* Diagnostic: print what we're commanding vs. actual every 1 s. */
    static rclcpp::Clock diag_clock(RCL_STEADY_TIME);
    if (n_joints_ > 0) {
        int s0 = joint_to_slave_[0];
        RCLCPP_INFO_THROTTLE(logger_, diag_clock, 1000,
            "j0 cmd_rad=%.6f tgt_cnt=%d  pos_cnt=%d  sw=0x%04X  ferr=%d  wkc=%d",
            hw_position_commands_[0],
            rxpdo_[s0]->target_position,
            txpdo_[s0]->position_actual,
            txpdo_[s0]->statusword,
            txpdo_[s0]->following_error,
            wkc);
    }
    return hardware_interface::return_type::OK;
}

}  // namespace mycobot_csp

PLUGINLIB_EXPORT_CLASS(mycobot_csp::MyCobotCSP,
                       hardware_interface::SystemInterface)
