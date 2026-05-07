/**
 * mycobot_pp.cpp — CiA 402 Profile Position plugin (mode 1).
 *
 * Sibling of mycobot_csp. Same EtherCAT bring-up + CiA 402 state machine.
 * Created because the Laifual L70I-E-100-BF firmware on this hardware
 * does NOT implement CSP (mode 8) despite the ESI XML listing it. PP
 * (mode 1) is universally supported and uses the same 0x607A target
 * position object — only the controlword handshake differs.
 *
 * == Profile Position handshake ==
 *
 *   Per CiA 402, PP commits each new target_position on a rising edge of
 *   controlword bit 4 ("New set-point"). For streaming use (MoveIt-style
 *   trajectory feed-through), we hold bit 5 ("Change set immediately") = 1
 *   so a new rising edge interrupts any in-progress motion. Bit 6
 *   (relative/absolute) = 0 → all targets are absolute.
 *
 *   write() toggles bit 4 every cycle. At 200 Hz this produces rising
 *   edges every 10 ms; combined with bit 5, the drive servos to the
 *   freshest target_position with ≤10 ms latency.
 *
 * == Profile parameters ==
 *
 *   slave_pp_config() writes generous defaults:
 *     0x6081 profile velocity     = 5 M counts/sec  (~2.4 rad/s)
 *     0x6083 profile acceleration = 50 M counts/sec²
 *     0x6084 profile deceleration = 50 M counts/sec²
 *     0x607F max profile velocity = 5 M counts/sec  (clamp)
 *
 *   For tiny per-cycle deltas streamed by a controller, the drive
 *   essentially follows the master target. For large step commands,
 *   motion is bounded by these profile limits.
 */

#include "mycobot_pp/mycobot_pp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace mycobot_pp {

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

const char * MyCobotPP::cia402_state_str(uint16_t status)
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
 * configure_* helpers
 * ========================================================================== */

int MyCobotPP::configure_pp_mode(ecx_contextt * context, uint16_t slave)
{
    /* Set 0x6060 = 1 (Profile Position). */
    int8_t mode = 1;
    int wc = ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE,
                          sizeof(mode), &mode, EC_TIMEOUTSAFE);
    if (wc <= 0) return -1;
    osal_usleep(timing::MODE_SETTLE_US);

    int8_t mode_rb = 0;
    int sz = sizeof(mode_rb);
    ecx_SDOread(context, slave, 0x6060, 0x00, FALSE, &sz, &mode_rb, EC_TIMEOUTSAFE);
    if (mode_rb != 1) return -1;

    /* Diagnostic readback (cannot RCLCPP from static SDO callback). */
    int8_t mode_display = -99;
    sz = sizeof(mode_display);
    ecx_SDOread(context, slave, 0x6061, 0x00, FALSE, &sz, &mode_display, EC_TIMEOUTSAFE);

    uint16_t error_code = 0xDEAD;
    sz = sizeof(error_code);
    ecx_SDOread(context, slave, 0x603F, 0x00, FALSE, &sz, &error_code, EC_TIMEOUTSAFE);

    fprintf(stderr,
        "[mycobot_pp slave=%u PP cfg] commanded_mode(0x6060)=%d  "
        "mode_display(0x6061)=%d  error_code(0x603F)=0x%04X\n",
        (unsigned)slave, (int)mode_rb, (int)mode_display, error_code);
    return 1;
}

int MyCobotPP::configure_interpolation_period(ecx_contextt * context, uint16_t slave)
{
    int8_t interp_val = 5;
    int8_t interp_idx = -3;
    if (ecx_SDOwrite(context, slave, 0x60C2, 0x01, FALSE,
                     sizeof(interp_val), &interp_val, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x60C2, 0x02, FALSE,
                     sizeof(interp_idx), &interp_idx, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotPP::configure_txpdo(ecx_contextt * context, uint16_t slave)
{
    /* TxPDO 0x1A02 — same 8 entries as CSV/CSP. Position feedback is
     * identical regardless of mode. */
    uint32_t tx_objs[] = {
        0x60410010, 0x60640020, 0x60770010, 0x60F40020,
        0x60B90010, 0x60BA0020, 0x60BC0020, 0x60FD0020
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

int MyCobotPP::configure_rxpdo(ecx_contextt * context, uint16_t slave)
{
    /* RxPDO 0x1602 — entry 2 = 0x607A target_position. Same as CSP. */
    uint32_t rx_objs[] = {
        0x60400010,
        0x607A0020,
        0x60B80010,
        0x60FE0120
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

int MyCobotPP::configure_brake_output(ecx_contextt * context, uint16_t slave)
{
    uint32_t output_mask = brake::RELEASED;
    if (ecx_SDOwrite(context, slave, 0x60FE, 0x02, FALSE,
                     sizeof(output_mask), &output_mask, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotPP::configure_sync_managers(ecx_contextt * context, uint16_t slave)
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

int MyCobotPP::configure_free_run_sync(ecx_contextt * context, uint16_t slave)
{
    uint16_t sync_free = 0;
    if (ecx_SDOwrite(context, slave, 0x1C32, 0x01, FALSE,
                     sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x1C33, 0x01, FALSE,
                     sizeof(sync_free), &sync_free, EC_TIMEOUTSAFE) <= 0) return -1;
    return 1;
}

int MyCobotPP::configure_pp_motion_limits(ecx_contextt * context, uint16_t slave)
{
    /* Generous PP motion limits. Counts/sec for velocity, counts/sec² for
     * accel/decel. With counts_per_rad ≈ 2,085,932:
     *   profile_velocity = 5,000,000 cps  ≈  2.40 rad/s
     *   profile_accel    = 50,000,000     ≈  24   rad/s²
     */
    uint32_t profile_velocity = 5000000u;
    uint32_t profile_accel    = 50000000u;
    uint32_t profile_decel    = 50000000u;
    uint32_t max_profile_velocity = 5000000u;
    uint32_t max_motor_speed = 10000000u;

    if (ecx_SDOwrite(context, slave, 0x6081, 0x00, FALSE,
                     sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x6083, 0x00, FALSE,
                     sizeof(profile_accel), &profile_accel, EC_TIMEOUTSAFE) <= 0) return -1;
    if (ecx_SDOwrite(context, slave, 0x6084, 0x00, FALSE,
                     sizeof(profile_decel), &profile_decel, EC_TIMEOUTSAFE) <= 0) return -1;
    /* 0x607F (max profile velocity) and 0x6080 (max motor speed) are
     * non-fatal if absent on this drive — ignore failure. */
    ecx_SDOwrite(context, slave, 0x607F, 0x00, FALSE,
                 sizeof(max_profile_velocity), &max_profile_velocity, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x6080, 0x00, FALSE,
                 sizeof(max_motor_speed), &max_motor_speed, EC_TIMEOUTSAFE);

    /* Disable following-error trip — generous window + max timeout. */
    uint32_t big_window = 0xFFFFFFFFu;
    uint16_t big_timeout = 0xFFFFu;
    ecx_SDOwrite(context, slave, 0x6065, 0x00, FALSE,
                 sizeof(big_window), &big_window, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x6066, 0x00, FALSE,
                 sizeof(big_timeout), &big_timeout, EC_TIMEOUTSAFE);
    return 1;
}

int MyCobotPP::slave_pp_config(ecx_contextt * context, uint16_t slave)
{
    if (configure_pp_mode(context, slave)               < 0) return -1;
    if (configure_interpolation_period(context, slave)  < 0) return -1;
    if (configure_txpdo(context, slave)                 < 0) return -1;
    if (configure_rxpdo(context, slave)                 < 0) return -1;
    if (configure_brake_output(context, slave)          < 0) return -1;
    if (configure_sync_managers(context, slave)         < 0) return -1;
    if (configure_free_run_sync(context, slave)         < 0) return -1;
    if (configure_pp_motion_limits(context, slave)      < 0) return -1;
    return 1;
}

int MyCobotPP::fieldbus_roundtrip()
{
    ecx_send_processdata(&context_);
    return ecx_receive_processdata(&context_, timing::RECEIVE_TIMEOUT_US);
}

BringupResult MyCobotPP::init_soem()
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
        context_.slavelist[s].PO2SOconfig = &MyCobotPP::slave_pp_config;
    }

    ecx_config_map_group(&context_, io_map_, group_);

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        uint16_t wd_off = 0;
        ecx_FPWR(&context_.port, context_.slavelist[s].configadr,
                 0x0422, sizeof(wd_off), &wd_off, EC_TIMEOUTRET);
    }

    ec_groupt * grp = context_.grouplist + group_;
    if (grp->Obytes != sizeof(pp_rxpdo_t)) {
        RCLCPP_ERROR(logger_,
            "Output PDO size mismatch: mapped %d bytes, expected %zu (pp_rxpdo_t).",
            grp->Obytes, sizeof(pp_rxpdo_t));
        ecx_close(&context_);
        return BringupResult::PDO_SIZE_MISMATCH;
    }
    if (grp->Ibytes != sizeof(pp_txpdo_t)) {
        RCLCPP_ERROR(logger_,
            "Input PDO size mismatch: mapped %d bytes, expected %zu (pp_txpdo_t).",
            grp->Ibytes, sizeof(pp_txpdo_t));
        ecx_close(&context_);
        return BringupResult::PDO_SIZE_MISMATCH;
    }

    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        rxpdo_[s] = reinterpret_cast<pp_rxpdo_t *>(context_.slavelist[s].outputs);
        txpdo_[s] = reinterpret_cast<pp_txpdo_t *>(context_.slavelist[s].inputs);
        if (!rxpdo_[s] || !txpdo_[s]) {
            RCLCPP_ERROR(logger_, "Slave %d: null PDO buffer after mapping.", s);
            ecx_close(&context_);
            return BringupResult::NULL_PDO_BUFFER;
        }
    }

    /* Seed last_position_counts_ via early roundtrip so priming/brake-release
     * sites can write target_position = current count, not 0. */
    fieldbus_roundtrip();
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        last_position_counts_[i] = txpdo_[s]->position_actual;
        last_target_counts_[i]   = txpdo_[s]->position_actual;
    }

    return BringupResult::OK;
}

BringupResult MyCobotPP::reach_safe_op()
{
    context_.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&context_, 0);
    for (int retries = 0; retries < timing::SAFE_OP_RETRIES; ++retries) {
        ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        ecx_readstate(&context_);
        if (context_.slavelist[0].state >= EC_STATE_SAFE_OP) {
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

BringupResult MyCobotPP::reach_operational()
{
    /* Prime PDOs: target = current count, controlword = SHUTDOWN, brake engaged. */
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        rxpdo_[s]->controlword      = cw::SHUTDOWN;
        rxpdo_[s]->target_position  = last_position_counts_[i];
        rxpdo_[s]->touch_probe_func = 0;
        rxpdo_[s]->physical_output  = brake::ENGAGED;
    }
    for (int i = 0; i < timing::PRIMING_INITIAL_CYCLES; ++i) fieldbus_roundtrip();
    for (int i = 0; i < timing::PRIMING_FINAL_CYCLES;   ++i) fieldbus_roundtrip();

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
        if (all_op) return BringupResult::OK;
    }
    return BringupResult::OPERATIONAL_TIMEOUT;
}

bool MyCobotPP::cia402_transition(int slave_idx,
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

BringupResult MyCobotPP::enable_drive(int s)
{
    fieldbus_roundtrip();
    uint16_t status = txpdo_[s]->statusword;
    RCLCPP_DEBUG(logger_, "Slave %d enable entry: sw=0x%04X (%s)",
                 s, status, cia402_state_str(status));

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
                "Slave %d: fault did not clear after %d retries (sw=0x%04X).",
                s, timing::FAULT_RESET_RETRIES, status);
            return BringupResult::FAULT_NOT_CLEARED;
        }
    }

    int retries = 0;
    if (!cia402_transition(s, cw::SHUTDOWN, sw::READY_TO_SWITCH_ON, sw::STATE_MASK,
                           timing::CIA402_TRANSITION_RETRIES, &retries)) {
        RCLCPP_ERROR(logger_, "Slave %d: failed Shutdown after %d retries.", s, retries);
        return BringupResult::SHUTDOWN_FAILED;
    }
    if (!cia402_transition(s, cw::SWITCH_ON_CMD, sw::SWITCHED_ON, sw::STATE_MASK,
                           timing::CIA402_TRANSITION_RETRIES, &retries)) {
        RCLCPP_ERROR(logger_, "Slave %d: failed Switch On after %d retries.", s, retries);
        return BringupResult::SWITCH_ON_FAILED;
    }

    /* Refresh target_position to live position right before ENABLE_OPERATION. */
    for (size_t i = 0; i < n_joints_; ++i) {
        if (joint_to_slave_[i] == s) {
            int32_t cur = txpdo_[s]->position_actual;
            last_position_counts_[i] = cur;
            last_target_counts_[i]   = cur;
            rxpdo_[s]->target_position = cur;
        }
    }

    if (!cia402_transition(s, cw::ENABLE_OP_CMD, sw::OPERATION_ENABLED, sw::STATE_MASK,
                           timing::CIA402_TRANSITION_RETRIES, &retries)) {
        RCLCPP_ERROR(logger_, "Slave %d: failed Enable Operation after %d retries.", s, retries);
        return BringupResult::ENABLE_OPERATION_FAILED;
    }

    /* Brake release: 20 cycles holding current position. */
    for (int i = 0; i < timing::BRAKE_RELEASE_CYCLES; ++i) {
        rxpdo_[s]->controlword     = cw::ENABLE_OP_CMD;
        rxpdo_[s]->target_position = txpdo_[s]->position_actual;
        rxpdo_[s]->physical_output = brake::RELEASED;
        fieldbus_roundtrip();
        osal_usleep(timing::POLL_CYCLE_US);
    }
    for (size_t i = 0; i < n_joints_; ++i) {
        if (joint_to_slave_[i] == s) {
            last_position_counts_[i] = txpdo_[s]->position_actual;
            last_target_counts_[i]   = txpdo_[s]->position_actual;
            setpoint_acked_[i] = true;
        }
    }

    /* Re-write 0x6060 = 1 post-enable in case the drive only applies the
     * mode after entering OPERATION_ENABLED. Then read back 0x6061 to
     * confirm we're actually in PP. */
    {
        int8_t target_mode = 1;
        ecx_SDOwrite(&context_, s, 0x6060, 0x00, FALSE,
                     sizeof(target_mode), &target_mode, EC_TIMEOUTSAFE);
        osal_usleep(timing::MODE_SETTLE_US);
        int8_t mode_display = -99;
        int sz = sizeof(mode_display);
        ecx_SDOread(&context_, s, 0x6061, 0x00, FALSE, &sz, &mode_display, EC_TIMEOUTSAFE);

        uint32_t pv = 0; sz = sizeof(pv);
        ecx_SDOread(&context_, s, 0x6081, 0x00, FALSE, &sz, &pv, EC_TIMEOUTSAFE);
        uint32_t pa = 0; sz = sizeof(pa);
        ecx_SDOread(&context_, s, 0x6083, 0x00, FALSE, &sz, &pa, EC_TIMEOUTSAFE);

        if (mode_display == 1) {
            RCLCPP_INFO(logger_,
                "Slave %d: PP active. profile_velocity=%u counts/sec, profile_accel=%u",
                s, pv, pa);
        } else {
            RCLCPP_WARN(logger_,
                "Slave %d: mode_display=%d (expected 1=PP). Drive may not be in PP. "
                "profile_velocity=%u",
                s, (int)mode_display, pv);
        }
    }

    RCLCPP_INFO(logger_,
        "Slave %d: OPERATION_ENABLED, brake released. position_actual=%d counts.",
        s, txpdo_[s]->position_actual);
    return BringupResult::OK;
}

void MyCobotPP::disable_drive(int s)
{
    if (!txpdo_[s] || !rxpdo_[s]) return;

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

void MyCobotPP::close_soem()
{
    if (!soem_running_) return;
    context_.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&context_, 0);
    ecx_close(&context_);
    soem_running_ = false;
    RCLCPP_INFO(logger_, "EtherCAT socket closed.");
}

/* ============================================================================
 * ros2_control SystemInterface lifecycle
 * ========================================================================== */
hardware_interface::CallbackReturn MyCobotPP::on_init(
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
    last_target_counts_.assign(n_joints_, 0);
    setpoint_acked_.assign(n_joints_, true);

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
                "Joint '%s' must declare a 'position' command interface (PP mode).",
                j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    rxpdo_.assign(max_slave + 1, nullptr);
    txpdo_.assign(max_slave + 1, nullptr);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyCobotPP::export_state_interfaces()
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
MyCobotPP::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    for (size_t i = 0; i < n_joints_; ++i) {
        ifaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION,
                            &hw_position_commands_[i]);
    }
    return ifaces;
}

hardware_interface::CallbackReturn MyCobotPP::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Activating MyCobotPP (%zu joint(s) on '%s')...",
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

    /* Seed cmd buffer with current encoder so first write() doesn't NaN. */
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];
        int32_t cnt = txpdo_[s]->position_actual;
        last_position_counts_[i] = cnt;
        last_target_counts_[i]   = cnt;
        hw_positions_[i]         = static_cast<double>(cnt) / counts_per_rad_[i];
        hw_velocities_[i]        = 0.0;
        hw_position_commands_[i] = hw_positions_[i];
    }

    RCLCPP_INFO(logger_, "MyCobotPP ACTIVE — %zu drive(s) operating in PP mode.",
                n_joints_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotPP::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Deactivating MyCobotPP...");
    for (size_t i = 0; i < n_joints_; ++i) {
        disable_drive(joint_to_slave_[i]);
    }
    close_soem();
    return hardware_interface::CallbackReturn::SUCCESS;
}

/* ============================================================================
 * read / write
 * ========================================================================== */
hardware_interface::return_type MyCobotPP::read(
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
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobotPP::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (size_t i = 0; i < n_joints_; ++i) {
        int s = joint_to_slave_[i];

        double cmd_rad = hw_position_commands_[i];
        if (!std::isfinite(cmd_rad)) cmd_rad = hw_positions_[i];
        int32_t cnt = static_cast<int32_t>(
            std::lround(cmd_rad * counts_per_rad_[i]));

        rxpdo_[s]->target_position = cnt;
        rxpdo_[s]->physical_output = brake::RELEASED;
        rxpdo_[s]->touch_probe_func = 0;

        /* PP setpoint handshake.
         *
         * State variable: setpoint_acked_[i].
         *   true  → drive has previously acked, ready to commit a new target
         *   false → bit 4 has been set; waiting for drive to ack via SW bit 12
         *
         * Logic each cycle:
         *   - If target changed since last commit AND we're ready, set bit 4
         *     and mark NOT acked.
         *   - If we're waiting for ack and SW bit 12 is set, clear bit 4 and
         *     mark acked (ready for next change).
         *
         * Bit 5 (CHANGE_IMMEDIATE) is held = 1 so each new rising edge
         * interrupts any in-progress motion. Bit 6 (ABS_REL) = 0 (absolute).
         */
        uint16_t sw_bits = txpdo_[s]->statusword;
        uint16_t cw_out = cw::PP_BASE;  /* OP_ENABLED + CHANGE_IMMEDIATE, bit 4 = 0 */

        if (setpoint_acked_[i]) {
            if (cnt != last_target_counts_[i]) {
                /* New target — pulse bit 4 high to commit it. */
                cw_out = cw::PP_NEW_SETPOINT;
                last_target_counts_[i] = cnt;
                setpoint_acked_[i] = false;
            }
        } else {
            /* Waiting for ack. Hold bit 4 high until we see SW bit 12. */
            cw_out = cw::PP_NEW_SETPOINT;
            if (sw_bits & sw::SETPOINT_ACK) {
                /* Drive acked — drop bit 4 to allow the next setpoint cycle. */
                cw_out = cw::PP_BASE;
                setpoint_acked_[i] = true;
            }
        }

        rxpdo_[s]->controlword = cw_out;
    }

    int wkc = fieldbus_roundtrip();
    if (wkc <= 0) {
        static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(logger_, throttle_clock, 1000,
            "PDO roundtrip wkc=%d (link issue or slave dropped from OP).", wkc);
    }

    /* Diagnostic: log cmd / target / actual / sw once per second. */
    static rclcpp::Clock diag_clock(RCL_STEADY_TIME);
    if (n_joints_ > 0) {
        int s0 = joint_to_slave_[0];
        RCLCPP_INFO_THROTTLE(logger_, diag_clock, 1000,
            "j0 cmd_rad=%.6f tgt_cnt=%d  pos_cnt=%d  sw=0x%04X  cw=0x%04X  acked=%d  wkc=%d",
            hw_position_commands_[0],
            rxpdo_[s0]->target_position,
            txpdo_[s0]->position_actual,
            txpdo_[s0]->statusword,
            rxpdo_[s0]->controlword,
            (int)setpoint_acked_[0],
            wkc);
    }
    return hardware_interface::return_type::OK;
}

}  // namespace mycobot_pp

PLUGINLIB_EXPORT_CLASS(mycobot_pp::MyCobotPP,
                       hardware_interface::SystemInterface)
