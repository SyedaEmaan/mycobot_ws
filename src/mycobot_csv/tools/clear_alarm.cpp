/**
 * clear_alarm.cpp — Standalone diagnostic + recovery tool for Laifual/CMD
 * Appolo CiA 402 drives that latch a vendor-specific FAULT (e.g. 0x002E)
 * which the standard CiA 402 fault-reset edge cannot clear.
 *
 * Build:
 *   g++ -std=c++17 clear_alarm.cpp -o clear_alarm \
 *     -I/usr/local/include/soem -L/usr/local/lib -lsoem -lpthread
 *
 * Run:
 *   sudo ./clear_alarm <ifname> <slave_index> [--no-restore | --diag-only]
 *
 *   --no-restore : skip the destructive methods [B] and [C]; only run the
 *                  diagnostic reads + the surgical encoder-clear method [A].
 *                  Use on slaves you do NOT want to factory-reset.
 *   --diag-only  : skip ALL clearing methods ([A], [B], [C]); only run the
 *                  diagnostic reads. Use on healthy slaves you want to
 *                  observe without perturbing.
 *
 * Steps:
 *   1) ecx_init + ecx_config_init → bus enumerated, slaves in PRE_OP
 *   2) Read 0x603F (Error Code) — confirms entry-state fault code
 *   3) Read ALARM0..ALARM9 (0x3200-0x3209) — vendor alarm history
 *   4) Three clearing attempts (each prints OK / abort code on failure):
 *        [A] 0x3685:01 = 1   "Clear all encoder errors"   (hidden firmware SDO)
 *            0x3685:02 = 1   "Clear multi loop errors"    (hidden firmware SDO)
 *        [B] 0x2000   = 1   "Load default Para"           (vendor factory restore)
 *        [C] 0x1011:01 = 'load'  (0x64616F6C)             (CiA 301 std restore)
 *   5) Re-read 0x603F + ALARM0 to show post-clear state
 *
 * After this tool runs, POWER-CYCLE the drive.
 *   - Method [B] wipes vendor parameters (likely encoder zero, PID gains) —
 *     counts_per_rad recalibration will be required.
 *   - Power-cycle ensures the drive re-reads parameters cleanly.
 *
 * SDO failures are non-fatal — the tool prints the abort code and continues
 * so a later method may still succeed.
 */

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

extern "C" {
#include "soem/soem.h"
}

static ecx_contextt g_ctx{};

static void drain_errors(const char * tag)
{
    while (ecx_iserror(&g_ctx)) {
        printf("    [%s] %s\n", tag, ecx_elist2string(&g_ctx));
    }
}

static int sdo_read_u16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t * out)
{
    int sz = (int)sizeof(*out);
    *out = 0;
    return ecx_SDOread(&g_ctx, slave, idx, sub, FALSE, &sz, out, EC_TIMEOUTRXM);
}

static int sdo_write_u16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t val)
{
    return ecx_SDOwrite(&g_ctx, slave, idx, sub, FALSE, (int)sizeof(val), &val, EC_TIMEOUTRXM);
}

static int sdo_write_u32(uint16_t slave, uint16_t idx, uint8_t sub, uint32_t val)
{
    return ecx_SDOwrite(&g_ctx, slave, idx, sub, FALSE, (int)sizeof(val), &val, EC_TIMEOUTRXM);
}

static void show_603F(uint16_t slave, const char * tag)
{
    uint16_t err = 0;
    int rc = sdo_read_u16(slave, 0x603F, 0x00, &err);
    if (rc > 0) {
        printf("  [%s] 0x603F Error Code = 0x%04X (%u)\n", tag, err, err);
    } else {
        printf("  [%s] 0x603F read FAILED\n", tag);
        drain_errors(tag);
    }
}

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <ifname> <slave_index> [--no-restore | --diag-only]\n", argv[0]);
        fprintf(stderr, "  e.g. sudo %s enxc8a3623069bf 2\n", argv[0]);
        fprintf(stderr, "  --no-restore : skip destructive methods [B] and [C]\n");
        fprintf(stderr, "  --diag-only  : skip ALL clearing methods ([A], [B], [C])\n");
        return 1;
    }

    const char * ifname = argv[1];
    int slave_arg = std::atoi(argv[2]);
    if (slave_arg < 1 || slave_arg > 65535) {
        fprintf(stderr, "Invalid slave index '%s'\n", argv[2]);
        return 1;
    }
    uint16_t slave = (uint16_t)slave_arg;

    bool no_restore = false;
    bool diag_only  = false;
    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-restore") == 0) no_restore = true;
        else if (std::strcmp(argv[i], "--diag-only") == 0) diag_only = true;
    }
    if (diag_only) no_restore = true;  // diag-only implies no-restore

    const char * mode_tag =
        diag_only  ? " (--diag-only: skipping [A], [B], [C])" :
        no_restore ? " (--no-restore: skipping [B] and [C])"  : "";
    printf("=== clear_alarm: ifname=%s slave=%u%s ===\n\n", ifname, slave, mode_tag);

    if (ecx_init(&g_ctx, ifname) <= 0) {
        fprintf(stderr, "ecx_init('%s') failed (need cap_net_raw or sudo, NIC unmanaged)\n", ifname);
        return 1;
    }

    if (ecx_config_init(&g_ctx) <= 0) {
        fprintf(stderr, "ecx_config_init: NO_SLAVES_FOUND on bus\n");
        ecx_close(&g_ctx);
        return 1;
    }

    printf("Bus: %d slave(s) discovered.\n", g_ctx.slavecount);

    if (slave > (uint16_t)g_ctx.slavecount) {
        fprintf(stderr, "Slave index %u out of range [1..%d]\n", slave, g_ctx.slavecount);
        ecx_close(&g_ctx);
        return 1;
    }

    /* Print observed state of each slave after config_init. */
    ecx_readstate(&g_ctx);
    for (int i = 1; i <= g_ctx.slavecount; ++i) {
        printf("  slave %d: state=0x%02X (al_status=0x%04X)\n",
               i, g_ctx.slavelist[i].state, g_ctx.slavelist[i].ALstatuscode);
    }

    /* Force the chosen slave to PRE_OP and wait. SDO/CoE only works in PRE_OP+. */
    g_ctx.slavelist[slave].state = EC_STATE_PRE_OP;
    ecx_writestate(&g_ctx, slave);
    uint16_t reached = ecx_statecheck(&g_ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    printf("  slave %u: requested PRE_OP, reached state=0x%02X (al_status=0x%04X)\n\n",
           slave, reached, g_ctx.slavelist[slave].ALstatuscode);
    if ((reached & 0x0F) < EC_STATE_PRE_OP) {
        fprintf(stderr, "Slave %u did not reach PRE_OP — SDOs will fail. Aborting.\n", slave);
        ecx_close(&g_ctx);
        return 1;
    }

    /* ---------- Step 2: Standard CiA 402 error code ---------- */
    printf("--- Step 2: 0x603F (CiA 402 Error Code) ---\n");
    show_603F(slave, "pre");
    printf("\n");

    /* ---------- Step 3: Vendor ALARM history ---------- */
    printf("--- Step 3: ALARM history (0x3200-0x3209) ---\n");
    for (uint16_t idx = 0x3200; idx <= 0x3209; ++idx) {
        uint16_t v = 0;
        int rc = sdo_read_u16(slave, idx, 0x00, &v);
        if (rc > 0) {
            printf("  ALARM%X (0x%04X) = 0x%04X (%u)\n",
                   idx - 0x3200, idx, v, v);
        } else {
            printf("  ALARM%X (0x%04X) = read FAILED\n", idx - 0x3200, idx);
            drain_errors("ALARM");
        }
    }
    printf("\n");

    /* ---------- Step 4: Clearing attempts ---------- */
    printf("--- Step 4: clearing attempts ---\n");

    if (diag_only) {
        printf("[A]  SKIPPED (--diag-only)\n");
    } else {
        /* [A] 0x3685:01 + :02 — vendor surgical clear (encoder errors) */
        printf("[A1] write 0x3685:01 = 1  (Clear all encoder errors)... ");
        {
            int rc = sdo_write_u16(slave, 0x3685, 0x01, 0x0001);
            printf("%s\n", rc > 0 ? "OK" : "FAILED");
            if (rc <= 0) drain_errors("A1");
        }
        usleep(50000);

        printf("[A2] write 0x3685:02 = 1  (Clear multi loop errors)... ");
        {
            int rc = sdo_write_u16(slave, 0x3685, 0x02, 0x0001);
            printf("%s\n", rc > 0 ? "OK" : "FAILED");
            if (rc <= 0) drain_errors("A2");
        }
        usleep(50000);
        show_603F(slave, "post-A");
        printf("\n");
    }

    if (no_restore) {
        printf("[B]  SKIPPED (--no-restore)\n");
        printf("[C]  SKIPPED (--no-restore)\n\n");
    } else {
        /* [B] 0x2000 — vendor "Load default Para" (factory restore for vendor params) */
        printf("[B]  write 0x2000   = 1  (Load default Para — vendor factory restore)... ");
        {
            int rc = sdo_write_u16(slave, 0x2000, 0x00, 0x0001);
            printf("%s\n", rc > 0 ? "OK" : "FAILED");
            if (rc <= 0) drain_errors("B");
        }
        usleep(200000);
        show_603F(slave, "post-B");
        printf("\n");

        /* [C] 0x1011:01 = 'load' — CiA 301 standard restore (not declared in ESI but try anyway) */
        printf("[C]  write 0x1011:01 = 0x64616F6C ('load' — CiA 301 std restore)... ");
        {
            int rc = sdo_write_u32(slave, 0x1011, 0x01, 0x64616F6Cu);
            printf("%s\n", rc > 0 ? "OK" : "FAILED");
            if (rc <= 0) drain_errors("C");
        }
        usleep(200000);
        show_603F(slave, "post-C");
        printf("\n");
    }

    /* ---------- Step 5: post state ---------- */
    printf("--- Step 5: post-clear state ---\n");
    show_603F(slave, "final");
    {
        uint16_t v = 0;
        int rc = sdo_read_u16(slave, 0x3200, 0x00, &v);
        if (rc > 0) printf("  ALARM0 (0x3200) = 0x%04X (%u)\n", v, v);
    }

    printf("\nDone. POWER-CYCLE the drive now, then re-run verify_four.\n");
    printf("If 0x603F is now 0x0000, the fault was cleared by one of the methods above.\n");

    ecx_close(&g_ctx);
    return 0;
}
