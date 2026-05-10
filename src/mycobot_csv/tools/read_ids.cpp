/**
 * read_ids.cpp — Read CoE Identity Object (0x1018) for every slave on the bus.
 *
 * Use this to fingerprint each drive — identical motors share the same
 * vendor/product/revision; serial numbers differ. Useful before/after
 * rewiring the EtherCAT daisy-chain to confirm which physical motor
 * landed at which slave index.
 *
 * Build:
 *   g++ -std=c++17 read_ids.cpp -o read_ids \
 *     -I/usr/local/include/soem -L/usr/local/lib -lsoem -lpthread
 *
 * Run (needs cap_net_raw on the binary, or sudo, plus NIC unmanaged):
 *   sudo setcap cap_net_admin,cap_net_raw=eip ./read_ids
 *   ./read_ids <ifname>
 */

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "soem/soem.h"
}

static ecx_contextt g_ctx{};

static int sdo_read_u32(uint16_t slave, uint16_t idx, uint8_t sub, uint32_t * out)
{
    int sz = (int)sizeof(*out);
    *out = 0;
    return ecx_SDOread(&g_ctx, slave, idx, sub, FALSE, &sz, out, EC_TIMEOUTRXM);
}

int main(int argc, char ** argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <ifname>\n", argv[0]);
        return 1;
    }
    const char * ifname = argv[1];

    if (ecx_init(&g_ctx, ifname) <= 0) {
        fprintf(stderr, "ecx_init('%s') failed (need cap_net_raw or sudo, NIC unmanaged)\n", ifname);
        return 1;
    }

    if (ecx_config_init(&g_ctx) <= 0) {
        fprintf(stderr, "ecx_config_init: NO_SLAVES_FOUND on bus\n");
        ecx_close(&g_ctx);
        return 1;
    }

    printf("Bus: %d slave(s) discovered.\n\n", g_ctx.slavecount);

    /* Force every slave to PRE_OP so SDO reads are accepted. */
    for (int s = 1; s <= g_ctx.slavecount; ++s) {
        g_ctx.slavelist[s].state = EC_STATE_PRE_OP;
        ecx_writestate(&g_ctx, s);
    }
    for (int s = 1; s <= g_ctx.slavecount; ++s) {
        ecx_statecheck(&g_ctx, s, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    }

    printf("=== 0x1018 Identity Object ===\n");
    printf("%-6s %-12s %-12s %-12s %-12s %s\n",
           "slave", "vendor", "product", "revision", "serial", "EEPROM name");
    printf("---------------------------------------------------------------------------\n");

    for (int s = 1; s <= g_ctx.slavecount; ++s) {
        uint32_t vendor = 0, product = 0, revision = 0, serial = 0;
        sdo_read_u32((uint16_t)s, 0x1018, 0x01, &vendor);
        sdo_read_u32((uint16_t)s, 0x1018, 0x02, &product);
        sdo_read_u32((uint16_t)s, 0x1018, 0x03, &revision);
        sdo_read_u32((uint16_t)s, 0x1018, 0x04, &serial);

        printf("%-6d 0x%08X   0x%08X   0x%08X   0x%08X   %s\n",
               s, vendor, product, revision, serial,
               g_ctx.slavelist[s].name);
    }

    /* CiA 402 motor parameters — these *should* differ between motor sizes. */
    printf("\n=== CiA 402 motor params ===\n");
    printf("%-6s %-22s %-22s %-12s %-12s %-12s\n",
           "slave",
           "0x608F enc-incr/rev",
           "0x6091 gear num/den",
           "0x6075 mA",
           "0x6076 mNm",
           "0x6080 maxRPM");
    printf("---------------------------------------------------------------------------------------------\n");

    for (int s = 1; s <= g_ctx.slavecount; ++s) {
        uint32_t enc_incr = 0, enc_rev = 0;
        uint32_t gear_num = 0, gear_den = 0;
        uint32_t rated_current = 0, rated_torque = 0, max_rpm = 0;

        sdo_read_u32((uint16_t)s, 0x608F, 0x01, &enc_incr);
        sdo_read_u32((uint16_t)s, 0x608F, 0x02, &enc_rev);
        sdo_read_u32((uint16_t)s, 0x6091, 0x01, &gear_num);
        sdo_read_u32((uint16_t)s, 0x6091, 0x02, &gear_den);
        sdo_read_u32((uint16_t)s, 0x6075, 0x00, &rated_current);
        sdo_read_u32((uint16_t)s, 0x6076, 0x00, &rated_torque);
        sdo_read_u32((uint16_t)s, 0x6080, 0x00, &max_rpm);

        char enc_buf[32], gear_buf[32];
        snprintf(enc_buf,  sizeof(enc_buf),  "%u/%u", enc_incr, enc_rev);
        snprintf(gear_buf, sizeof(gear_buf), "%u/%u", gear_num, gear_den);

        printf("%-6d %-22s %-22s %-12u %-12u %-12u\n",
               s, enc_buf, gear_buf, rated_current, rated_torque, max_rpm);
    }

    ecx_close(&g_ctx);
    return 0;
}
