#include "hpsdr_protocol.h"
#include <string.h>

int hpsdr_classify(const uint8_t *buf, int len)
{
    if (len < 4 || buf[0] != 0xEF || buf[1] != 0xFE)
        return PKT_UNKNOWN;

    switch (buf[2]) {
    case 0x02:  return PKT_DISCOVERY;
    case 0x04:
        return (buf[3] & 0x01) ? PKT_START : PKT_STOP;
    case 0x01:
        if (len >= HPSDR_PKT_SIZE) return PKT_EP2;
        return PKT_UNKNOWN;
    default:
        return PKT_UNKNOWN;
    }
}

void hpsdr_build_discovery_reply(uint8_t *reply, int in_use)
{
    memset(reply, 0, HPSDR_DISCOVERY_REPLY);
    reply[0]  = 0xEF;
    reply[1]  = 0xFE;
    reply[2]  = 0x02;
    reply[3]  = in_use ? 0x02 : 0x00;
    reply[4]  = 0x00;  // MAC
    reply[5]  = 0x1C;
    reply[6]  = 0xC0;
    reply[7]  = 0xA2;
    reply[8]  = 0x22;
    reply[9]  = 0x5B;
    reply[10] = 0x06;  // board type (Hermes)
    reply[11] = 0x25;  // protocol version
}

int hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result)
{
    if (!buf || len < HPSDR_PKT_SIZE)
        return 0;

    result->mox       = 0;
    result->freq      = 0;
    result->n_samples = 0;

    const uint8_t *ptr = buf + 8;  // skip Metis header

    for (int frame = 0; frame < 2; frame++) {
        if (ptr[0] != 0x7F || ptr[1] != 0x7F || ptr[2] != 0x7F) {
            ptr += 512;
            continue;
        }

        uint8_t c0 = ptr[3];
        int addr = (c0 >> 1) & 0x1F;   // your correct parsing
        int mox  = c0 & 0x01;

        if (frame == 0)
            result->mox = mox;

        // TX frequency at C&C address 0x02
        if (addr == 0x02) {
            result->freq = ((uint32_t)ptr[4] << 24) |
                           ((uint32_t)ptr[5] << 16) |
                           ((uint32_t)ptr[6] <<  8) |
                           ((uint32_t)ptr[7]);
        }

        ptr += 8;  // skip sync + C&C header

        for (int j = 0; j < 63 && result->n_samples < SAMPLES_PER_PKT; j++) {
            int16_t is = (int16_t)(((uint16_t)ptr[4] << 8) | (uint16_t)ptr[5]);
            int16_t qs = (int16_t)(((uint16_t)ptr[6] << 8) | (uint16_t)ptr[7]);

            result->iq[result->n_samples * 2 + 0] = (float)is / 32768.0f;
            result->iq[result->n_samples * 2 + 1] = (float)qs / 32768.0f;

            ptr += 8;
            result->n_samples++;
        }
    }

    return result->n_samples;
}