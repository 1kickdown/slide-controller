#include "crc8.h"

uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0u;
    size_t i;
    int bit;

    for (i = 0; i < len; i++) {
        crc = (uint8_t)(crc ^ data[i]);
        for (bit = 0; bit < 8; bit++) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((unsigned)(crc << 1) ^ 0x07u);
            } else {
                crc = (uint8_t)((unsigned)crc << 1);
            }
        }
    }
    return crc;
}
