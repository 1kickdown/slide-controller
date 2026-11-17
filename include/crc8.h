#ifndef CRC8_H
#define CRC8_H

#include <stddef.h>
#include <stdint.h>

/* CRC-8, polynomial 0x07, initial value 0 */
uint8_t crc8(const uint8_t *data, size_t len);

#endif
