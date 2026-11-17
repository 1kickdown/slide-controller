#ifndef PROTO_H
#define PROTO_H

#include <stddef.h>
#include <stdint.h>

/* Frame: SOF | LEN | CMD | payload... | CRC8(LEN, CMD, payload) */
#define PROTO_SOF 0xA5u
#define PROTO_MAX_PAYLOAD 6u

#define CMD_ENABLE 0x01u      /* payload: 1 byte, 0 = off, else on            */
#define CMD_SET_TARGET 0x02u  /* payload: int16 little endian, 0.1 mm units   */
#define CMD_STATUS_REQ 0x03u
#define CMD_STATUS 0x83u      /* mode, homed, pos int16 (0.1 mm), duty int8 % */

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_msg_t;

typedef enum {
    PS_SOF = 0,
    PS_LEN,
    PS_BODY,
    PS_CRC
} proto_state_t;

typedef struct {
    proto_state_t state;
    uint8_t       len;
    uint8_t       idx;
    uint8_t       body[PROTO_MAX_PAYLOAD + 1u];
    uint32_t      crc_errors;
    uint32_t      framing_errors;
} proto_parser_t;

typedef enum {
    PROTO_PENDING = 0,
    PROTO_MESSAGE,
    PROTO_BAD_CRC,
    PROTO_BAD_FRAME
} proto_result_t;

void           proto_init(proto_parser_t *p);
proto_result_t proto_feed(proto_parser_t *p, uint8_t byte, proto_msg_t *msg);

/* Returns the frame length, or 0 if the payload is too long or cap too small. */
size_t proto_encode(uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out,
                    size_t cap);

#endif
