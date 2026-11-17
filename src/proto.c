#include "proto.h"

#include <string.h>

#include "crc8.h"

void proto_init(proto_parser_t *p)
{
    p->state = PS_SOF;
    p->len = 0u;
    p->idx = 0u;
    memset(p->body, 0, sizeof(p->body));
    p->crc_errors = 0u;
    p->framing_errors = 0u;
}

proto_result_t proto_feed(proto_parser_t *p, uint8_t byte, proto_msg_t *msg)
{
    uint8_t frame[PROTO_MAX_PAYLOAD + 2u];

    switch (p->state) {
    case PS_SOF:
        if (byte == PROTO_SOF) {
            p->state = PS_LEN;
        }
        return PROTO_PENDING;
    case PS_LEN:
        if (byte < 1u || byte > PROTO_MAX_PAYLOAD + 1u) {
            p->state = PS_SOF;
            p->framing_errors++;
            return PROTO_BAD_FRAME;
        }
        p->len = byte;
        p->idx = 0u;
        p->state = PS_BODY;
        return PROTO_PENDING;
    case PS_BODY:
        p->body[p->idx] = byte;
        p->idx++;
        if (p->idx == p->len) {
            p->state = PS_CRC;
        }
        return PROTO_PENDING;
    case PS_CRC:
    default:
        p->state = PS_SOF;
        frame[0] = p->len;
        memcpy(&frame[1], p->body, p->len);
        if (crc8(frame, (size_t)p->len + 1u) != byte) {
            p->crc_errors++;
            return PROTO_BAD_CRC;
        }
        msg->cmd = p->body[0];
        msg->len = (uint8_t)(p->len - 1u);
        memcpy(msg->payload, &p->body[1], msg->len);
        return PROTO_MESSAGE;
    }
}

size_t proto_encode(uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out,
                    size_t cap)
{
    size_t total = (size_t)len + 4u;
    uint8_t body_len = (uint8_t)(len + 1u);

    if (len > PROTO_MAX_PAYLOAD || cap < total) {
        return 0u;
    }
    out[0] = PROTO_SOF;
    out[1] = body_len;
    out[2] = cmd;
    if (len > 0u) {
        memcpy(&out[3], payload, len);
    }
    out[total - 1u] = crc8(&out[1], (size_t)body_len + 1u);
    return total;
}
