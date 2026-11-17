#include "app.h"

#include <math.h>

static void handle_message(app_t *a, const proto_msg_t *m)
{
    switch (m->cmd) {
    case CMD_ENABLE:
        if (m->len == 1u) {
            a->enabled = m->payload[0] != 0u;
        }
        break;
    case CMD_SET_TARGET:
        if (m->len == 2u) {
            uint16_t raw = (uint16_t)((uint16_t)m->payload[0] | (uint16_t)(m->payload[1] << 8));

            a->target_mm = (float)(int16_t)raw / 10.0f;
        }
        break;
    case CMD_STATUS_REQ:
        a->status_pending = true;
        break;
    default:
        break;
    }
}

static int16_t to_tenth_mm(float mm)
{
    float v = roundf(mm * 10.0f);

    if (v > 32767.0f) {
        v = 32767.0f;
    }
    if (v < -32768.0f) {
        v = -32768.0f;
    }
    return (int16_t)v;
}

static size_t encode_status(const app_t *a, const slide_output_t *out, uint8_t *tx,
                            size_t cap)
{
    uint8_t payload[5];
    uint16_t pos = (uint16_t)to_tenth_mm(a->slide.prev_pos_mm);
    int8_t duty_pct = (int8_t)roundf(out->duty * 100.0f);

    payload[0] = (uint8_t)out->mode;
    payload[1] = out->homed ? 1u : 0u;
    payload[2] = (uint8_t)(pos & 0xFFu);
    payload[3] = (uint8_t)(pos >> 8);
    payload[4] = (uint8_t)duty_pct;
    return proto_encode(CMD_STATUS, payload, (uint8_t)sizeof(payload), tx, cap);
}

void app_init(app_t *a)
{
    slide_default_config(&a->cfg);
    slide_init(&a->slide);
    rb_init(&a->rx);
    proto_init(&a->parser);
    a->enabled = false;
    a->target_mm = 0.0f;
    a->status_pending = false;
    a->rx_overflows = 0u;
}

bool app_rx_byte(app_t *a, uint8_t byte)
{
    if (!rb_push(&a->rx, byte)) {
        a->rx_overflows++;
        return false;
    }
    return true;
}

size_t app_tick(app_t *a, int32_t encoder_ticks, bool bottom_switch, float *duty,
                uint8_t *tx, size_t tx_cap)
{
    slide_input_t in;
    slide_output_t out;
    uint8_t byte;
    size_t written = 0u;

    while (rb_pop(&a->rx, &byte)) {
        proto_msg_t msg;

        if (proto_feed(&a->parser, byte, &msg) == PROTO_MESSAGE) {
            handle_message(a, &msg);
        }
    }

    in.enabled = a->enabled;
    in.encoder_ticks = encoder_ticks;
    in.bottom_switch = bottom_switch;
    in.target_mm = a->target_mm;
    out = slide_step(&a->slide, &a->cfg, &in);
    *duty = out.duty;

    if (a->status_pending) {
        written = encode_status(a, &out, tx, tx_cap);
        if (written > 0u) {
            a->status_pending = false;
        }
    }
    return written;
}
