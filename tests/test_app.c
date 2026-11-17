#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "crc8.h"
#include "proto.h"
#include "ringbuf.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        g_checks++;                                                             \
        if (!(cond)) {                                                          \
            g_failures++;                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
        }                                                                       \
    } while (0)

static void send_frame(app_t *a, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[16];
    size_t n = proto_encode(cmd, payload, len, buf, sizeof(buf));
    size_t i;

    for (i = 0; i < n; i++) {
        (void)app_rx_byte(a, buf[i]);
    }
}

static void send_enable(app_t *a, uint8_t on)
{
    send_frame(a, CMD_ENABLE, &on, 1);
}

static void send_target(app_t *a, int16_t tenth_mm)
{
    uint8_t p[2];

    p[0] = (uint8_t)((uint16_t)tenth_mm & 0xFFu);
    p[1] = (uint8_t)((uint16_t)tenth_mm >> 8);
    send_frame(a, CMD_SET_TARGET, p, 2);
}

static void test_crc8(void)
{
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

    CHECK(crc8(check, sizeof(check)) == 0xF4u);
    CHECK(crc8(check, 0) == 0u);
}

static void test_ringbuf(void)
{
    ringbuf_t rb;
    uint8_t b = 0;
    size_t i;

    rb_init(&rb);
    CHECK(!rb_pop(&rb, &b));
    for (i = 0; i < RB_CAPACITY; i++) {
        CHECK(rb_push(&rb, (uint8_t)i));
    }
    CHECK(!rb_push(&rb, 0xFFu));
    CHECK(rb_count(&rb) == RB_CAPACITY);
    for (i = 0; i < RB_CAPACITY; i++) {
        CHECK(rb_pop(&rb, &b));
        CHECK(b == (uint8_t)i);
    }
    CHECK(rb_count(&rb) == 0u);

    /* wrap around */
    for (i = 0; i < 3u * RB_CAPACITY; i++) {
        CHECK(rb_push(&rb, (uint8_t)(i + 7u)));
        CHECK(rb_pop(&rb, &b));
        CHECK(b == (uint8_t)(i + 7u));
    }
}

static void test_encode_limits(void)
{
    uint8_t buf[16];
    uint8_t payload[PROTO_MAX_PAYLOAD + 1u] = {0};

    CHECK(proto_encode(CMD_STATUS_REQ, NULL, 0, buf, sizeof(buf)) == 4u);
    CHECK(proto_encode(CMD_ENABLE, payload, PROTO_MAX_PAYLOAD, buf, sizeof(buf)) ==
          PROTO_MAX_PAYLOAD + 4u);
    CHECK(proto_encode(CMD_ENABLE, payload, PROTO_MAX_PAYLOAD + 1u, buf, sizeof(buf)) == 0u);
    CHECK(proto_encode(CMD_ENABLE, payload, 2, buf, 5) == 0u);
}

static proto_result_t feed_all(proto_parser_t *p, const uint8_t *data, size_t n,
                               proto_msg_t *msg, int *messages)
{
    proto_result_t last = PROTO_PENDING;
    size_t i;

    for (i = 0; i < n; i++) {
        proto_result_t r = proto_feed(p, data[i], msg);

        if (r == PROTO_MESSAGE) {
            (*messages)++;
        }
        last = r;
    }
    return last;
}

static void test_parser_roundtrip_and_resync(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    uint8_t buf[16];
    uint8_t payload[2] = {0x34, 0x12};
    uint8_t garbage[] = {0x00, 0xFF, 0x13, 0x37};
    size_t n;
    int messages = 0;

    proto_init(&p);
    n = proto_encode(CMD_SET_TARGET, payload, 2, buf, sizeof(buf));
    CHECK(feed_all(&p, garbage, sizeof(garbage), &msg, &messages) == PROTO_PENDING);
    CHECK(feed_all(&p, buf, n, &msg, &messages) == PROTO_MESSAGE);
    CHECK(messages == 1);
    CHECK(msg.cmd == CMD_SET_TARGET);
    CHECK(msg.len == 2u);
    CHECK(msg.payload[0] == 0x34u && msg.payload[1] == 0x12u);
}

static void test_parser_rejects_bad_crc_and_length(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    uint8_t buf[16];
    uint8_t on = 1;
    size_t n;
    int messages = 0;
    uint8_t bad_len[] = {PROTO_SOF, 0x00};
    uint8_t too_long[] = {PROTO_SOF, 0x20};

    proto_init(&p);
    n = proto_encode(CMD_ENABLE, &on, 1, buf, sizeof(buf));
    buf[n - 1u] ^= 0x01u;
    CHECK(feed_all(&p, buf, n, &msg, &messages) == PROTO_BAD_CRC);
    CHECK(messages == 0);
    CHECK(p.crc_errors == 1u);

    CHECK(feed_all(&p, bad_len, sizeof(bad_len), &msg, &messages) == PROTO_BAD_FRAME);
    CHECK(feed_all(&p, too_long, sizeof(too_long), &msg, &messages) == PROTO_BAD_FRAME);
    CHECK(p.framing_errors == 2u);

    /* a valid frame right after the errors is still accepted */
    n = proto_encode(CMD_ENABLE, &on, 1, buf, sizeof(buf));
    CHECK(feed_all(&p, buf, n, &msg, &messages) == PROTO_MESSAGE);
    CHECK(messages == 1);
}

static void test_parser_random_bytes(void)
{
    proto_parser_t p;
    proto_msg_t msg;
    uint32_t seed = 12345u;
    int i;

    proto_init(&p);
    for (i = 0; i < 200000; i++) {
        proto_result_t r;

        seed = seed * 1664525u + 1013904223u;
        r = proto_feed(&p, (uint8_t)(seed >> 24), &msg);
        CHECK(r == PROTO_PENDING || r == PROTO_MESSAGE || r == PROTO_BAD_CRC ||
              r == PROTO_BAD_FRAME);
        CHECK(p.idx <= PROTO_MAX_PAYLOAD + 1u);
        if (r == PROTO_MESSAGE) {
            CHECK(msg.len <= PROTO_MAX_PAYLOAD);
        }
    }
}

static void test_app_enable_home_and_target(void)
{
    app_t a;
    float duty = 1.0f;
    uint8_t tx[16];

    app_init(&a);
    (void)app_tick(&a, 0, false, &duty, tx, sizeof(tx));
    CHECK(a.slide.mode == SLIDE_DISABLED);
    CHECK(duty == 0.0f);

    send_enable(&a, 1);
    send_target(&a, 3000);
    (void)app_tick(&a, 0, false, &duty, tx, sizeof(tx));
    CHECK(a.slide.mode == SLIDE_HOMING);
    CHECK(duty == a.cfg.homing_duty);
    CHECK(fabsf(a.target_mm - 300.0f) < 1e-3f);

    (void)app_tick(&a, 0, true, &duty, tx, sizeof(tx));
    CHECK(a.slide.mode == SLIDE_POSITION);
    CHECK(a.slide.homed);

    send_enable(&a, 0);
    (void)app_tick(&a, 0, false, &duty, tx, sizeof(tx));
    CHECK(a.slide.mode == SLIDE_DISABLED);
    CHECK(duty == 0.0f);
}

static void test_app_negative_target(void)
{
    app_t a;
    float duty;
    uint8_t tx[16];

    app_init(&a);
    send_target(&a, -125);
    (void)app_tick(&a, 0, false, &duty, tx, sizeof(tx));
    CHECK(fabsf(a.target_mm + 12.5f) < 1e-3f);
}

static void test_app_status_reply(void)
{
    app_t a;
    float duty;
    uint8_t tx[16];
    proto_parser_t p;
    proto_msg_t msg;
    int messages = 0;
    size_t n;

    app_init(&a);
    send_enable(&a, 1);
    (void)app_tick(&a, 0, false, &duty, tx, sizeof(tx));
    (void)app_tick(&a, 0, true, &duty, tx, sizeof(tx));

    send_frame(&a, CMD_STATUS_REQ, NULL, 0);
    n = app_tick(&a, 0, false, &duty, tx, sizeof(tx));
    CHECK(n == 9u);

    proto_init(&p);
    CHECK(feed_all(&p, tx, n, &msg, &messages) == PROTO_MESSAGE);
    CHECK(msg.cmd == CMD_STATUS);
    CHECK(msg.len == 5u);
    CHECK(msg.payload[0] == (uint8_t)SLIDE_POSITION);
    CHECK(msg.payload[1] == 1u);

    /* no reply without a request */
    CHECK(app_tick(&a, 0, false, &duty, tx, sizeof(tx)) == 0u);
}

static void test_app_ignores_corrupt_frames(void)
{
    app_t a;
    float duty;
    uint8_t tx[16];
    uint8_t buf[16];
    uint8_t on = 1;
    size_t n;
    size_t i;

    app_init(&a);
    n = proto_encode(CMD_ENABLE, &on, 1, buf, sizeof(buf));
    buf[2] ^= 0x10u;
    for (i = 0; i < n; i++) {
        (void)app_rx_byte(&a, buf[i]);
    }
    (void)app_tick(&a, 0, false, &duty, tx, sizeof(tx));
    CHECK(!a.enabled);
    CHECK(a.parser.crc_errors == 1u);
}

static void test_app_rx_overflow_is_counted(void)
{
    app_t a;
    size_t i;

    app_init(&a);
    for (i = 0; i < RB_CAPACITY + 5u; i++) {
        (void)app_rx_byte(&a, 0u);
    }
    CHECK(a.rx_overflows == 5u);
}

int main(void)
{
    test_crc8();
    test_ringbuf();
    test_encode_limits();
    test_parser_roundtrip_and_resync();
    test_parser_rejects_bad_crc_and_length();
    test_parser_random_bytes();
    test_app_enable_home_and_target();
    test_app_negative_target();
    test_app_status_reply();
    test_app_ignores_corrupt_frames();
    test_app_rx_overflow_is_counted();

    printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
