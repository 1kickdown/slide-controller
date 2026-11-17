#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "proto.h"
#include "ringbuf.h"
#include "slide.h"

typedef struct {
    slide_config_t cfg;
    slide_state_t  slide;
    ringbuf_t      rx;
    proto_parser_t parser;
    bool           enabled;
    float          target_mm;
    bool           status_pending;
    uint32_t       rx_overflows;
} app_t;

void app_init(app_t *a);

/* Called from the receive interrupt: only queues the byte. */
bool app_rx_byte(app_t *a, uint8_t byte);

/*
 * One control period: parses queued bytes, runs the controller and, if a status
 * was requested, writes a status frame to tx. Returns the number of bytes in tx.
 */
size_t app_tick(app_t *a, int32_t encoder_ticks, bool bottom_switch, float *duty,
                uint8_t *tx, size_t tx_cap);

#endif
