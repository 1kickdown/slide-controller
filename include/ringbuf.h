#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RB_CAPACITY 64u

typedef struct {
    uint8_t data[RB_CAPACITY];
    size_t  head;
    size_t  tail;
    size_t  count;
} ringbuf_t;

void   rb_init(ringbuf_t *rb);
bool   rb_push(ringbuf_t *rb, uint8_t byte);
bool   rb_pop(ringbuf_t *rb, uint8_t *byte);
size_t rb_count(const ringbuf_t *rb);

#endif
