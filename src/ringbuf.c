#include "ringbuf.h"

void rb_init(ringbuf_t *rb)
{
    rb->head = 0u;
    rb->tail = 0u;
    rb->count = 0u;
}

bool rb_push(ringbuf_t *rb, uint8_t byte)
{
    if (rb->count == RB_CAPACITY) {
        return false;
    }
    rb->data[rb->head] = byte;
    rb->head = (rb->head + 1u) % RB_CAPACITY;
    rb->count++;
    return true;
}

bool rb_pop(ringbuf_t *rb, uint8_t *byte)
{
    if (rb->count == 0u) {
        return false;
    }
    *byte = rb->data[rb->tail];
    rb->tail = (rb->tail + 1u) % RB_CAPACITY;
    rb->count--;
    return true;
}

size_t rb_count(const ringbuf_t *rb)
{
    return rb->count;
}
