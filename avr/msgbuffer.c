#include <util/atomic.h>
#include "msgbuffer.h"

void msgbuffer_init(volatile msgbuffer_t *buf) {
    buf->head = -1;
    buf->tail = -1;
    buf->count = 0;
}

bool msgbuffer_push(volatile msgbuffer_t *buf, volatile msg_t *msg) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (buf->count == BUF_SIZE) {
            return false;
        }

        buf->head++;
        if (buf->head > BUF_SIZE - 1) {
            buf->head = 0;
        }
        buf->buffer[buf->head] = *msg;
        buf->count++;
    }

    return true;
}

bool msgbuffer_pop(volatile msgbuffer_t *buf, volatile msg_t *msg) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (buf->count == 0) {
            return false;
        }

        buf->tail++;
        if (buf->tail > BUF_SIZE - 1) {
            buf->tail = 0;
        }

        *msg = buf->buffer[buf->tail];
        buf->count--;
    }

    return true;

}
