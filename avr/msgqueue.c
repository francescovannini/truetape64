#include <util/atomic.h>
#include <stddef.h>
#include "msgqueue.h"

void msgqueue_init(volatile msgqueue_t *buf) {
    buf->head = -1;
    buf->tail = -1;
    buf->count = 0;
}

void msgqueue_push(volatile msgqueue_t *buf, volatile msg_t *msg, volatile flags_t *fl) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (buf->count == MSGQ_SIZE) {
            if (fl != NULL) {
                *fl |= 1 << FL_ERR_QUEUE_FULL;
            }
            return;
        }

        buf->head++;
        if (buf->head > MSGQ_SIZE - 1) {
            buf->head = 0;
        }
        buf->buffer[buf->head] = *msg;
        buf->count++;
    }
}

void msgqueue_pop(volatile msgqueue_t *buf, volatile msg_t *msg, volatile flags_t *fl) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (buf->count == 0) {
            if (fl != NULL) {
                *fl |= 1 << FL_ERR_QUEUE_EMPTY;
            }
            return;
        }

        buf->tail++;
        if (buf->tail > MSGQ_SIZE - 1) {
            buf->tail = 0;
        }

        *msg = buf->buffer[buf->tail];
        buf->count--;
    }
}

uint8_t msgqueue_count(volatile msgqueue_t *buf) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return buf->count;
    }
    return buf->count;
}

