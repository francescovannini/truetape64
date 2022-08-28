#include <util/atomic.h>
#include <stddef.h>
#include "msgqueue.h"

msg_t msg_buffer[MSGQ_SIZE];

void msgqueue_init(volatile struct msgqueue_t *q, volatile uint8_t offset, volatile uint8_t size) {
    q->size = size;
    q->head = -1;
    q->tail = -1;
    q->count = 0;
    q->offset = offset;
}

void msgqueue_push(volatile msgqueue_t *q, volatile msg_t *msg, volatile eflags_t *fl, uint8_t flagbit) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (q->count == q->size) {
            if (fl != NULL) {
                *fl |= 1 << flagbit;
            }
            return;
        }

        q->head++;
        if (q->head > q->size - 1) {
            q->head = 0;
        }

        msg_buffer[q->head + q->offset] = *msg;

        q->count++;
    }
}

void msgqueue_pop(volatile msgqueue_t *q, volatile msg_t *msg, volatile eflags_t *fl, uint8_t flagbit) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (q->count == 0) {
            if (fl != NULL) {
                *fl |= 1 << flagbit;
            }
            return;
        }

        q->tail++;
        if (q->tail > q->size - 1) {
            q->tail = 0;
        }

        *msg = msg_buffer[q->tail + q->offset];
        q->count--;
    }
}

msg_t* msgqueue_peek(volatile msgqueue_t *q) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (q->count > 0) {
            return &msg_buffer[q->tail + q->offset];
        }
    }
    return NULL;
}

uint8_t msgqueue_count(volatile msgqueue_t *q) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return q->count;
    }
    return q->count;
}
