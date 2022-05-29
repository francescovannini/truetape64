#ifndef TRUETAPE64_MSGQUEUE_H
#define TRUETAPE64_MSGQUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include "msg.h"
#include "main.h"

typedef struct { ;
    volatile msg_t buffer[MSGQ_SIZE];
    volatile int8_t head;
    volatile int8_t tail;
    volatile uint8_t count;
} msgqueue_t;

void msgqueue_init(volatile msgqueue_t *buf);
void msgqueue_push(volatile msgqueue_t *buf, volatile msg_t *msg, volatile flags_t *fl);
void msgqueue_pop(volatile msgqueue_t *buf, volatile msg_t *msg, volatile flags_t *fl);
uint8_t msgqueue_count(volatile msgqueue_t *buf);

#endif //TRUETAPE64_MSGQUEUE_H
