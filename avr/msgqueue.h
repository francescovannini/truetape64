#ifndef TRUETAPE64_MSGQUEUE_H
#define TRUETAPE64_MSGQUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include "msg.h"
#include "main.h"

typedef struct msgqueue_t { ;
    volatile uint8_t size;
    volatile uint8_t offset;
    volatile int8_t head;
    volatile int8_t tail;
    volatile uint8_t count;
} msgqueue_t;

void msgqueue_init(volatile struct msgqueue_t *q, volatile uint8_t offset, volatile uint8_t size);
void msgqueue_push(volatile struct msgqueue_t *q, volatile struct msg_t *msg, volatile eflags_t *fl, uint8_t flagbit);
void msgqueue_pop(volatile struct msgqueue_t *q, volatile struct msg_t *msg, volatile eflags_t *fl, uint8_t flagbit);
msg_t* msgqueue_peek(volatile msgqueue_t *q);
uint8_t msgqueue_count(volatile struct msgqueue_t *q);

#endif //TRUETAPE64_MSGQUEUE_H
