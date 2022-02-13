#ifndef TRUETAPE64_MSGBUFFER_H
#define TRUETAPE64_MSGBUFFER_H

#include <stdbool.h>
#include <stdint.h>
#include "msg.h"
#include "main.h"

typedef struct { ;
    volatile msg_t buffer[BUF_SIZE];
    volatile int8_t head;
    volatile int8_t tail;
    volatile uint8_t count;
} msgbuffer_t;

void msgbuffer_init(volatile msgbuffer_t *buf);
bool msgbuffer_push(volatile msgbuffer_t *buf, volatile msg_t *msg);
bool msgbuffer_pop(volatile msgbuffer_t *buf, volatile msg_t *msg);

#endif //TRUETAPE64_MSGBUFFER_H
