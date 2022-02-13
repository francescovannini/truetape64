#ifndef TRUETAPE64_MSG_H
#define TRUETAPE64_MSG_H

#include <stdint.h>

union msg_header {
    struct {
        uint8_t init : 1;
        uint8_t read : 1;
        uint8_t write: 1;
        uint8_t error: 1;
        uint8_t sense: 1;
    } flags;
    unsigned char byte_value;
};

typedef struct {
    union msg_header header;
    uint32_t data : 26;
    uint8_t checksum : 6;
} msg_t;

uint8_t compute_checksum(volatile msg_t *msg);

#endif //TRUETAPE64_MSG_H
