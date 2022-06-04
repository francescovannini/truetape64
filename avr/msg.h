#ifndef TRUETAPE64_MSG_H
#define TRUETAPE64_MSG_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

union msg_header {
    struct {
        uint8_t set_mode: 1;        // 0 : PC -> TT64: must be combined with one of read_tape or write_tape
        uint8_t read_tape : 1;      // 1 : PC -> TT64: TT64 will send all next msgs with cassette_sense=1 and data payload, then one message with cassette_sense=0, then will reset
        uint8_t write_tape: 1;      // 2 : PC -> TT64: TT64 will send all next msgs with cassette_sense=1 and in the data the number of buffers the PC must send until a msg with cassette_sense=0 will be sent (in case tape is stopped) or eod is received, then will reset
        uint8_t eod: 1;             // 3 : PC -> TT64: End Of Data when writing tape, TT64 will reset
        uint8_t cassette_sense: 1;  // 4 : PC <- TT64: when 1, PLAY is pressed on the Datassette
        uint8_t error_detected: 1;  // 5 : PC <- TT64: an error has been detected. The first 8 bits of the data are the error fields. TT64 needs to be reset manually.
    } fields;
    unsigned char byte_value;
};

typedef struct {
    union msg_header header;
    uint32_t data: 26;
    uint8_t checksum: 6;
} msg_t;

bool checksum(volatile msg_t *msg, volatile msg_t *msg_cs_update, volatile msg_t *msg_cs_compare, volatile flags_t *fl);

#endif //TRUETAPE64_MSG_H
