#ifndef TRUETAPE64_MSG_H
#define TRUETAPE64_MSG_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define MSG_DATA_LEN  10 // must be even!!
#define MSG_PLS8_LEN  MSG_DATA_LEN
#define MSG_PLS16_LEN MSG_DATA_LEN / 2

typedef uint8_t eflags_t;
#define FL_ERR_GENERIC          0
#define FL_ERR_SEND_QUEUE_EMPTY 1
#define FL_ERR_SEND_QUEUE_FULL  2
#define FL_ERR_RECV_QUEUE_EMPY  3
#define FL_ERR_RECV_QUEUE_FULL  4
#define FL_ERR_FRAMING          5

union msg_header {
    struct {
        uint8_t set_mode: 1;        // 0 : PC -> TT64: must be combined with one of read_tape or write_tape
        uint8_t read_tape: 1;       // 1 : PC -> TT64: TT64 will send all next msgs with cassette_sense=1 and payload, then one message with cassette_sense=0, then will reset
        uint8_t write_tape: 1;      // 2 : PC -> TT64: TT64 will send all next msgs with cassette_sense=1 and in the payload the number of buffers the PC must send until a msg with cassette_sense=0 will be sent (in case tape is stopped) or eod is received, then will reset
        uint8_t eod: 1;             // 3 : PC -> TT64: End Of Data when writing tape, TT64 will reset
        uint8_t cassette_sense: 1;  // 4 : PC <- TT64: when 1, PLAY is pressed on the Datassette
        uint8_t error_detected: 1;  // 5 : PC <- TT64: an error has been detected. The first 8 bits of the payload are the error fields. TT64 needs to be reset manually.
        uint8_t ack: 1;             // 6 : PC <- TT64: TT64 acknowledging write request; payload is ack_t
        uint8_t ext_pulse_len: 1;   // 7 : PC <> TT64: length of data in the payload field is encoded as 16-bits value
    } fields;
    unsigned char byte_value;
};

typedef struct ack_t {
    uint8_t available_msg;
    eflags_t error_flags;
} ack_t;

typedef struct msg_t {
    union msg_header header;
    union {
        ack_t ack;
        uint8_t bytes[MSG_DATA_LEN];
        uint8_t pulse8[MSG_PLS8_LEN];
        uint16_t pulse16[MSG_PLS16_LEN];
    } data;
} msg_t;

#endif //TRUETAPE64_MSG_H
