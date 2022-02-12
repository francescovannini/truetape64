#ifndef TRUETAPE64_MAIN_H
#define TRUETAPE64_MAIN_H

#include <inttypes.h>

#define BUF_SIZE            4
#define MIN_PULSE_LEN       9           // Less than this, pulse would appear as 0 in the TAP
#define MAX_PULSE_LEN       67108863    // Only 26 bits are available for message data so 2^26 - 1

#define UART_UBRR           3           // 250kbps at 16Mhz

#define LED_PIN             PB2
#define LED_PORT            PORTB
#define LED_PORT_DDR        DDRB

#define SENSE_IN_PIN        PB0
#define SENSE_IN_PINS       PINB
#define SENSE_IN_PORT       PORTB
#define SENSE_IN_PORT_DDR   DDRB

#define SENSE_OUT_PIN       PD3
#define SENSE_OUT_PORT      PORTD
#define SENSE_OUT_PORT_DDR  DDRD

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

//typedef struct {
//    uint16_t tm_counter;
//    uint16_t tm_overflows;
//} pulselen_t;

typedef struct {
    union msg_header header;
    uint32_t data : 26;
    uint8_t checksum : 6;
} msg_t;

#define CLR_CASSETTE_SENSE  SENSE_OUT_PORT &= ~(1 << SENSE_OUT_PIN)
#define SET_CASSETTE_SENSE  SENSE_OUT_PORT |= 1 << SENSE_OUT_PIN

#endif //TRUETAPE64_MAIN_H
