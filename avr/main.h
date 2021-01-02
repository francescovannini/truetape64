#ifndef TRUETAPE64_MAIN_H
#define TRUETAPE64_MAIN_H

#include <inttypes.h>

#define BUF_SIZE            24
#define MIN_PULSE_LEN       9       // Less than this, pulse would appear as 0 in the TAP

#define UART_UBRR           3       // 250kbps at 16Mhz

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

typedef struct {
    uint16_t tm_counter;
    uint8_t tm_overflows;
} pulselen_t;

#define CLR_CASSETTE_SENSE  SENSE_OUT_PORT &= ~(1 << SENSE_OUT_PIN)
#define SET_CASSETTE_SENSE  SENSE_OUT_PORT |= 1 << SENSE_OUT_PIN

#endif //TRUETAPE64_MAIN_H
