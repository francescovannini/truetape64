#ifndef TRUETAPE64_MAIN_H
#define TRUETAPE64_MAIN_H

#include <inttypes.h>

typedef struct {
    uint16_t tm_counter;
    uint8_t tm_overflows;
} pulselen_t;


#define BUF_SIZE            24
#define MIN_PULSE_LEN       9       // Less than this, pulse would appear as 0 in the TAP

#define CONTROL_PORT_DDR    DDRB
#define CONTROL_PORT_OUT    PORTB
#define CONTROL_PORT_IN     PINB

#define SENSE_IN_PIN        PB0
#define SENSE_OUT_PIN       PB1
#define LED_PIN             PB2
#define UART_UBRR           3       // 250kbps at 16Mhz

#define CLR_CASSETTE_SENSE  CONTROL_PORT_OUT &= ~(1 << SENSE_OUT_PIN)
#define SET_CASSETTE_SENSE  CONTROL_PORT_OUT |= 1 << SENSE_OUT_PIN

#endif //TRUETAPE64_MAIN_H
