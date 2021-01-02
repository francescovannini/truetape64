#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include "main.h"

volatile uint8_t tm_overflows;
volatile pulselen_t pulse_buf[BUF_SIZE];
volatile int8_t pulse_buf_h = -1, pulse_buf_t = -1;
volatile bool pulse_buf_ovf = false;
volatile bool cassette_sense = false;

inline void enqueue_pulse(const volatile uint8_t *tm_overflow, const volatile uint16_t *tm_counter) {

    if ((!cassette_sense) || pulse_buf_ovf) {
        return;
    }

    // Skip too short pulses
    if ((*tm_overflow == 0) && (*tm_counter < MIN_PULSE_LEN)) {
        return;
    }

    if (pulse_buf_h == BUF_SIZE - 1) {
        pulse_buf_ovf = true;
        return;
    }

    if (pulse_buf_h == -1) {
        pulse_buf_h = 0;
    }

    pulse_buf_t++;
    pulse_buf[pulse_buf_t].tm_counter = *tm_counter;
    pulse_buf[pulse_buf_t].tm_overflows = *tm_overflow;
}

inline void send_pulse_data(const volatile pulselen_t *p) {
    uint8_t checksum = 64;

    for (uint8_t i = 0; i < (uint8_t) sizeof(pulselen_t); i++) {
        uint8_t *t = ((uint8_t *) p) + i;
        while (!(UCSRA & (1 << UDRE))) {}
        UDR = *t;
        checksum += *t;
    }

    while (!(UCSRA & (1 << UDRE))) {}
    UDR = checksum;
}

int main(void) {
    bool tmp_sense;

    // Status led
    LED_PORT_DDR |= 1 << LED_PIN;

    // Cassette sense input, Datassette F-6 pin to PB1
    SENSE_IN_PORT_DDR &= ~(1 << SENSE_IN_PIN);
    SENSE_IN_PORT |= 1 << SENSE_IN_PIN; // Pull-up

    // Cassette sense output to serial CTS
    SENSE_OUT_PORT_DDR |= 1 << SENSE_OUT_PIN;
    SET_CASSETTE_SENSE;

    // Capturing falling edges with Timer/Counter 1
    DDRD &= ~(1 << PD6); // Input on PD6
    TIMSK |= (1 << ICIE1) | (1 << TOIE1);     //Set capture interrupt and overflow interrupt
    TCCR1B = (0 << ICNC1) | (0 << ICES1)
             | (0 << CS12) | (1 << CS11) | (0 << CS10);  //Set capture falling edge, /8 prescaler

    // USART init
    UBRRL = UART_UBRR;
    UBRRH = UART_UBRR >> 8;
    UCSRB = (1 << RXEN) | (1 << TXEN) | (0 << U2X); // Enable RX and TX, disable 2X mode
    UCSRC = (0 << USBS) | (1 << UCSZ1) | (1 << UCSZ0); // 8, N, 1

    // Blink when boot done
    LED_PORT |= 1 << LED_PIN;
    _delay_ms(500);
    LED_PORT &= ~(1 << LED_PIN);

    sei();
    for (;;) {

        tmp_sense = !(SENSE_IN_PORT & (1 << SENSE_IN_PIN) >> SENSE_IN_PIN); //FIXME Debounce maybe?
        if (tmp_sense != cassette_sense) {
            cassette_sense = tmp_sense;
            if (cassette_sense) {
                CLR_CASSETTE_SENSE;
            } else {
                SET_CASSETTE_SENSE;
                pulse_buf_ovf = false;
            }
        }

        // Send data in buffer
        if (pulse_buf_h >= 0) {
            send_pulse_data(&pulse_buf[pulse_buf_h]);
            pulse_buf_h++;
            if (pulse_buf_h > pulse_buf_t) {
                pulse_buf_h = -1;
                pulse_buf_t = -1;
            }
        }

        if (pulse_buf_ovf) {
            SET_CASSETTE_SENSE;
            for (uint8_t blink = 0; blink < 5; blink++) {
                LED_PORT |= 1 << LED_PIN;
                _delay_ms(100);
                LED_PORT &= ~(1 << LED_PIN);
                _delay_ms(100);
            }
        } else {
            LED_PORT &= ~(1 << LED_PIN);
        }
    }
}

ISR(TIMER1_CAPT_vect) {
    uint16_t tmp = ICR1;
    TCNT1 = 0;
    enqueue_pulse(&tm_overflows, &tmp);
    tm_overflows = 0;
}

ISR(TIMER1_OVF_vect) {
    const uint16_t max = 0xFFFF;
    if (tm_overflows < 0xFF) {
        tm_overflows++;
    } else {
        enqueue_pulse(&tm_overflows, &max);
        tm_overflows = 0;
    }
}

//ISR(BADISR_vect)
//{
//    PORTB |= 1 << PB0;
//    for (;;);
//}

#pragma clang diagnostic pop
#pragma clang diagnostic pop