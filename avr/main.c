#pragma ide diagnostic ignored "hicpp-signed-bitwise"
#pragma ide diagnostic ignored "EndlessLoop"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>

#include "main.h"
#include "msgbuffer.h"

volatile uint8_t tm_overflows;
volatile bool cassette_sense = false;
volatile bool error_status = false;

volatile msgbuffer_t recv_buf;
volatile msgbuffer_t send_buf;

volatile uint8_t serial_rx_buf[sizeof(msg_t)];
volatile uint8_t serial_rx_buf_count;

volatile uint8_t serial_tx_buf[sizeof(msg_t)];
volatile int8_t serial_tx_buf_pos = -1;

inline uint8_t compute_checksum(volatile const uint8_t *buf, volatile const uint8_t buf_len) {
    uint8_t checksum = 64;
    for (uint8_t i = 0; i < buf_len; i++) {
        checksum += buf[i];
    }
    return checksum;
}

// Serial has received one byte
ISR(USART_RX_vect) {
    if (serial_rx_buf_count < sizeof(msg_t)) {
        serial_rx_buf[serial_rx_buf_count] = UDR;
        serial_rx_buf_count++;
    } else { // RX buffer is full and should contain a message a this point, so next byte is the checksum
        if (compute_checksum(&serial_rx_buf[0], sizeof(msg_t)) == UDR) { // Message is valid
            error_status = !msgbuffer_push(&recv_buf, (msg_t *) &serial_rx_buf);
        }
        serial_rx_buf_count = 0;
    }
}

// Serial is ready to send the next byte
ISR(USART_UDRE_vect) {

    if (serial_tx_buf_pos >= 0) { // There is a transmission ongoing
        if (serial_tx_buf_pos < sizeof(msg_t)) { // Send next byte in TX buffer
            UDR = serial_tx_buf[serial_tx_buf_pos++];
        } else { // No more bytes in the TX buffer, so next byte to send is the checksum
            serial_tx_buf_pos = -1;
            UDR = compute_checksum(&serial_tx_buf[0], sizeof(msg_t));
        }
    } else {
        if (send_buf.count > 0) { // Pops next message from ring buffer, writes it into the TX buffer
            if (msgbuffer_pop(&send_buf, (msg_t *) &serial_tx_buf)) {
                serial_tx_buf_pos = 0;
            } else {
                error_status |= true;
            }
        } else { // Nothing else to send, disables this ISR
            UCSRB &= ~(1 << UDRIE);
        }
    }
}

// Falling edge
ISR(TIMER1_CAPT_vect) {
    uint16_t tmp = ICR1; //FIXME try to avoid this copy
    TCNT1 = 0;
    union msg_header header;
    header.byte_value = 0;
    header.flags.read = 1;
    header.flags.sense = cassette_sense > 0;
    if ((tm_overflows > 0) || (tmp > MIN_PULSE_LEN)) {
        error_status |= !msgbuffer_push_new(&send_buf, &header, &tm_overflows, &tmp);
        UCSRB |= (1 << UDRIE);
    }

    tm_overflows = 0;
}

// Timer overflow
ISR(TIMER1_OVF_vect) {
    const uint16_t max = 0xFFFF;
    union msg_header header;
    header.byte_value = 0;
    header.flags.read = 1;
    header.flags.sense = cassette_sense > 0;
    if (tm_overflows < 0xFF) {
        tm_overflows++;
    } else {
        error_status |= !msgbuffer_push_new(&send_buf, &header, &tm_overflows,
                                            &max); // pulse is longer than detectable length
        UCSRB |= (1 << UDRIE);
        tm_overflows = 0;
    }
}

int main(void) {
    bool tmp_sense;

    msgbuffer_init(&recv_buf);
    msgbuffer_init(&send_buf);

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

    TCCR1B = (0 << ICNC1) | // Noise canceller disabled
             (0 << ICES1) | // Capture on falling edge
             (0 << CS12) |  //
             (1 << CS11) |  // Prescaler CLOCK/8
             (0 << CS10);   //

    // USART setup
    UBRRL = UART_UBRR;      // Set baud rate
    UBRRH = UART_UBRR >> 8; // Set baur rate

    UCSRB = (1 << RXCIE) |  // Fires USART_RX_vect when USART has received a new byte
            (0 << UDRIE) |  // Fires USART_UDRE_vect when USART is ready to send next byte
            (1 << RXEN) |   // Enable USART reception
            (1 << TXEN);    // Enable USART transmission

    UCSRC = (1 << UCSZ1) | (1 << UCSZ0) |   // 8 bit
            (0 << UPM1) | (0 << UPM0) |   // no parity
            (0 << USBS);                    // 1 stop bit

    // Blink when boot done
    LED_PORT |= 1 << LED_PIN;
    _delay_ms(500);
    LED_PORT &= ~(1 << LED_PIN);
    _delay_ms(500);

    sei();
    for (;;) {
        tmp_sense = !(SENSE_IN_PINS & (1 << SENSE_IN_PIN) >> SENSE_IN_PIN); //FIXME Debounce maybe?
        if (tmp_sense != cassette_sense) {
            cassette_sense = tmp_sense;
            if (cassette_sense) {
                CLR_CASSETTE_SENSE;
                TIMSK |= (1 << TOIE1) | // Fires TIMER1_OVF_vect when TIMER1 overflows
                         (1 << ICIE1);  // Fires TIMER1_CAPT_vect when TIMER1 has completed a capture
            } else {
                TIMSK &= ~((1 << TOIE1) | // Fires TIMER1_OVF_vect when TIMER1 overflows
                           (1 << ICIE1));  // Fires TIMER1_CAPT_vect when TIMER1 has completed a capture
                SET_CASSETTE_SENSE;
                error_status = false;
            }
        }

        if (error_status) {
            LED_PORT |= 1 << LED_PIN;
            _delay_ms(500);
            LED_PORT &= ~(1 << LED_PIN);
            _delay_ms(500);
        }

    }
}














































//int main(void) {
//
//    // Status led
//    LED_PORT_DDR |= 1 << LED_PIN;
//
//    // USART init
//    UBRRL = UART_UBRR;
//    UBRRH = UART_UBRR >> 8;
//    UCSRB = (1 << RXEN) | (1 << TXEN) | (0 << U2X); // Enable RX and TX, disable 2X mode
//    UCSRC = (0 << USBS) | (1 << UCSZ1) | (1 << UCSZ0); // 8, N, 1
//
//    // Blink when boot done
//    LED_PORT |= 1 << LED_PIN;
//    _delay_ms(100);
//    LED_PORT &= ~(1 << LED_PIN);
//
//    sei();
//    for (;;) {
//        if (serial_receive_msg_t(&msg_rcv_buf[0])) {
//            serial_send_msg_t(&msg_rcv_buf[0]);
//        } else {
//            LED_PORT |= 1 << LED_PIN;
//            _delay_ms(100);
//            LED_PORT &= ~(1 << LED_PIN);
//            _delay_ms(100);
//            LED_PORT |= 1 << LED_PIN;
//            _delay_ms(100);
//            LED_PORT &= ~(1 << LED_PIN);
//            _delay_ms(100);
//        }
//    }
//}
//
//
//int main(void) {
//    bool tmp_sense;
//    msg_t msg;
//
//    // Status led
//    LED_PORT_DDR |= 1 << LED_PIN;
//
//    // Cassette sense input, Datassette F-6 pin to PB1
//    SENSE_IN_PORT_DDR &= ~(1 << SENSE_IN_PIN);
//    SENSE_IN_PORT |= 1 << SENSE_IN_PIN; // Pull-up
//
//    // Capturing falling edges with Timer/Counter 1
//    DDRD &= ~(1 << PD6); // Input on PD6
//    TIMSK |= (1 << ICIE1) |
//             (1 << TOIE1);     //Set capture interrupt and overflow interrupt //FIXME this only is read mode
//    TCCR1B = (0 << ICNC1) | (0 << ICES1)
//             | (0 << CS12) | (1 << CS11) | (0 << CS10);  //Set capture falling edge, /8 prescaler
//
//    // USART init
//    UBRRL = UART_UBRR;
//    UBRRH = UART_UBRR >> 8;
//    UCSRB = (1 << RXEN) | (1 << TXEN) | (0 << U2X); // Enable RX and TX, disable 2X mode
//    UCSRC = (0 << USBS) | (1 << UCSZ1) | (1 << UCSZ0); // 8, N, 1
//
//    // Blink when boot done
//    LED_PORT |= 1 << LED_PIN;
//    _delay_ms(500);
//    LED_PORT &= ~(1 << LED_PIN);
//
//    sei();
//    for (;;) {
//
//
//        tmp_sense = !(SENSE_IN_PINS & (1 << SENSE_IN_PIN) >> SENSE_IN_PIN); //FIXME Debounce maybe?
//        if (tmp_sense != cassette_sense) {
//            cassette_sense = tmp_sense;
//            if (cassette_sense) {
//                //CLR_CASSETTE_SENSE;
//            } else {
//                //SET_CASSETTE_SENSE;
//                overflow = false;
//            }
//        }
//
//        // Send data in buffer
//        if (head >= 0) {
//            serial_send_msg_t(&msg_rcv_buf[head]);
//            head++;
//            if (head > tail) {
//                head = -1;
//                tail = -1;
//            }
//        }
//
//        if (overflow) {
//            //SET_CASSETTE_SENSE;
//            for (uint8_t blink = 0; blink < 5; blink++) {
//                LED_PORT |= 1 << LED_PIN;
//                _delay_ms(100);
//                LED_PORT &= ~(1 << LED_PIN);
//                _delay_ms(100);
//            }
//        } else {
//            LED_PORT &= ~(1 << LED_PIN);
//        }
//    }
//}
