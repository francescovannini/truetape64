#pragma ide diagnostic ignored "hicpp-signed-bitwise"
#pragma ide diagnostic ignored "EndlessLoop"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include <stdio.h>
#include "main.h"
#include "msgqueue.h"
#include "msg.h"

volatile flags_t error_flags;

/* Messages received from serial and messages ready to be sent through serial are placed in these two queues */
volatile msgqueue_t recv_q;
volatile msgqueue_t send_q;

/* Serial port routines use these two buffers when sending and receiving payload */
volatile uint8_t serial_rx_buf[sizeof(msg_t)];
volatile uint8_t serial_rx_buf_count;
volatile uint8_t serial_tx_buf[sizeof(msg_t)]; //TODO get rid of this
volatile int8_t serial_tx_buf_pos = -1;
volatile uint8_t awaiting_msgs = 0; //there is a request for these msgs but they haven't been received yet
volatile uint8_t cur_msg_data_pos = 0;

// Serial has received one byte
ISR(USART_RX_vect) {
    error_flags |= rbi(UCSRA, UPE)
            << FL_ERR_CHECKSUM; //TODO should we check the framing bit as well? it seems no as from datasheet "The FE bit is zero when the stop bit of received payload is one. Always set this bit to zero when writing to UCSRA."
    serial_rx_buf[serial_rx_buf_count] = UDR;
    serial_rx_buf_count++;

    if (serial_rx_buf_count == sizeof(msg_t)) {
        msgqueue_push(&recv_q, (msg_t *) &serial_rx_buf, &error_flags);
        serial_rx_buf_count = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (awaiting_msgs) {
                awaiting_msgs--;
            }
        }
    }
}

// Serial is ready to send the next byte
ISR(USART_UDRE_vect) {
    if (serial_tx_buf_pos >= 0) { // There is a transmission ongoing
        if (serial_tx_buf_pos < sizeof(msg_t)) { // Send next byte in TX buffer
            UDR = serial_tx_buf[serial_tx_buf_pos++];
        } else { // No more bytes in the TX buffer
            serial_tx_buf_pos = -1;
        }
    } else {
        if (msgqueue_count(&send_q) > 0) { // Pops next message from ring buffer, writes it into the TX buffer
            msgqueue_pop(&send_q, (msg_t *) &serial_tx_buf, &error_flags);
            //TODO check error_detected eod_mode on queue empty? it should not fail as count has just been checked, but...
            serial_tx_buf_pos = 0;
        } else { // Nothing else to send, disables this ISR
            UCSRB &= ~(1 << UDRIE);
        }
    }
}

volatile uint8_t timer0_counter = 0;

ISR (TIMER0_OVF_vect) {
    if (timer0_counter == 0) {
        if (error_flags) {
            TGL_LED;
        }
    }
    timer0_counter++;
    timer0_counter &= 0b00000111;
}

// Create new payload message to be sent to PC
volatile msg_t tmr_tmp_msg;

//void __attribute__ ((noinline)) send_data_msg(volatile const uint32_t *payload) {
//    tmr_tmp_msg.header.byte_value = 0;
//    tmr_tmp_msg.header.fields.cassette_sense = CASSETTE_STOPPED;
//    tmr_tmp_msg.payload = *payload;
//    msgqueue_push(&send_q, &tmr_tmp_msg, &error_flags);
//    UCSRB |= (1 << UDRIE); // Start serial transmission
//}

volatile uint32_t tm_counter; //TODO can this be smaller?

//TODO these two don't work anymore, need to rewrite them using new packet format
ISR(TIMER1_CAPT_vect) { // Falling edge
    tm_counter += ICR1;
    TCNT1 = 0;
    tm_counter = 0;
//    send_data_msg(&tm_counter);
}

ISR(TIMER1_OVF_vect) { // Timer overflow
    tm_counter += 0xFFFF;
    if (tm_counter > MAX_PULSE_LEN) {
        tm_counter = MAX_PULSE_LEN;
//        send_data_msg(&tm_counter);
        tm_counter = 0;
    }
}

volatile uint8_t eod_mode = 0;
volatile msg_t tmr_tmp_msg;

ISR(TIMER1_COMPA_vect) {
    if (rbi(WRITE_PINS, WRITE_PIN)) { // Level is high
        if (eod_mode == 0) {

            // Pop next msg from queue
            if (((tmr_tmp_msg.header.fields.ext_pulse_len == 0) && (cur_msg_data_pos == MSG_PLS8_LEN)) ||
                ((tmr_tmp_msg.header.fields.ext_pulse_len) && (cur_msg_data_pos == MSG_PLS16_LEN))) {
                cur_msg_data_pos = 0;
                msgqueue_pop(&recv_q, &tmr_tmp_msg, &error_flags); //TODO maybe check if this goes well?
                eod_mode |= tmr_tmp_msg.header.fields.eod;
            }

            if (tmr_tmp_msg.header.fields.ext_pulse_len) {
                if (tmr_tmp_msg.data.pulse16[cur_msg_data_pos] > 0) {
                    OCR1A = tmr_tmp_msg.data.pulse16[cur_msg_data_pos];
                }
            } else {
                if (tmr_tmp_msg.data.pulse8[cur_msg_data_pos] > 0) {
                    OCR1A = tmr_tmp_msg.data.pulse8[cur_msg_data_pos];
                }
            }

            cur_msg_data_pos++;

        } else { // eod_mode must be 1 here, so next comparison will complete the pulse of the last msg
            eod_mode++;
        }
    } else { // Level is low
        if (eod_mode == 2) { // last pulse is completed
            TCCR1A = 0;
            TCCR1B = 0;
            TIMSK &= ~(1 << OCIE1A);
        }
    }
}


int main(void) {
    msgqueue_init(&recv_q, 0, MSGQ_SIZE - 1);
    msgqueue_init(&send_q, MSGQ_SIZE - 1, 1);

    // Status led
    LED_PORT_DDR |= 1 << LED_PIN;

    // Write pin to Datassette normally up
    WRITE_PORT_DDR |= 1 << WRITE_PIN;
    WRITE_PORT |= 1 << WRITE_PIN;

    // Cassette cassette_sense input, Datassette F-6 pin to PB0
    SENSE_IN_PORT_DDR &= ~(1 << SENSE_IN_PIN);
    SENSE_IN_PORT |= 1 << SENSE_IN_PIN; // Pull-up

    // Capturing falling edges with Timer/Counter 1
    DDRD &= ~(1 << PD6); // Input on PD6

    // USART setup
    UBRRL = UART_UBRR;      // Set baud rate (low byte)
    UBRRH = UART_UBRR >> 8; // Set baud rate (high byte)

    UCSRA = (1 << U2X);     // Double speed

    UCSRB = (1 << RXCIE) |  // Fires USART_RX_vect when USART has received a new byte
            (0 << UDRIE) |  // Fires USART_UDRE_vect when USART is ready to send next byte
            (1 << RXEN) |   // Enable USART reception
            (1 << TXEN);    // Enable USART transmission

    UCSRC = (0 << UCSZ2) | (1 << UCSZ1) | (1 << UCSZ0) |    // 8 bit
            (1 << UPM1) | (0 << UPM0) |                     // Even parity
            (0 << USBS);                                    // 1 stop bit

    TIMSK = 0;

    // Timer0 setup
    TCCR0A = 0x00;
    TCCR0B = (1 << CS02) | (1 << CS00); // Clock / 1024
    TIMSK |= (1 << TOIE0); // Enable Timer0 interrupt
    sei();

    // Blink when boot done
    TGL_LED;
    _delay_ms(500);
    TGL_LED;
    _delay_ms(500);

    error_flags = 0;
    uint8_t sm = SM_IDLE;
    uint8_t tmp_byte;
    msg_t tmp_msg;

    for (;;) {
        switch (sm) {
            case SM_IDLE:
                tmp_byte = 0;
                msgqueue_pop(&recv_q, &tmp_msg, (flags_t *) &tmp_byte);
                if (tmp_byte > 0) {
                    _delay_us(1000);
                    break; // probably empty queue
                }

                if (tmp_msg.header.fields.set_mode) {
                    if ((tmp_msg.header.fields.write_tape) && (tmp_msg.header.fields.read_tape)) { // TEST MODE
                        tmp_msg.header.byte_value = 0;
                        msgqueue_push(&send_q, &tmp_msg, &error_flags);
                        UCSRB |= (1 << UDRIE); // Start serial transmission
                        break;
                    }

                    if (tmp_msg.header.fields.write_tape) {
                        sm = SM_BEGIN_WRITE;
                        break;
                    }

                    if (tmp_msg.header.fields.read_tape) {
                        sm = SM_BEGIN_READ;
                        break;
                    }
                }

                break;

            case SM_BEGIN_WRITE:
                // Wait until REC+PLAY is pressed
                while (CASSETTE_STOPPED) {
                    _delay_us(1000);
                }

                // Request payload to fill queue and wait until queue is full
                tmp_msg.header.byte_value = 0;
                tmp_msg.header.fields.cassette_sense = 1;
                tmp_msg.header.fields.error_detected = error_flags > 0;
                tmp_msg.data.bytes[0] = recv_q.size - msgqueue_count(&recv_q);
                msgqueue_push(&send_q, &tmp_msg, &error_flags);
                UCSRB |= (1 << UDRIE); // Start serial transmission

                // Wait for queue to be filled
                while (msgqueue_count(&recv_q) < recv_q.size) {
                    _delay_us(1000);
                }

                // Pop first message and feed pulse length to the timer
                msgqueue_pop(&recv_q, &tmr_tmp_msg, &error_flags);
                cur_msg_data_pos = 0;
                if (tmr_tmp_msg.header.fields.ext_pulse_len) { //FIXME should check if pulse len > 0
                    OCR1A = tmr_tmp_msg.data.pulse16[cur_msg_data_pos];
                } else {
                    OCR1A = tmr_tmp_msg.data.pulse8[cur_msg_data_pos];
                }
                cur_msg_data_pos++;

                // Setup TIMER1
                TCCR1B = (0 << CS12) |  //
                         (1 << CS11) |  // Prescaler CLOCK/8 = 2MHz
                         (0 << CS10) |  //
                         (1 << WGM13) | // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
                         (1 << WGM12);  // * Waveform Generator Mode 15 (table 45, page 106 of datasheet)    *
                TCCR1A = (1 << WGM11) | // * Fast PWM, TOP=OCR1A, Update OCR1A at TOP, TOV1 flag set at TOP  *
                         (1 << WGM10) | // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
                         (0 << COM1A1) |// Compare Output Mode: Clear OC1A on Compare Match when upcounting
                         (1 << COM1A0); //                      Set OC1A on Compare Match when downcounting

                TIMSK |= (1 << OCIE1A);
                WRITE_PORT &= ~(1 << WRITE_PIN);
                eod_mode = 0;
                sm = SM_WRITE;
                break;

            case SM_WRITE:
                do {
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        tmp_byte = recv_q.size - recv_q.count - awaiting_msgs;
                    }

                    if (tmp_byte >= MSGQ_REQ_THRESH) {
                        tmp_msg.header.byte_value = 0;
                        tmp_msg.header.fields.cassette_sense = CASSETTE_STOPPED;
                        if (error_flags > 0) {
                            tmp_msg.header.fields.error_detected = 1;
                            tmp_msg.data.bytes[0] = error_flags;
                        } else {
                            tmp_msg.data.bytes[0] = tmp_byte;
                            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                                awaiting_msgs += tmp_byte;
                            }
                        }
                        msgqueue_push(&send_q, &tmp_msg, &error_flags);
                        UCSRB |= (1 << UDRIE); // Start serial transmission
                    }

                } while (rbi(TIMSK, OCIE1A) && !CASSETTE_STOPPED);
                sm = SM_END_WRITE;
                break;

            case SM_END_WRITE:
                _delay_us(1000000); // Trailing low level (needed?)
                WRITE_PORT_DDR |= 1 << WRITE_PIN;
                WRITE_PORT |= 1 << WRITE_PIN;
                sm = SM_INITIATE_RESET;
                break;

            case SM_BEGIN_READ:
            case SM_READ:
            case SM_END_READ:
                sm = SM_INITIATE_RESET;
                break;

            case SM_INITIATE_RESET:
            default:
                sm = SM_IDLE;
                while(1) {
                    TGL_LED;
                    _delay_ms(1000);
                }
                break;
        }

        //TODO Do a payload transfer test. See how fast can we receive the payload; when the queue is full, just empty it and request more payload until it's full again. Count number of BPS we can send from python

//        tmp_sense = !(SENSE_IN_PINS & (1 << SENSE_IN_PIN) >> SENSE_IN_PIN); //FIXME Debounce maybe?
//        if (cassette_sense != tmp_sense) {
//            cassette_sense = tmp_sense;
//            if (cassette_sense) {
//                CLR_CASSETTE_SENSE;
//                TIMSK |= (1 << TOIE1) | // Fires TIMER1_OVF_vect when TIMER1 overflows
//                         (1 << ICIE1);  // Fires TIMER1_CAPT_vect when TIMER1 has completed a capture
//            } else {
//                TIMSK &= ~((1 << TOIE1) | // Fires TIMER1_OVF_vect when TIMER1 overflows
//                           (1 << ICIE1));  // Fires TIMER1_CAPT_vect when TIMER1 has completed a capture
//                SET_CASSETTE_SENSE;
//                error_flags = false;
//            }
//        }

    }

}



































//int main(void) {
//
//    // Status led
//    LED_PORT_DDR |= 1 << LED_PIN;
//
//    // USART set_mode
//    UBRRL = UART_UBRR;
//    UBRRH = UART_UBRR >> 8;
//    UCSRB = (1 << RXEN) | (1 << TXEN) | (0 << U2X); // Enable RX and TX, disable 2X set_mode
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
//    // Cassette cassette_sense input, Datassette F-6 pin to PB1
//    SENSE_IN_PORT_DDR &= ~(1 << SENSE_IN_PIN);
//    SENSE_IN_PORT |= 1 << SENSE_IN_PIN; // Pull-up
//
//    // Capturing falling edges with Timer/Counter 1
//    DDRD &= ~(1 << PD6); // Input on PD6
//    TIMSK |= (1 << ICIE1) |
//             (1 << TOIE1);     //Set capture interrupt and overflow interrupt //FIXME this only is read_tape set_mode
//    TCCR1B = (0 << ICNC1) | (0 << ICES1)
//             | (0 << CS12) | (1 << CS11) | (0 << CS10);  //Set capture falling edge, /8 prescaler
//
//    // USART set_mode
//    UBRRL = UART_UBRR;
//    UBRRH = UART_UBRR >> 8;
//    UCSRB = (1 << RXEN) | (1 << TXEN) | (0 << U2X); // Enable RX and TX, disable 2X set_mode
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
//        // Send payload in buffer
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
