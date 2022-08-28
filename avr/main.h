#ifndef TRUETAPE64_MAIN_H
#define TRUETAPE64_MAIN_H

#include <inttypes.h>

#define MSGQ_SIZE           3
#define MSGQ_REQ_THRESH     0
#define MAX_PULSE_LEN       ((2 ^ 24) - 1)
#define MIN_PULSE_LEN       5

#define UART_UBRR           3           // 250kbps at 16Mhz

#define LED_PIN             PB2
#define LED_PORT            PORTB
#define LED_PORT_DDR        DDRB

#define WRITE_PIN           PB3
#define WRITE_PINS          PINB
#define WRITE_PORT          PORTB
#define WRITE_PORT_DDR      DDRB

#define SENSE_IN_PIN        PB0
#define SENSE_IN_PINS       PINB
#define SENSE_IN_PORT       PORTB
#define SENSE_IN_PORT_DDR   DDRB
#define CASSETTE_STOPPED    (SENSE_IN_PINS & (1 << SENSE_IN_PIN) >> SENSE_IN_PIN)

//#define SENSE_OUT_PIN       PD3
//#define SENSE_OUT_PORT      PORTD
//#define SENSE_OUT_PORT_DDR  DDRD
//#define CLR_CASSETTE_SENSE  SENSE_OUT_PORT &= ~(1 << SENSE_OUT_PIN)
//#define SET_CASSETTE_SENSE  SENSE_OUT_PORT |=   1 << SENSE_OUT_PIN

#define CLR_CASSETTE_WRITE  WRITE_PORT &= ~(1 << WRITE_PIN)
#define SET_CASSETTE_WRITE  WRITE_PORT |= 1 << WRITE_PIN
#define GET_CASSETTE_WRITE  WRITE_PORT & (1 << WRITE_PIN)

#define SET_LED             LED_PORT |=   1 << LED_PIN
#define CLR_LED             LED_PORT &= ~(1 << LED_PIN)
#define TGL_LED             LED_PORT ^= 1 << LED_PIN




#define SM_IDLE             0
#define SM_BEGIN_WRITE      1
#define SM_WRITE            2
#define SM_END_WRITE        3
#define SM_BEGIN_READ       4
#define SM_READ             5
#define SM_END_READ         6
#define SM_INITIATE_RESET   8



#ifndef rbi
#define rbi(sfr, bit) (((sfr)>>(bit)) & 1)
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef fbi
#define fbi(sfr, bit) (_SFR_BYTE(sfr) ^= _BV(bit))
#endif

#endif //TRUETAPE64_MAIN_H
