#include <LUFA/Drivers/USB/USB.h>

#define VERSION         "1.04"

#define TICKS_PER_US 4

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00
#define CPU_8MHz        0x01
#define CPU_4MHz        0x02
#define CPU_2MHz        0x03
#define CPU_1MHz        0x04
#define CPU_500kHz      0x05
#define CPU_250kHz      0x06
#define CPU_125kHz      0x07
#define CPU_62kHz       0x08

//On-board LED
//Set the LED pins as outputs.
#define LED_CONFIG	(DDRD |= (3<<5))

//MINIMUS LEDs are active-low.
#define LED_BLUE_OFF (PORTD |= (1<<5))
#define LED_BLUE_ON (PORTD &= ~(1<<5))
#define LED_BLUE_TOGGLE  (PORTD ^= (1<<5))

#define LED_RED_OFF (PORTD |= (1<<6))
#define LED_RED_ON (PORTD &= ~(1<<6))
#define LED_RED_TOGGLE  (PORTD ^= (1<<6))

//Button
#define BUTTON_DOWN (~PINB & 2)
#define BUTTON_UP   (PINB & 2)

//Clock the D-Type flip-flop on PB4
#define ASSERT_CLOCK PORTB |= (1 << 4)
#define DENY_CLOCK   PORTB &= ~(1 << 4)

// EPROM addresses
#define EEPROM_BIT_DELAY_US 10
#define EEPROM_INPUT_MODE   13
#define EEPROM_OUTPUT_MODE  14
#define EEPROM_ECHO         15

void handleRandomEvent(void);
void setupRandom(void);
