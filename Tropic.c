/*
 * Random.c
 *
 * Created: 18/01/2015 14:13:04
 *  Author: jackc
 *
 * Handle inputs on an interrupt. 
 * Inputs are currently two free-running ring oscillators. Other inputs could be added,
 * e.g. a metastable flipflop, a radio receiver.
 */ 


#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/interrupt.h>

#include "VirtualSerial.h"
#include "Tropic.h"



#include <util/delay.h>

#include <LUFA/Drivers/USB/Class/Device/CDCClassDevice.h>


#ifdef __AVR_ATmega32U2__
#endif

//Stuff that can be stored in the EEPROM
uint16_t bit_delay_us = 300;
uint8_t cpu_mhz;
uint8_t output_mode;
uint8_t input_mode;
uint8_t echo;

int32_t requested_bytes = 0;
uint8_t byte_count = 0;
uint16_t missed = 0;     //number of times PRB was called when no bit was available.
uint16_t interrupts = 0;
uint16_t ticks_idle = 0;
uint16_t ticks_work = 0;
uint8_t flags = 0;

//Bytes shared between ISRs and main program
volatile uint8_t inbits;
volatile uint8_t adch;
volatile uint8_t osc_status = 0;    //bits 0 and/or 1 are set if OSC0 or OSC1 is dead.

/*
 * On an interrupt, shift two bits from the two oscillators into 
 * inbits, and set bits_ready.
 * This is the entropy source!
 */
ISR(TIMER1_COMPA_vect) {
	ASSERT_CLOCK;
    
	//Input in PB3
	inbits = ((PINB & (1 << 3)) >> 3) | 128;
	LED_RED_TOGGLE;
    interrupts++;

	DENY_CLOCK;	
}

/*
 * On interrupt, read the ADC to get the mean voltage on an oscillator tap.
 * Set LS 2 bits of osc_status 
 * Then switch channels.
 * If the channel is stuck, set a status reading. 'Stuck' means the tap is
 * not within a few percent of 0.5 * Vcc.
 * NOTE: if the oscillator is working, then all the gates must be connected, and
 * it doesn't matter where in the ring we take our tap.
 * So we say the oscillator is malfunctioning, IF the 'mean' voltage at the tap is not:
 * 2.5V, +/- 0.55V, == 128 +/- 28
 * 
 */
#ifdef __AVR_AT90USB__
ISR(ADC_vect) {
    adch = ADCH;
    if ((ADCH < 100) || (ADCH > 156)) {
        osc_status |= (1 << (ADMUX & 1)); 
    } else {
        osc_status &= ~(1 << (ADMUX & 1)); 
    }
    ADMUX ^= 1;
}
#endif

/*
 * Flashers for the built-in LED
 */
void flash_blue(int count) {
	for (int i = 0; i < count; i++) {
		LED_BLUE_ON;
		_delay_ms(100);
		LED_BLUE_OFF;
		_delay_ms(100);
	}
}

void flash_red(int count) {
	for (int i = 0; i < count; i++) {
		LED_RED_ON;
		_delay_ms(100);
		LED_RED_OFF;
		_delay_ms(100);
	}
}

/*
Wrapper for _delay_us that uses a variable for the delay time.
Assume we have a delay of at least 100 us.
*/ 
void delay_us(uint16_t us) {
	while ((us-=50) > 0) {
		_delay_us(50);
	}	
}

/*
 * Set up timer 1
 */
void setTimer1ISR(void) {
    cli();
    OCR1A = (TICKS_PER_US * bit_delay_us);
    TCCR1A = 0;        // set entire TCCR1A register to 0
    TCCR1B = 0;
        
    // turn on CTC (Clear Timer on Compare match) mode
    // Counter is zeroed when TCNT1 == OCR1A, and interrupt fires
    TCCR1B |= (1 << WGM12); 
    TCCR1B |= (1 << CS10); // turn on clock
           
    TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
    sei();
}

#ifdef __AVR_AT90USB__
// Set up ADC: We use ADC0.
// We will also use ADC1 in due course.
void setAdcISR() {
    cli();
    
    //Select channel 0 single-ended (i.e. zero least-significant 4 bits)
    ADMUX &= ~(0b1111);

    //Set reference voltage to AVCC
    ADMUX |= (1 << REFS0);

    // Set prescaler to 128, sample interval == 31KHz
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    //Left-align ADC: most-significant 8 bits in ADCH.
    //Makes it possible to just read ADCH to get an 8-bit reading.
    ADMUX |= (1 << ADLAR);

    // ADHSM = 0: disable high-speed mode. ADTS2:0: Auto-trigger source = free-running.
    // Other bits are unused
    ADCSRB = 0;

    // Enable the ADC (seems like this has to happen first!)
    ADCSRA |= (1 << ADEN);

    //Auto-trigger Enable
    ADCSRA |= (1 << ADATE);

    //Enable ADC interrupts
    ADCSRA |= (1 << ADIE);

    //Start ADC measurements
    ADCSRA |= (1 << ADSC);

    sei();
}
#endif


#define stopTimerISR() TIMSK1 &= ~(1 << OCIE1A)    // Disable timer compare interrupt    
#define startTimerISR() TIMSK1 |= (1 << OCIE1A)    // Enable timer compare interrupt


void setupRandom(void) {

    //Enable output on PORTD5 and PORTD6 (Minimus LEDs)
	LED_CONFIG;

	//Switch on Red LED
    //LED_RED_ON;
    
	//Set pin B4 as output (to flip-flop CLK2). Other pins are input.
	//PORT B all input (0) except B4. B3 is input from flipflop. B1 is input from button.
	DDRB = 0b00010000;

	//pullup on pin B1 (input)
	PORTB = 0b00000010;

    //Asserting PINB1 forces sample interval, MHz, input mode, output mode and
    //echo to sane values. Otherwise EEPROM settings are used.
    if (BUTTON_DOWN) {
        bit_delay_us = 250;
        input_mode = 'b';
        output_mode = 'h';
        echo = 1;
    } else {
	    bit_delay_us = eeprom_read_word((uint16_t*) EEPROM_BIT_DELAY_US);
	    input_mode = eeprom_read_byte((uint8_t*) EEPROM_INPUT_MODE);
	    output_mode = eeprom_read_byte((uint8_t*) EEPROM_OUTPUT_MODE);
	    echo = eeprom_read_byte((uint8_t*) EEPROM_ECHO);
    }

	CPU_PRESCALE(CPU_8MHz);

    //Set the interrupt handlers
    setTimer1ISR();   
    
	//Wait for host driver to come ready
	LED_RED_OFF;
	LED_BLUE_OFF;

	flash_blue(5);

}


/*
 * Go to sleep until interrupted.
 */
void idle(void) {
    static uint16_t ticks_work_start = 0;
    ticks_work = TCNT1 - ticks_work_start;
    uint16_t ticks_idle_start = TCNT1;
	set_sleep_mode(SLEEP_MODE_IDLE);
	cli();
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
    ticks_idle = TCNT1 - ticks_idle_start;
    ticks_work_start = TCNT1;
}


/*
The possible sample modes are:
- Just PINB1
- Just PIND3
- PINB1 XOR PIND3
- PINB1 VN PIND3
*/



/*
 * Print 8 binary digits to USB Serial
 */

void print_binary(uint8_t b) {
	for (uint8_t i = 0; i < 8; i++) {
		if (b & 128) {
			fputc('1', &USBSerialStream);
		} else {
			fputc('0', &USBSerialStream);
		}
		b <<= 1;
	}
}


/*
 * Output a byte to USB Serial according to output_mode.
 */
void output_byte(uint8_t b) {

	switch(output_mode) {
		case 'b':
			print_binary(b);
			break;

		case 'h':
			fprintf(&USBSerialStream, "%02x", b);
			//Print a newline every 16 bytes, and a space between bytes
			if (++byte_count == 16) {
				byte_count = 0;
				fprintf(&USBSerialStream, "\r\n");
			} else {
				fprintf(&USBSerialStream, " ");
			}
			LED_RED_TOGGLE;
			break;
			
		case 'r':
		case 'c':
			fputc(b, &USBSerialStream);
			break;

		default:
			flash_blue(5);
	}
}

/*
Read decimal digits from num up to a NUL.
*/
int32_t parse_num(uint8_t* num) {
	uint8_t idigit = 0;
	uint8_t digit_char;
	int32_t result = 0;

	while ((digit_char = num[idigit++]) != 0) {
		if (digit_char < 0x30 || digit_char > 0x39) {
			return -1;
		}
		result *= 10;
		result += (digit_char - 0x30);
	}
	return result;
}

void printStatus(void) {
    uint8_t prom_input_mode = eeprom_read_byte((uint8_t*) EEPROM_INPUT_MODE);
    uint8_t prom_output_mode = eeprom_read_byte((uint8_t*) EEPROM_OUTPUT_MODE);
    uint16_t prom_bit_delay_us = eeprom_read_word((uint16_t*) EEPROM_BIT_DELAY_US);
    uint8_t prom_echo = eeprom_read_byte((uint8_t*) EEPROM_ECHO);

    fprintf(&USBSerialStream, "Version: %s\r\n", VERSION);
    
    fprintf(&USBSerialStream, "Input mode: %c (PROM: %c)\r\n", input_mode, prom_input_mode);
    fprintf(&USBSerialStream, "Output mode: %c (PROM: %c)\r\n", output_mode, prom_output_mode);
    fprintf(&USBSerialStream, "Bit delay: %d (PROM: %d)\r\n", bit_delay_us, prom_bit_delay_us);
    fprintf(&USBSerialStream, "Echo: %d (PROM: %d)\r\n", echo, prom_echo);
    fputs("\r\n", &USBSerialStream);
    
    fprintf(&USBSerialStream, "Requested: %li\r\n", requested_bytes);
    fputs("OSC_STATUS: ", &USBSerialStream);
    print_binary(osc_status);
    fputs("\r\n", &USBSerialStream);

    fputs("TCCR1A: ", &USBSerialStream);
    print_binary(TCCR1A);
    fputs("\r\n", &USBSerialStream);

    fputs("TCCR1B: ", &USBSerialStream);
    print_binary(TCCR1B);
    fputs("\r\n", &USBSerialStream);

    fputs("TIMSK1: ", &USBSerialStream);
    print_binary(TIMSK1);
    fputs("\r\n", &USBSerialStream);

    fputs("OCR1A: ", &USBSerialStream);
    print_binary(OCR1A >> 8);
    print_binary(OCR1A & 0xFF);
    fprintf(&USBSerialStream, " (%d)\r\n", OCR1A);

    fprintf(&USBSerialStream, "Interrupts: %u\r\n", interrupts);

    fprintf(&USBSerialStream, "Missed: %u\r\n", missed);
    missed = 0;

    fprintf(&USBSerialStream, "Last idle: %u\r\n", ticks_idle);

    fprintf(&USBSerialStream, "Last work: %u\r\n", ticks_work);

	fprintf(&USBSerialStream, "PINB: %0x\r\n", PINB & ~(128 + 8));

	fputs("\r\n", &USBSerialStream);
}

/*
 * Parse a command.
 * Commands that request data reset any command in progress, and set global variables;
 * other commands act immediately.
 * Commands:
 * i<b/d/x/v/h>     Set input mode
 * o<b/h/r>         Set output mode
 * d<n>             Set bit-delay to n microseconds.
 * r<n>             Request bytes
 * s                Output status
 */
void handleCommand(uint8_t* command) {
    int32_t num;

	switch(command[0]) {
        case 'i':
            switch(command[1]) {
                case 'b':   //PINB1
                case 'd':   //PIND3
                case 'a':   //ADC0
                case 'x':   //XOR the pins
                case 'v':   //VN the pins
                case '2':   //Output both bits
                    input_mode = command[1];
                    fputs("OK\r\n", &USBSerialStream);
                    break;
                default:
                    fputs("Invalid command (i[b|d|a|x|v|h|2])\r\n", &USBSerialStream);
            }
            break;

        case 'o':
            switch (command[1]) {
                case 'b':
                case 'h':
                case 'r':
                case 'c':
                    output_mode = command[1];
                    fputs("OK\r\n", &USBSerialStream);
                    break;
                default:
                    fputs("Invalid command (o[b|h|r|c])\r\n", &USBSerialStream);
            }
    	    break;
    	
    	case 'd':
        	num = parse_num(&command[1]);
            // Don't set a bit-delay smaller than 5 microseconds
            if (num >= 5) {
                bit_delay_us = num;
                fputs("OK\r\n", &USBSerialStream);
            } else {
                fputs("Invalid decimal number (d<num>)\r\n", &USBSerialStream);
            }            
            break;
            
        case 'p':
           	eeprom_write_word((uint16_t*)EEPROM_BIT_DELAY_US, bit_delay_us);
           	eeprom_write_byte((uint8_t*)EEPROM_OUTPUT_MODE, output_mode);
           	eeprom_write_byte((uint8_t*)EEPROM_INPUT_MODE, input_mode);
           	eeprom_write_byte((uint8_t*)EEPROM_ECHO, echo);
            fputs("OK\r\n", &USBSerialStream);
            break;        

    	case 'm':
    	    num = parse_num(&command[1]);
    	    if (num != -1) {
        	    cpu_mhz = num;
                fputs("OK\r\n", &USBSerialStream);
    	    } else {
                fputs("Invalid decimal number (d<num>)\r\n", &USBSerialStream);
            }
    	    break;

    	case 'r':
        	num = parse_num(&command[1]);
            if (num != -1) {
                byte_count = 0;
                requested_bytes = num;
            } else {
                fputs("Invalid command (r<num>)\r\n", &USBSerialStream);
            }
        	break;
        
        case 's':
            printStatus();
            break;
            
        case 'e':
            echo = !echo;
            break;

        case 'q':
            requested_bytes = 0;
            fputs("OK\r\n", &USBSerialStream);
            break;

    	default:
            fputs("Invalid command (i|o|p|r|s|q)\r\n", &USBSerialStream);
	}
    command[0] = 0;
}


/*
 * Read byte from USB Serial. On CR, handle the completed command.
 * For use with the eventLoop.
 */
void checkForCommand(void) {
	int16_t b;
	static uint8_t len;
	static uint8_t command[10];

	b = fgetc(&USBSerialStream);
	if (b > 0) {
		LED_BLUE_TOGGLE;
		b &= 0xFF;
        if (echo) {
        	fputc(b, &USBSerialStream);
        	if (b == 0x0D) {
        		fputc(0x0A, &USBSerialStream);
        	}
        }        
		if (len > 8) {
			len = 0;
		}
		if (b == 0x0D || b == 0x0A) {
			handleCommand(command);
            len = 0;
		} else {
		    command[len++] = (b & 0xFF);
		    command[len] = 0;
        }        
	}
}

uint8_t VNMAP[] = {0, 2, 3, 0};
void processRawBits(void) {
    uint8_t b = 0;
    uint8_t process_bits = 0;
    static uint8_t randbyte = 0;
    static uint8_t randbits = 0;
  
    if (!(requested_bytes > 0 || output_mode == 'c')) {
        return;
    }
    
    cli();
    process_bits = (inbits & 128);
    b = inbits & 1;
    sei();
    
    if (process_bits != 0) {
    	LED_BLUE_TOGGLE;
        switch(input_mode) {
            //Only PINB3
            case 'b':
                randbyte <<= 1;
                randbyte |= (b & 1);
                randbits++;
                break;
                
            //Only PIND8
            case 'd':
                randbyte <<= 1;
                randbyte |= ((b >> 1) & 1);
                randbits++;
                break;

            case 'x':
                //XOR PINB3 and PIND8
                randbyte <<= 1;
                randbyte |= ((b >> 1) & 1) ^ (b & 1);
                randbits++;
                break;
                
            case 'v':
                //VN PINB3 and PIND8
                randbyte <<= 1;
        		uint8_t mapped = VNMAP[(b & 3)];
		        if (mapped & 2) {
    		        randbyte |= (mapped & 1);
                    randbits++;
                }
                break;
            
            case '2':
                //Output both bits
                randbyte <<= 2;
                randbyte |= (b & 3);
                randbits += 2;
                break;
                
            case 'a':
                //ADC
                randbyte <<= 1;
                randbyte |= (b & 4);
                randbits += 1;
                break;
                
            default: ;
        }
        
		if (randbits == 8) {
			randbits = 0;
			if (requested_bytes > 0 || output_mode == 'c') {
				//LED_RED_TOGGLE;
				output_byte(randbyte);
				requested_bytes--;
			} else {
				//Fulfilled request, and not continuous output mode
				stopTimerISR();
			}
		}
    } else {
        missed++;
    }
}

/*
 * Produce no bits while status is dodgy, and light the LED.
 */
uint8_t checkOscillatorStatus(void) {
    if (osc_status & 3) {
        //LED_ON;
        _delay_ms(10);
        return 1;
    }
    //LED_OFF;
    return 0;
}    

/*
 * We handle two events:
 * - random byte ready
 * - input byte ready
 */
void handleRandomEvent(void) {
	processRawBits();
	checkForCommand();
	idle();
}


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	setupRandom();

	SetupHardware();

	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY); //LED_RED
	GlobalInterruptEnable();

	for (;;)
	{
		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
		//CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		handleRandomEvent();

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}
