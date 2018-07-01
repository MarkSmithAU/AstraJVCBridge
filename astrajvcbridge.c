/* 
(c) Mark Smith 2018
GPL v3
Not licensed for commercial use
*/

/*
Holden Astra to JVC Radio steering wheel remote code for ATTINY85.

One resistor (470 ohm) for the voltage divider and one capacitor are the only external components required.
The whole circuit can be powered from the 5v antenna line on the back of the JVC which has power when the unit is on.
The internal 8Mhz oscillator is fast and accurate enough for our purposes.

PORTB0 is the line to connect to the JVC remote control input.
PORTB4 is the analogue input via the 470 resistor and car i.e. 5v - 470 - car remote wire in radio harness (Blue/Red)

The processing steps are:
1. Read ADC values
2. Translate ADC values using  tolerances to a command
3. Debounce the command for 10ms
4. Process the debounced command in a state machine in main() to allow sequential of single codes

See https://www.avforums.com/threads/jvc-stalk-adapter-diy.248455/ for the raw protocol details.

Astra raw resistance values and mappings with 5v source and 470 ohm resistor:
Button	Resistance	JVC Mapping	JVC Code	ADC Value
None	3652		Idle		None		907
up		1466		Source		0x08		775
back	790			Back		0x13		642
fwd		466			Forwards	0x12		510
O/0		283			Mute		0x06		385
-		190			VolDn		0x05		295
+		163			VolUp		0x04		264

A tolerance of 15 ADC steps is the largest we can use due to the -/+ voltages being so close.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "bitmacros.h"
#include "bitNames.h"
#include "debounce.h"

#define true (!0)
#define false (0)

// Voltage values, assuming vref = 5v
#define CAR_IDLE	907
#define CAR_MUTE	385
#define CAR_VOLDN	295
#define CAR_VOLUP	264
#define CAR_UPARR	775
#define CAR_BKARR	642
#define CAR_FDARR	510

// JVC Commands
#define JVC_VOLUP	0x04
#define JVC_VOLDN	0x05
#define	JVC_MUTE	0x06
#define JVC_SRC		0x08
#define JVC_SKIPBK	0x11
#define JVC_SKIPFD	0x12
#define JVC_SKIPBKH 0x13
#define JVC_SKIPFDH 0x14

// States
enum {VAL_IDLE, VAL_VOLUP, VAL_VOLDN, VAL_SRC, VAL_SEEKFWD, VAL_SEEKBK, VAL_MUTE};

// Calcs for ADC thresholds for handling values
// vref = 5v
// 10 bits = 1024
// 1 bit = ~0.005V
// +-0.2V = 30
#define TOLLERANCE	15

void JVCCommand(unsigned char cmd);
void ADCInit();
uint16_t ADCRead();
uint8_t DecodeAnalogue(uint16_t adcVal);

// time tick variables used for interval timer
static volatile unsigned long uptime = 0L;
static volatile unsigned long target;

// time tick variables used for 1ms interval timer
volatile unsigned char tick = 0;
debounceData cDebounce;
volatile unsigned char cCombined = 0, cCombinedLast = 0;
uint8_t decodedValue = VAL_IDLE;

void waitForTick(uint16_t count)
{
	for (; count > 0; count--) {
		while (tick == 0) /* wait for ISR */;
		tick = 0;
	}
}

// 527us
ISR(TIMER1_COMPA_vect)
{
	tick = 1;
	
	cCombined = getDebounced(&cDebounce, decodedValue);
}

void interruptSleep(unsigned long ms)
{
	unsigned long target;
	target = uptime + ms;
	while (uptime <= target);
}

int main(void)
{
	uint16_t adcVal;

	
	/* Define pull-ups and set outputs high */
	/* Define directions for port pins */
	DDRB =  0b00000000;
	PORTB = 0b00000000; //1 for pullup
	
	// 10ms debounce, idle value is high, one shot is disabled (keep reporting the triggered value repeatedly)
	initDebounce(&cDebounce, 10, VAL_IDLE, 0);

	// Set up Timer1 for 527us interrupts
	OCR1A = (F_CPU / 4 / (128UL * (527UL - 67UL))) - 1;
	OCR1C = (F_CPU / 4 / (128UL * (527UL - 67UL))) - 1;
	TCCR1 = _BV(CS13) | _BV(CTC1);
	_setBit(TIMSK, OCIE1A);
	
	ADCInit();

	sei();	// Enable global interrupts

	while(1) {
		/* 
		This will be executed every 527us, unless waitForTick clears it during a send sequence
		*/
		if (tick == 1) {
			
			adcVal = ADCRead();
			decodedValue = DecodeAnalogue(adcVal);
			
			// see ISR for this as well
			cli();
			cCombined = getDebounced(&cDebounce, decodedValue);
			sei();

			//VAL_SRC, 
			switch (cCombined) {
				case VAL_SEEKFWD:
					/* Seek: This could have been held in which case a different code is required */
					if (cCombined != cCombinedLast) {
						/* first time */
						JVCCommand(JVC_SKIPFD);
					} else {
						/* held */
						JVCCommand(JVC_SKIPFDH);
					}
					break;
					
				case VAL_SEEKBK:
					/* Seek: This could have been held in which case a different code is required */
					if (cCombined != cCombinedLast) {
						/* first time */
						JVCCommand(JVC_SKIPBK);
						} else {
						/* held */
						JVCCommand(JVC_SKIPBKH);
					}
					break;

				case VAL_VOLUP:
					JVCCommand(JVC_VOLUP);
					waitForTick(400UL);
					break;
					
				case VAL_VOLDN:
					JVCCommand(JVC_VOLDN);
					waitForTick(400UL);
					break;
					
				case VAL_SRC:
					/* Only send a code once per button press */
					if (cCombined != cCombinedLast) {
						/* first time */
						JVCCommand(JVC_SRC);
					}
					break;
					
				case VAL_MUTE:
					/* Only send a code once per button press */
					if (cCombined != cCombinedLast) {
						/* first time */
						JVCCommand(JVC_MUTE);
					}
					break;
					
				case VAL_IDLE:
					/* Idle - do nothing */
					break;
			}
			cCombinedLast = cCombined;
			tick = 0;
		}
	}

	return 0;
}


void JVCPulseLengthEncoding(unsigned char val) {
	_movNamedBitNoPullUp(JVC, 0);
	waitForTick(1);
	_movNamedBitNoPullUp(JVC, 1);
	waitForTick(1);
	if (val != 0) {
		waitForTick(2);
	}
}

void JVC7BitByte(unsigned char cmd) {
	JVCPulseLengthEncoding(cmd & 0b00000001);
	JVCPulseLengthEncoding(cmd & 0b00000010);
	JVCPulseLengthEncoding(cmd & 0b00000100);
	JVCPulseLengthEncoding(cmd & 0b00001000);
	JVCPulseLengthEncoding(cmd & 0b00010000);
	JVCPulseLengthEncoding(cmd & 0b00100000);
	JVCPulseLengthEncoding(cmd & 0b01000000);
}

void JVCCommand(unsigned char cmd) {
	for (int i = 1; i <= 3; i++) {
		// Header
		_movNamedBitNoPullUp(JVC, 1);		// Bus reset
		waitForTick(1);
		
		_movNamedBitNoPullUp(JVC, 0);     // AGC
		waitForTick(16);
		
		_movNamedBitNoPullUp(JVC, 1);     // AGC
		waitForTick(8);
		
		JVCPulseLengthEncoding(1);    // 1 Start Bit
		
		JVC7BitByte(0x47);

		//Body
		JVC7BitByte(cmd);
		
		//Footer
		JVCPulseLengthEncoding(1);
		JVCPulseLengthEncoding(1);    // 2 stop bits
	}
}

void ADCInit()
{
  /* this function initialises the ADC 

        ADC Notes
	
	Prescaler
	
	ADC Prescaler needs to be set so that the ADC input frequency is between 50 - 200kHz.
	
	Example prescaler values for various frequencies
	
	Clock   Available prescaler values
   ---------------------------------------
	 1 MHz   8 (125kHz), 16 (62.5kHz)
	 4 MHz   32 (125kHz), 64 (62.5kHz)
	 8 MHz   64 (125kHz), 128 (62.5kHz)
	16 MHz   128 (125kHz)

   below example set prescaler to 128 for mcu running at 8MHz


  */

  ADMUX =
            (0 << ADLAR) |     // right shift result
            (0 << REFS1) |     // Sets ref. voltage to VCC, bit 1
            (0 << REFS0) |     // Sets ref. voltage to VCC, bit 0
            (0 << MUX3)  |     // use ADC2 for input (PB4), MUX bit 3
            (0 << MUX2)  |     // use ADC2 for input (PB4), MUX bit 2
            (1 << MUX1)  |     // use ADC2 for input (PB4), MUX bit 1
            (0 << MUX0);       // use ADC2 for input (PB4), MUX bit 0

  ADCSRA = 
            (1 << ADEN)  |     // Enable ADC 
            (1 << ADPS2) |     // set prescaler to 128, bit 2 
            (1 << ADPS1) |     // set prescaler to 128, bit 1 
            (1 << ADPS0);      // set prescaler to 128, bit 0  
}

uint16_t ADCRead()
{
	uint16_t val;
	ADCSRA |= (1 << ADSC);         // start ADC measurement
	while (ADCSRA & (1 << ADSC) ); // wait till conversion complete

	val = (uint16_t)ADCL;
	val |= ((uint16_t)ADCH) << 8;
	
	return val;
}


uint8_t inRange(uint16_t adcVal, uint16_t value, uint16_t tollerance)
{
	volatile uint16_t lower, upper;
	volatile int16_t chk;
	chk = (signed)value - (signed)tollerance;
	
	if (chk <= 0) {
		lower = 0;
	} else {
		lower = value - tollerance;
	}
	
	upper = value + tollerance;
	
	if (adcVal >= lower  && adcVal <= upper)
		return true;
	else
		return false;
}

uint8_t DecodeAnalogue(uint16_t adcVal)
{
	if (inRange(adcVal, CAR_VOLUP, TOLLERANCE)) return VAL_VOLUP;
	if (inRange(adcVal, CAR_VOLDN, TOLLERANCE)) return VAL_VOLDN;
	if (inRange(adcVal, CAR_UPARR, TOLLERANCE)) return VAL_SRC;
	if (inRange(adcVal, CAR_FDARR, TOLLERANCE)) return VAL_SEEKFWD;
	if (inRange(adcVal, CAR_BKARR, TOLLERANCE)) return VAL_SEEKBK;
	if (inRange(adcVal, CAR_MUTE, TOLLERANCE)) return VAL_MUTE;
	return VAL_IDLE;	
}