# Holden Astra to JVC Radio steering wheel remote code for ATTINY85

```
This code can be used on any vehicle that uses a 5v (or lower) output from the vehicle or a resistor divider bridge
to supply the steering wheel controls through to the radio.  
The ATTINY85 ADC pin must not receive a voltage above VCC (5v) i.e. 12V would be bad!  Use a high impedance resistor 
divider to get the correct voltage if required.

One resistor (470 ohm) for the voltage divider and one capacitor are the only external components required for the Astra.
The whole circuit can be powered from the 5v amplifier line on the back of the JVC which has power when the unit is on.

I/O and connection notes:
PORTB0 is the line to connect to the JVC remote control input.
PORTB4 is the analogue input via the 470 resistor and car
	i.e. 5v -> 470 ohm resistor -> to PORTB4 AND car remote wire in radio harness (Blue/Red)
RESET: short to VCC
VCC: 10uF Capacitor across VCC/GND, power with a 7805 regulator from the Amplifier control wire in the JVC connector.
GND: connect to GND in radio wiring harness

The internal 8Mhz oscillator is fast and accurate enough for our purposes.
Fuses for programming the ATTINY85:
SELFPRGEN = [ ]
RSTDISBL = [ ]
DWEN = [ ]
SPIEN = [X]
WDTON = [ ]
EESAVE = [ ]
BODLEVEL = 4V3
CKDIV8 = [ ]
CKOUT = [ ]
SUT_CKSEL = INTRCOSC_8MHZ_6CK_14CK_64MS
EXTENDED = 0xFF (valid)
HIGH = 0xDC (valid)
LOW = 0xE2 (valid)

The processing steps are:
1. Read ADC values
2. Translate ADC values using tolerances to a command
3. Debounce the command for 5ms (done in the ISR to ensure consistent timing)
4. Process the debounced command in a state machine in main() to allow sequenced codes and delays as required

See https://www.avforums.com/threads/jvc-stalk-adapter-diy.248455/ for the raw protocol details.

Astra raw resistance values and mappings with 5v source and 458 ohm (measured) resistor:
Button	Resistance	JVC Mapping	JVC Code	ADC Value
None	3652		Idle		None		910
up		1466		Source		0x08		780
back	790			Back		0x13		648
fwd		466			Forwards	0x12		516
O/0		283			Sound		0x06		391
-		83			VolDn		0x05		157
+		163			VolUp		0x04		269
```
