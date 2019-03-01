/*
 * ToF-CAN-Firmware.cpp
 *
 * Created: 2/28/2019 7:40:37 PM
 * Author : erics
 */ 


// TODO: do we need to switch to an external oscillator?
/*
 * Fuses Set to:
 * Low:  0xE2
 * High: 0XD1
 * Ext:  0xF7
 */

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

#define TWBR TWBR0
#define TWCR TWCR0
#define TWSR TWSR0
#define TWDR TWDR0

#include "libs/I2C-master-lib/i2c_master.h"
#include "libs/I2C-master-lib/i2c_master.c"
#include "libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h"
#include "libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.cpp" // <-- sue me. this actually decreases memory ussage, cause we don't get the whole staic library

#define PIN_LED_B DDD6
#define PIN_LED_G DDD5
#define PIN_LED_R DDD7

void init_uart(uint16_t baudrate) {

	uint16_t UBRR_val = (F_CPU/16)/(baudrate-1);

	UBRR0H = UBRR_val >> 8;
	UBRR0L = UBRR_val;

	UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); // UART TX (Transmit) 
	UCSR0C |= (1<<USBS0) | (3<<UCSZ00); //Modus Asynchronpus 8N1 (8 Data bits, No Parity, 1 Stop bit)
}

void uart_putc(const unsigned char c) {

	while(!(UCSR0A & (1<<UDRE0))); // wait until sending is possible
	UDR0 = c; // output character saved in c
}

void uart_puts(const char *s) {
	while(*s){
		uart_putc(*s);
		s++;
	}
}

void writeVL53L0X(uint8_t rgstr, uint8_t val) {
	
}

Adafruit_VL53L0X tof = Adafruit_VL53L0X();

char buffer[16];
uint16_t count = 0;
int main(void)
{
	DDRD |= (1<<PIN_LED_B);
	DDRD |= (1<<PIN_LED_G);
	DDRD |= (1<<PIN_LED_R);
	
	init_uart(9600);
	i2c_init();
	
	tof.begin();
   
   VL53L0X_RangingMeasurementData_t measure;
    while (1) 
    {
		
		tof.rangingTest(&measure, false);
		if (measure.RangeStatus != 4) {
			uart_puts("Distance (mm): ");
			itoa(measure.RangeMilliMeter, buffer, 10);
			uart_puts(buffer);
			uart_puts("\r\n");
		} else {
			// OUT OF RANGE
		}
		
	    PORTD |= (1<<PIN_LED_B);
	    _delay_ms(1);
	    PORTD &= ~(1<<PIN_LED_B);
	    _delay_ms(99);
	    count++;
    }
}

