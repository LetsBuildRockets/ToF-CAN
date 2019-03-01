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

#define TWBR TWBR0
#define TWCR TWCR0
#define TWSR TWSR0
#define TWDR TWDR0

#include "libs/I2C-master-lib/i2c_master.h"
#include "libs/I2C-master-lib/i2c_master.c"
#include "libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h"

#define PIN_LED_B DDD5
#define PIN_LED_G DDD6
#define PIN_LED_R DDD7

#define VL53L0X_WRITE 0x52
#define VL53L0X_READ 0x53


void init_uart(uint16_t baudrate) {

	uint16_t UBRR_val = (F_CPU/16)/(baudrate-1);

	UBRR0H = UBRR_val >> 8;
	UBRR0L = UBRR_val;

	UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); // UART TX (Transmit - senden) einschalten
	UCSR0C |= (1<<USBS0) | (3<<UCSZ00); //Modus Asynchron 8N1 (8 Datenbits, No Parity, 1 Stopbit)
}

void uart_putc(unsigned char c) {

	while(!(UCSR0A & (1<<UDRE0))); // wait until sending is possible
	UDR0 = c; // output character saved in c
}

void uart_puts(char *s) {
	while(*s){
		uart_putc(*s);
		s++;
	}
}

void writeVL53L0X(uint8_t rgstr, uint8_t val) {
	
}


char buffer[16];
uint16_t count = 0;
int main(void)
{
	DDRD |= (1<<PIN_LED_B);
	DDRD |= (1<<PIN_LED_G);
	DDRD |= (1<<PIN_LED_R);
	
	init_uart(9600);
	i2c_init();
   
    while (1) 
    {
		itoa(count, buffer, 10);
		uart_puts(buffer);
		uart_puts("\r\n");
	    PORTD |= (1<<PIN_LED_B);
	    _delay_ms(10);
	    PORTD &= ~(1<<PIN_LED_B);
	    _delay_ms(990);
	    count++;
    }
}

