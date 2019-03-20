/*
 * ToF-CAN-Firmware.cpp
 *
 * Created: 2/28/2019 7:40:37 PM
 * Author : erics
 */ 


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
#include <avr/interrupt.h>
#include <avr/eeprom.h> 

#define TWBR TWBR0
#define TWCR TWCR0
#define TWSR TWSR0
#define TWDR TWDR0

#define SPDR SPDR0
#define SPCR SPCR0
#define SPSR SPSR0

#include "libs/I2C-master-lib/i2c_master.h"
#include "libs/I2C-master-lib/i2c_master.c"
#include "libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h"
#include "libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.cpp" // <-- sue me. this actually decreases memory ussage, cause we don't get the whole staic library
#include "libs/mcp2515/mcp2515.h"
#include "libs/mcp2515/mcp2515.c"

#define PIN_LED_B DDD6
#define PIN_LED_G DDD5
#define PIN_LED_R DDD7

#define DEFAULT_CAN_BUS_ADDR 0x620
#define EEPROM_ADDR_LOCATION 0x0000

#define ERROR_NONE 0
#define ERROR_OUT_OF_RANGE 1
#define ERROR_WRITING_TO_CAN 2
#define ERROR_INIT_CAN 3
#define ERROR_INIT_VL53L0X 3

#define COLOR_OFF 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_BLUE 3

int errorCode = ERROR_NONE;
uint32_t canBusAddr;


void setColor(int color) {
	switch(color){
		case COLOR_RED:
			PORTD &= ~(1<<PIN_LED_G);
			PORTD &= ~(1<<PIN_LED_B);
			PORTD |= (1<<PIN_LED_R);
			break;
		case COLOR_GREEN:
			PORTD &= ~(1<<PIN_LED_R);
			PORTD &= ~(1<<PIN_LED_B);
			PORTD |= (1<<PIN_LED_G);
			break;
		case COLOR_BLUE:
			PORTD &= ~(1<<PIN_LED_R);
			PORTD &= ~(1<<PIN_LED_G);
			PORTD |= (1<<PIN_LED_B);
			break;
		default:
			PORTD &= ~(1<<PIN_LED_R);
			PORTD &= ~(1<<PIN_LED_G);
			PORTD &= ~(1<<PIN_LED_B);
		break;
	}
}

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

static int fput(char c, FILE* f) {
	uart_putc(c);
	return 0;
}

void print_can_message(tCAN *message)
{
	uint8_t length = message->header.length;
	
	printf("id:     0x%3x\r\n", message->id);
	printf("length: %d\r\n", length);
	printf("rtr:    %d\r\n", message->header.rtr);
	
	if (!message->header.rtr) {	
		printf("data:  ");
		
		for (uint8_t i = 0; i < length; i++) {
			printf("0x%02x ", message->data[i]);
		}
		printf("\r\n");
	}
}

Adafruit_VL53L0X tof = Adafruit_VL53L0X();

uint8_t count = 0;

int main(void)
{
	static FILE uart_stdout;
	fdev_setup_stream(&uart_stdout, fput, NULL, _FDEV_SETUP_WRITE);
	stdout = &uart_stdout;
	
	DDRD |= (1<<PIN_LED_B);
	DDRD |= (1<<PIN_LED_G);
	DDRD |= (1<<PIN_LED_R);
	
	setColor(COLOR_RED);
		
	init_uart(9600);
	_delay_ms(500);
	
	if (!mcp2515_init()) {
		uart_puts("Can't communicate with MCP2515!\r\n");
		errorCode = ERROR_INIT_CAN;
		for (;;);
	}
	else {
		uart_puts("MCP2515 is active\r\n");
	}
	i2c_init();
	if(!tof.begin()){
		uart_puts("Can't connect to VL53L0X!");
		errorCode = ERROR_INIT_VL53L0X;
	}
	
	uint32_t newAddr = eeprom_read_word(EEPROM_ADDR_LOCATION);
	if(newAddr >= 0x0620 && newAddr <0x1000) {
		printf("Got CAN address in EEPROM: 0x%04x!\r\n", newAddr);
		canBusAddr = newAddr;
	} else {		
		printf("Bad CAN address in EEPROM, defaulting to 0x%04x!\r\n", DEFAULT_CAN_BUS_ADDR);
	}	
	
	_delay_ms(500);
	
	VL53L0X_RangingMeasurementData_t measure;
	tCAN message;
		
	message.id = canBusAddr;
	message.header.rtr = 0;
	message.header.length = 3;
	message.data[0] = 0x00; // byte error code
	message.data[1] = 0x00; // byte H
	message.data[2] = 0x00; // byte L
		

	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
	while (1) 
    {
		if(errorCode != ERROR_INIT_CAN && errorCode != ERROR_INIT_VL53L0X) {
			tof.rangingTest(&measure, false);
			if (measure.RangeStatus != 4) {
				uint16_t distance = measure.RangeMilliMeter;
				if(distance < 8191) {
					printf("Distance (mm): %u, range status:%d\r\n", distance, measure.RangeStatus);
				
					message.data[1] = (distance>>8) & 0xFF;
					message.data[2] = (distance) & 0xFF;
				
					errorCode = ERROR_NONE;
				} else {
					// sometimes we get a bad reading of 8191 or 8192?
					errorCode = ERROR_OUT_OF_RANGE;
				}
			} else {
				// OUT OF RANGE
				errorCode = ERROR_OUT_OF_RANGE;
			}
		}
		
		
		message.data[0] = errorCode;
		
		if (mcp2515_send_message(&message)) {
			//uart_puts("sent them bytes\r\n");
		} else {
			uart_puts("Error writing message to can bus!\r\n");
			print_can_message(&message);
			errorCode = ERROR_WRITING_TO_CAN;
		}
						
	    _delay_ms(15);
		if(errorCode == ERROR_NONE) {
			setColor(COLOR_GREEN);
		} else if(errorCode == ERROR_OUT_OF_RANGE) {
			setColor(COLOR_BLUE);
		} else if(errorCode == ERROR_WRITING_TO_CAN) {
			if(count<3) { 
				setColor(COLOR_RED);
			} else {
				setColor(COLOR_OFF);
			}
		} else {
			setColor(COLOR_RED);
		}
		if(++count>4) count = 0;
    }
}

