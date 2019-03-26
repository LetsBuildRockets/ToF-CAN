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

#include "defs.h"

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

#include "libs/I2C-master-lib/i2c_master.h"
#include "libs/I2C-master-lib/i2c_master.c"
#include "libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h"
#include "libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.cpp" // <-- sue me. this actually decreases memory ussage, cause we don't get the whole staic library
#include "libs/mcp2515/mcp2515.h"
#include "libs/mcp2515/mcp2515.c"

#define PIN_LED_B DDD6
#define PIN_LED_G DDD5
#define PIN_LED_R DDD7

#define DEFAULT_CAN_BUS_ADDR 0x620ul
#define EEPROM_ADDR_LOCATION 0x0000ul

#define COLOR_OFF 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_BLUE 3

#define VERSION_MAJOR 1
#define VERSION_MINOR 1

int errorCode = ERROR_NONE, lastErrorCode = -1;
uint16_t lastDistance = 0;
uint32_t canBusAddr = 0;


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
	UCSR0C |= (1<<USBS0) | (3<<UCSZ00); //Modus Asynchronous 8N1 (8 Data bits, No Parity, 1 Stop bit)
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
	
	if (!message->header.rtr)
	{	
		printf("data:  ");
		
		for (uint8_t i = 0; i < length; i++)
		{
			printf("0x%02x ", message->data[i]);
		}
		printf("\r\n");
	}
}

Adafruit_VL53L0X tof = Adafruit_VL53L0X();

uint8_t count = 0;

int sendCANmsg(uint8_t bytes[], int length)
{
		tCAN message;
		
		message.id = canBusAddr;
		message.header.rtr = 0;
		message.header.length = length;
		for(int i = 0; i<length; i++)
			message.data[i] = bytes[i];
		if (mcp2515_send_message(&message))
		{
			//uart_puts("sent them bytes\r\n");
			return 0;
		}
		else
		{
			uart_puts("Error writing message to can bus!\r\n");
			print_can_message(&message);
			errorCode = ERROR_WRITING_TO_CAN;
			return -1;
		}
}

int sendError(uint8_t _err)
{
	uint8_t bytes[2];
	bytes[0] = CTRL_SEND_ERROR;
	bytes[1] = _err;
	return sendCANmsg(bytes, 2);	
}

int sendMeasurement(uint16_t distance)
{
	uint8_t bytes[3];
	bytes[0] = CTRL_SEND_DISTANCE;
	bytes[1] = (distance>>8) & 0xFF;
	bytes[2] = (distance) & 0xFF;
	return sendCANmsg(bytes, 3);
}

void init()
{
	eeprom_busy_wait();
	uint16_t newAddr = eeprom_read_word(EEPROM_ADDR_LOCATION);
	if(newAddr >= 0x0620 && newAddr <0x1000)
	{
		printf("Got CAN address in EEPROM: 0x%04x!\r\n", newAddr);
		canBusAddr = newAddr;
	}
	else
	{
		printf("Bad CAN address in EEPROM, defaulting to 0x%04lx!\r\n", DEFAULT_CAN_BUS_ADDR);
	}
	if (!mcp2515_init(canBusAddr))
	{
		uart_puts("Can't communicate with MCP2515!\r\n");
		errorCode = ERROR_INIT_CAN;
		sendError(errorCode);
		for (;;);
	}
	else
	{
		uart_puts("MCP2515 is active\r\n");
	}
	i2c_init();
	if(!tof.begin()){
		uart_puts("Can't connect to VL53L0X!");
		errorCode = ERROR_INIT_VL53L0X;
		sendError(errorCode);
		for (;;);
	}
}

ISR(INT0_vect)
{
	// This is a really long ISR (also it has a busy loop in it...), but it should rarely be called (only during init), so I'm not too worried about it
	
	tCAN message;
	mcp2515_get_message(&message);
	uart_puts("We received a message!\r\n");
	print_can_message(&message);
	if(message.id > 0 && message.id == canBusAddr) // I'm not sure this check is necessary, but I want to be sure that if the mask/filters aren't working, we ignore the data
	{
		uint8_t len = message.header.length;
		print_can_message(&message);
		switch (message.data[0])
		{
			case CTRL_GET_FIRMWARE_VERSION:
			{
				uint8_t bytes[3] = {CTRL_SEND_FIRMWARE_VERSION, VERSION_MAJOR, VERSION_MINOR};
				sendCANmsg(bytes, 3);
				break;
			}
			case CTRL_SET_NEW_ADDR:
			{
				if(len > 3)
				{
					uint16_t newAddr = message.data[1] << 8 | message.data[2];
					eeprom_busy_wait();
					eeprom_write_word(EEPROM_ADDR_LOCATION, newAddr);
					init();
				}
				else
				{
					sendError(ERROR_NOT_ENOUGH_DATA_BYTES);
				}
				break;
			}
			default:
			{
				// hmm... command byte doesn't seem right...
				sendError(ERROR_BAD_CTRL_BYTE);
			}
		}
	}
}


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
			
	init();
	
	_delay_ms(500);
	
	// interrupt setup
	DDRD |= 1<< DDD2;		// Set PD2 as input (Using for interrupt INT0)
	PORTD |= 1<< DDD2;		// Enable PD2 pull-up resist
	EICRA = (1<<ISC01)| (0<<ISC00);					// Enable INT0 on rising
	
	
	VL53L0X_RangingMeasurementData_t measure;

	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
	while (1) 
    {
		cli();
		if(errorCode != ERROR_INIT_CAN && errorCode != ERROR_INIT_VL53L0X)
		{
			tof.rangingTest(&measure, false);
			if (measure.RangeStatus != 4)
			{
				uint16_t distance = measure.RangeMilliMeter;
				if(distance < 8191)
				{
					printf("Distance (mm): %u, range status:%d\r\n", distance, measure.RangeStatus);
					// let's not flood the can bus...
					if(distance != lastDistance)
					{						
						sendMeasurement(distance);
						lastDistance = distance;
					}
				
					errorCode = ERROR_NONE;
				}
				else
				{
					// sometimes we get a bad reading of 8191 or 8192?
					errorCode = ERROR_OUT_OF_RANGE;
				}
			}
			else
			{
				// OUT OF RANGE
				errorCode = ERROR_OUT_OF_RANGE;
			}
		}
		
		// let's not flood the can bus...
		if(errorCode != lastErrorCode)
		{
			sendError(errorCode);
			lastErrorCode = errorCode;
		}
				
		if(errorCode == ERROR_NONE)
		{
				setColor(COLOR_GREEN);
			}
			else if(errorCode == ERROR_OUT_OF_RANGE)
			{
				setColor(COLOR_BLUE);
			}
			else if(errorCode == ERROR_WRITING_TO_CAN)
			{
				if(count<3)
				{
						setColor(COLOR_RED);
					}
					else
					{
						setColor(COLOR_OFF);
				}
			}
			else
			{
			setColor(COLOR_RED);
		}
		if(++count>4) count = 0;
		
		sei();
			
	    _delay_ms(15);

    }
}

