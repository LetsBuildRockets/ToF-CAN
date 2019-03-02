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
#include <avr/interrupt.h>

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

#define CAN_BUS_ADDR 0x620

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
		printf("daten:  ");
		
		for (uint8_t i = 0; i < length; i++) {
			printf("0x%02x ", message->data[i]);
		}
		printf("\r\n");
	}
}

Adafruit_VL53L0X tof = Adafruit_VL53L0X();

char buffer[16];
uint16_t count = 0;
int main(void)
{
	static FILE uart_stdout;
	fdev_setup_stream(&uart_stdout, fput, NULL, _FDEV_SETUP_WRITE);
	stdout = &uart_stdout;
	
	DDRD |= (1<<PIN_LED_B);
	DDRD |= (1<<PIN_LED_G);
	DDRD |= (1<<PIN_LED_R);
		
	init_uart(9600);
	_delay_us(200);
	
	//sei();
	if (!mcp2515_init()) {
		uart_puts("Can't communicate with MCP2515!\r\n");
		for (;;);
	}
	else {
		uart_puts("MCP2515 is active\r\n");
	}
	i2c_init();
	tof.begin();
	
	
	VL53L0X_RangingMeasurementData_t measure;
	tCAN message;
		
	message.id = CAN_BUS_ADDR;
	message.header.rtr = 0;
	message.header.length = 2;
	message.data[0] = 0xab;
	message.data[1] = 0xcd;
		
		
	//printf("\r\nwechsle zum Loopback-Modus\r\n");
	//mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));

	//mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    printf("CNF1: 0x%02x\r\n", mcp2515_read_register(CNF1));
	while (1) 
    {
		
		tof.rangingTest(&measure, false);
		if (measure.RangeStatus != 4) {
			uint16_t distance = measure.RangeMilliMeter;
			printf("Distance (mm): %ld", distance);
			uart_puts(buffer);
			uart_puts("\r\n");
			
			message.data[0]= (distance>>8) & 0xFF;
			message.data[1]= (distance) & 0xFF;
			
			if (mcp2515_send_message(&message)) {
				uart_puts("sent them bytes");
			} else {
				uart_puts("Error writing message to can bus!\r\n");
				print_can_message(&message);
			}
			
		} else {
			// OUT OF RANGE
		}
		
	    PORTD |= (1<<PIN_LED_B);
	    _delay_ms(1);
	    PORTD &= ~(1<<PIN_LED_B);
	    _delay_ms(999);
	    count++;
    }
}

