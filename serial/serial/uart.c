/*
 * uart.c
 *
 * Created: 1/20/2019 11:21:01 AM
 *  Author: psknayak
 */ 

#define F_CPU 14745600
#define TRUE 1
#define FALSE 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

volatile unsigned char data[25];
volatile unsigned char h[25];
unsigned char count = 0;
volatile unsigned char flag;
void lcd_port_config(void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void buzzer_pin_config(void)
{
	DDRC = DDRC | 0x08; //Setting PORTC 3 as output
	PORTC = PORTC & 0xF7; //Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on(void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off(void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

void port_init()
{
	lcd_port_config();
	buzzer_pin_config();
}

void uart2_init(void)
{
	UCSR2B = 0x00; //disable while setting baud rate
	UCSR2A = 0x00;
	UCSR2C = 0x06;
	UBRR2L = 0x5F; //set baud rate lo
	UBRR2H = 0x00; //set baud rate hi
	UCSR2B = 0x98;
}

void append(char s[], char c)
{
	int len = sizeof(s)/sizeof(s[0]);
	s[len]=c;
	s[len+1]='\0';
}

ISR(USART2_RX_vect)		// ISR for receive complete interrupt
{
	data[count]= UDR2;
	UDR2 = data[count];
	if(data[count] == 0x0A)
	{
		lcd_cursor(2,1);
		pos = count;
		count++;
		flag = FALSE;
	}
	else if(count > pos && flag == FALSE)
	{
		lcd_wr_char(data[count]);
		append(a,data[count]);
		count++;
		flag = FALSE;
	}
	else if(flag==TRUE)
	{
		lcd_wr_char(data[count]);
		append(h,data[count]);
		count++;
		flag = TRUE;
	}
}
	
void init_devices(void)
{
	cli();
	port_init();
	uart2_init();
	sei();
}
int main(void)
{	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	lcd_cursor(1,1);
	while(1);
}			