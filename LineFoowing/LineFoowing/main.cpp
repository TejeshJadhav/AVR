#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include "lcd.c"

bool debug = false;

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void smart_left();
void smart_right();
void read_line_sensor();


volatile bool delimit = false;
volatile bool slashnflag = false;
volatile bool button_flag = false;

unsigned int node_count = 0;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
char last_state = 'c';

char data[25];
char h[25];
char a[25];
int data_count = 0;

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning



//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void uart2_init()
{
	UCSR2B = 0x00;									//Disable while setting baud rate.
	UCSR2A = 0x00;
	UCSR2C = 0x06;
	UBRR2L = 0x5F;									//Set baud rate low.
	UBRR2H = 0x00;									//Set baud rate high.
	UCSR2B = 0x98;	
}

/*
void cat(volatile char* dest_ptr,volatile char * src_ptr)
{
	if((NULL != dest_ptr) && (NULL != src_ptr))
	{
		while(NULL != *dest_ptr)
		{
			dest_ptr++;
		}
		while(NULL != *src_ptr)
		{
			*dest_ptr++ = *src_ptr++;
		}
		*dest_ptr = NULL;
	}
}*/

void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void button_pin_config (void)
{
	DDRE  = DDRE & 0x7F;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x80; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init()
{
	uart2_init();
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	button_pin_config();
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

void button_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | (1<<ISC71); // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | (1<<INT7); // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

ISR(INT7_vect)
{
	button_flag = true;
}

int cursor_pos = 1;

ISR(USART2_RX_vect)		// ISR for receive complete interrupt
{
	data[data_count] = UDR2;
	if(data[data_count] != '\n' && slashnflag == false)
	{
		strcat(h,&data[data_count]);
		data_count++;
	}
	else if(data[data_count] == '\n' && slashnflag == false)
	{
		slashnflag = true;
	}
	else if (slashnflag == true && data[data_count] != '#')
	{
		strcat(a,&data[data_count]);
		data_count++;
	}
	else if (data[data_count] == '#')
	{
			delimit = true;
			lcd_cursor(1,1);
			lcd_string(h);
			lcd_cursor(2,1);
			lcd_string(a);
	}
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sensor Values At Desired Row And Column Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void stop (void)
{
	motion_set (0x00);
}

void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/4.090; // division by resolution to get shaft count  //5.44
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if (debug == true)
		{
			lcd_print(2,1,ShaftCountLeft,2);
			lcd_print(2,4,ReqdShaftCountInt,2);
			lcd_print(2,7,ShaftCountRight,2);
		}
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}


void forward (void) 
{
  motion_set (0x06);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
	velocity(0,100);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
	velocity(100,0);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	velocity(255,255);
	left(); //Turn left
	angle_rotate(Degrees);
	last_state = 'l';
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	velocity(255,255);
	right(); //Turn right
	angle_rotate(Degrees);
	last_state = 'r';
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void correction_smart_left()
{
	soft_left();
	velocity(150,150);
	_delay_ms(200);
	read_line_sensor();
	unsigned char vel = 255;
	while(1)
	{
		read_line_sensor();
		if(Center_white_line > 0x15)
		{
			stop();
			vel = vel - 5;
			velocity(vel,vel);
			break;
		}
	}
}

void correction_smart_right()
{
	soft_left();
	velocity(150,150);
	_delay_ms(200);
	read_line_sensor();
	unsigned char vel = 255;
	while(1)
	{
		read_line_sensor();
		if(Center_white_line > 0x15)
		{
			stop();
			vel = vel - 5;
			velocity(vel,vel);
			break;
		}
	}
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	button_interrupt_init();
	adc_init();
	timer5_init();
	lcd_set_4bit();
	lcd_init();
	sei();   //Enables the global interrupts
}

void read_line_sensor()
{
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			if (debug == true)
			{
				print_sensor(1,1,3);	//Prints value of White Line Sensor1
				print_sensor(1,5,2);	//Prints Value of White Line Sensor2
				print_sensor(1,9,1);	//Prints Value of White Line Sensor3
			}
}

void line_follow()
{
	read_line_sensor();
	while((Center_white_line>0x15 && Left_white_line>0x15) || (Center_white_line>0x15 && Right_white_line>0x15))
	{
		forward();
		velocity(255,255);
		read_line_sensor();
	}
	while(1)
	{
	flag=0;
	read_line_sensor();
	if (Center_white_line<0x15 && Right_white_line<0x15 && Left_white_line<0x15)
	{
		if(last_state == 'c')
		{
			velocity(255,255);
			forward();
		}
		else if(last_state == 'l')
		{
			//stop();
			//velocity(80,255);
			//forward();
			soft_left();	//soft left
			//correction_smart_left();
		}
		else if(last_state == 'r')
		{
			//stop();
			//velocity(255,80);
			//forward();			
			soft_right();	//soft right
			//correction_smart_left();		
		}
	}
	else if(Center_white_line>0x15 && Left_white_line>0x15 && Right_white_line>0x15)
	{
		stop();
		velocity(0,0);
		node_count++;
		if (last_state == 'l')
		{
			last_state = 'r';
		}
		else if (last_state == 'r')
		{
			last_state = 'l';
		}
		//last_state = 'c';
		break;
	}
	else
	{
		if(Left_white_line > 25 &&  Center_white_line < 25 && Right_white_line < 25)
		{
			//stop();
			soft_left();	//soft left
			//correction_smart_left();
			//			velocity(80,255);
			//			forward();
			while(Center_white_line < 25)
			{
				read_line_sensor();
			}
			last_state = 'l';
		}
		if(Left_white_line < 25 && Center_white_line < 25 && Right_white_line > 25)
		{
			//stop();
			soft_right();	//soft right
			//correction_smart_right()
			//			velocity(255,80);
			//			forward();
			while(Center_white_line < 25)
			{
				read_line_sensor();
			}
			last_state = 'r';
		}
		if(Left_white_line < 25 && Center_white_line > 25 && Right_white_line < 25)
		{
			
			velocity(255,255);
			forward();
			last_state = 'c';
		}
	}
	if (debug == true)
	{
		lcd_print(1,13,node_count,2);
	}
	}
}


void smart_left()
{
	left();
	velocity(150,150);
	_delay_ms(200);
	read_line_sensor();
	unsigned char vel = 255;
	while(1)
	{
		read_line_sensor();
		if(Left_white_line > 0x15)
		{
			stop();
			vel = vel - 10;
			if(vel<60) vel=60;
			velocity(vel,vel);
			break;	
		}
	}
}

void smart_right()
{
	right();
	velocity(150,150);
	_delay_ms(200);
	read_line_sensor();
	unsigned char vel = 255;
	while(1)
	{
		read_line_sensor();
		if(Right_white_line > 0x15)
		{
			stop();
			vel = vel - 5;
			velocity(vel,vel);
			break;
		}
	}
}
void serial_print_lcd(char *str)
{	
	char l1[20],l2[20];
	bool flag = true;
	int count = 0;
	//lcd_cursor(1,1);
	//lcd_wr_char('a');
	for(int i=0; i<= 25; i++)
	{
		if(str[i] != '\n' && flag == true)
		{
			l1[i] = str[i];
			l1[i+1] = '\0';
			//lcd_wr_char('b');
		}
		else
		{
			flag = false;
			l2[count] = str[i];
			l2[count+1] = '\0';
			count++;
		}
		
	}
	lcd_cursor(1,1);
	lcd_string(l1);
	lcd_cursor(2,1);
	lcd_string(l2);
}

//Main Function
int main()
{
	init_devices();
	//_delay_ms(5000);
	//lcd_cursor(1,1);
	while(!(delimit == true  && button_flag == true))
	{
		_delay_ms(1);
	}
	while(1)
	{
		_delay_ms(200);
		line_follow();
		//_delay_ms(500);
		//smart_left();
		//_delay_ms(500);
		//line_follow();
		//_delay_ms(500);
		//smart_right();
		//left_degrees(70);  //65 pe working properly;
		//_delay_ms(500);
	}
}