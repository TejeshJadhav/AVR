#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "lcd.c"
#include "locomotion.h"

bool debug = false;

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void read_line_sensor();


volatile bool delimit = false;
volatile bool slashnflag = false;
volatile bool button_flag = false;

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;


char data[25];
char h[25];
char a[25];
int data_count = 0;



struct coor
{
	int x;
	int y;
};

struct cell
{
	int num;
	int cor[4][2];
};

struct animal
{
	char name[3];
	int cor[1][2];
};

struct animal animals[20];
struct cell cells[25];

void def_cells()
{
	int a = 0,b = 0;
	int x = 0, y = 1;
	int count = 0;
	for(b = 0; b < 5; b++)
	{
		for(a = 0; a < 5; a++ )
		{
			cells[count].num = count;
			
			cells[count].cor[0][x] = a;
			cells[count].cor[0][y] = b;
			cells[count].cor[1][x] = a;
			cells[count].cor[1][y] = b + 1;
			cells[count].cor[2][x] = a + 1;
			cells[count].cor[2][y] = b;
			cells[count].cor[3][x] = a + 1;
			cells[count].cor[3][y] = b + 1;
			count++;
		}
	}
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


//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

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
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	button_pin_config();
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

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2

 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
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

void back (void)
{
	motion_set (0x09);
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

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	button_interrupt_init();
	uart2_init();
	adc_init();
	timer1_init();
	timer5_init();
	lcd_set_4bit();
	lcd_init();
	sei();   //Enables the global interrupts
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
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
		//_delay_ms(300);
		forward_mm(120);
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
int read_proximity()
{
	lcd_print(1,1,ADC_Conversion(6),3);
	return ADC_Conversion(12);
}

void nagada()
{
	servo_1(0);
	servo_2(0);
}

void pick()
{
	_delay_ms(200);
	servo_1(90);
	_delay_ms(800);
	servo_2(90);
	_delay_ms(700);
	servo_1(0);
}

void drop()
{
	servo_2_free();
	servo_1_free();
}

void pick_from_left()
{
	velocity(200,200);
	soft_left_2_degrees(80);
	_delay_ms(200);
	back_mm(30);
	stop();
	pick();
	_delay_ms(200);
	soft_right_degrees(80);
}

void pick_from_right()
{
	velocity(200,200);
	soft_right_2_degrees(80);
	_delay_ms(200);
	back_mm(30);
	stop();
	pick();
	_delay_ms(200);
	soft_left_degrees(80);
}

//Main Function
int main()
{
	init_devices();
	//_delay_ms(5000);
	//lcd_cursor(1,1);
	
	/*nagada();
	while(!(delimit == true  && button_flag == true))
	{
		_delay_ms(1);
	}
	while(1)
	{
		_delay_ms(200);
		line_follow();
		_delay_ms(200);
		pick();
		_delay_ms(2000);
		nagada();
		//_delay_ms(500);
		//smart_left();
		//_delay_ms(500);
		//line_follow();
		//_delay_ms(500);
		//smart_right();
		//left_degrees(70);  //65 pe working properly;
		//_delay_ms(500);
	}*/
	nagada();
	while(1)
	{
		//nagada();
		line_follow();
		_delay_ms(200);
		pick_from_right();
		nagada();
		line_follow();
		_delay_ms(200);
		pick_from_left();
	}
	/*while(1)
	{
		read_proximity();
	}*/
}