/*
 Fire Bird V ATMEGA2560 microcontroller board has connection for 3 servo motors (S1, S2, S3).
 Servo motors move between 0 to 180 degrees proportional to the pulse train 
 with the on time of 0.6 to 2 ms with the frequency between 40 to 60 Hz. 50Hz is most recommended.

 We are using Timer 1 at 10 bit fast PWM mode to generate servo control waveform.
 In this mode servo motors can be controlled with the angular resolution of 1.86 degrees.
 Although angular resolution is less this is very simple method.
 
 There are better ways to produce very high resolution PWM but it involves interrupts at the frequency of the PWM.
 High resolution PWM is used for servo control in the Hexapod robot.
  
 Connection Details:	PORTB 5 OC1A --> Servo 1: Camera pod pan servo
 						PORTB 6 OC1B --> Servo 2: Camera pod tile servo
						PORTB 7 OC1C --> Servo 3: Reserved 
 					  	
 2. 5V supply to these motors is provided by separate low drop voltage regulator "5V Servo" which can
 	supply maximum of 800mA current. It is a good practice to move one servo at a time to reduce power surge 
	in the robot's supply lines. Also preferably take ADC readings while servo motor is not moving or stopped
	moving after giving desired position.
*********************************************************************************/

#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Configure PORTB 5 pin for servo motor 1 operation
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


//Initialize the ports
void port_init(void)
{
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
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


//Function to initialize all the peripherals
void init_devices(void)
{
 cli(); //disable all interrupts
 port_init();
 timer1_init();
 sei(); //re-enable interrupts 
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

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

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


//Main function
int main(void)
{
while(1)
{
 unsigned char i = 0;
 init_devices();
 //servo_2(70);	
 servo_1(90);
 for (i = 0; i < 80; i++)
 {
  //servo_1(i);
  servo_2(i);
  _delay_ms(20);
 }
 _delay_ms(5000);
 for (i = 80; i > 0; i--)
 {
	// servo_1(i);
	 servo_2(i);
	 _delay_ms(20);
 } 

// _delay_ms(700);
// servo_1_free(); 
// servo_2_free();
}
return 1;
}