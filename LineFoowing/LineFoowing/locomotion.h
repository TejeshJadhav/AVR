/*
 * locomotion.h
 *
 * Created: 02-02-2019 06:54:44 PM
 *  Author: tejesh
 */ 


#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
char last_state = 'c';
unsigned int node_count = 0;

void motors_delay();
void smart_left();
void smart_right();
void velocity(unsigned char, unsigned char);
void left_encoder_pin_config (void);
void right_encoder_pin_config (void);
void motion_pin_config (void);
void left_position_encoder_interrupt_init (void);
void right_position_encoder_interrupt_init (void);
void motion_set (unsigned char);
void stop (void);
void angle_rotate(unsigned int);
void forward (void);
void back (void);
void left (void);
void right (void);
void soft_right (void);
void left_degrees(unsigned int);
void right_degrees(unsigned int);
void soft_left_2 (void);
void soft_right_2 (void);
void soft_left_degrees(unsigned int);
void soft_left_2_degrees(unsigned int);
void soft_right_2_degrees(unsigned int);
void correction_smart_left();
void correction_smart_right();
void linear_distance_mm(unsigned int);
void back_mm(unsigned int);
void forward_mm(unsigned int);
void line_follow();
#endif /* LOCOMOTION_H_ */