// 4/2/2010   
#ifndef _PID_H_
#define _PID_H_

#include <avr/io.h>
// for 3600 Gyro
#include <avr/interrupt.h>
#include "delay.h"
#include <inttypes.h>
#include <avr/eeprom.h>
#include "lm629.h"
#include "uart_comm.h"
#include "tri_val.h"

#define robot_width 250
#define RAD 1000

struct PID_path
{
	int16_t x;
	int16_t y;
};

extern int32_t KP;
extern int32_t KI;
extern int32_t KD;
extern int32_t IL;
extern int32_t UL;

extern int32_t PID_desire_angle;
extern int32_t PID_start_angle;
extern int32_t PID_start_X;
extern int32_t PID_start_Y;
extern int32_t PID_target_X;
extern int32_t PID_target_Y;
extern int32_t PID_cos;                         //cos and sin give the direction of the line
extern int32_t PID_sin;

extern int32_t PID_center_X;
extern int32_t PID_center_Y;
extern int32_t PID_radius;
extern int8_t PID_dir;

extern volatile int32_t PID_error_position;
extern volatile int32_t PID_error;
extern volatile int32_t PID_prev_error;	
extern volatile int32_t PID_integration;
extern volatile int32_t PID_u;   //adjust velocity on angle error

extern uint8_t PID_status;                 //0 : not working  1 : only angle 3 : both angle and position

void PID_calculation_u(void);             // add this function to an interupt to get u-angle and u-position

/*  set the target before moving each time  */
void PID_set_target(void);                   //keep the angle      
void PID_set_target_distance(int32_t pos);   //keep the angle and stop after the distance
void PID_set_target_angle(int16_t angle);    //set a desired angle the robot will turn to that angle
void PID_set_target_XY(int32_t X,int32_t Y,int32_t SX,int32_t SY,int8_t dir); //set a line from the current position to the target 
void PID_set_target_XYAngle(int32_t X,int32_t Y,int32_t angle);  //set a line path through the target slope is the angle
void PID_set_target_XYAngle_back(int32_t X,int32_t Y,int32_t angle); 

int PID_position_done(void);    //return 1 to show the robot reach the target
int PID_arc_done(void);
int PID_angle_done(void);      //return 1 to show the robot turned to the target angle

void PID_velocity_twin_start(int32_t vel1, int32_t vel2);  //keep running this to adjust the speed
void PID_set_status(uint8_t status);  //0 : not working  1 : only angle 3 : both angle and position
void PID_disable(void);  //0 : not working 

void PID_set_filter(int32_t kp,int32_t ki,int32_t kd,int32_t il,int32_t ul);         //set kp ki kd il=integration limit ul = u limit

void PID_set_polyline(int16_t sa, int16_t rad, int16_t radius);

int8_t PID_path_polyline(struct PID_path path[],uint8_t num,int32_t speed,int32_t final_angle);
//int8_t PID_path_polyline_back(struct PID_path path[],uint8_t num,int32_t speed,int32_t final_angle);
void PID_path_arc(int32_t x,int32_t y,int32_t sx, int32_t sy , int32_t sa);
void PID_arc_speed(int32_t v[]);  //Calculate the speed of left and right wheel when doing arc moving


#endif