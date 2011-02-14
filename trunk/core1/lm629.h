/******************************************************************************
Hong Kong Univerisity Of Science and Technology, Copyright (c) 2005
Robocon 2005 Team 1

$Id: lm629.h 108 2005-06-22 07:17:53Z sam $

******************************************************************************/

#ifndef _LM629_H_
#define _LM629_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "delay.h"
#include <inttypes.h>
#include <avr/eeprom.h>

/*

	lm629_set_filter	0      0      30       0    1024    2048 # driving L
	lm629_set_filter	1      0      30       0    1024    2048 # driving R

*/


/******************************************************************************
  Declaration
******************************************************************************/
// Flag to monitor trajectory completion
#define LM629_TRAJ_COMPLETION 0x08
#define LM629_POSITION_ERROR 0x04
#define LM629_BREAK_POINT 0x02



// LM629 Command Addresses
#define LM629_RESET    0x00 // Reset LM629
#define LM629_DFH      0x02 // Define Home
#define LM629_SIP      0x03 // Set Index Position
#define LM629_LPEI     0x1B // Interrupt on Error
#define LM629_LPES     0x1A // Stop on Error
#define LM629_SBPA     0x20 // Set Breakpoint, Absolute
#define LM629_SBPR     0x21 // Set Breakpoint, Relative
#define LM629_MSKI     0x1C // Mask Interrupts
#define LM629_RSTI     0x1D // Reset Interrupts
#define LM629_LFIL     0x1E // Load Filter Parameters
#define LM629_UDF      0x04 // Update Filter
#define LM629_LTRJ     0x1F // Load Trajectory
#define LM629_STT      0x01 // Start Motion
#define LM629_RDSIGS   0x0C // Read Signals Register
#define LM629_RDIP     0x09 // Read Index Position
#define LM629_RDDP     0x08 // Read Desired Position
#define LM629_RDRP     0x0A // Real Real Position
#define LM629_RDDV     0x07 // Read Desired Velocity
#define LM629_RDRV     0x0B // Read Real Velocity
#define LM629_RDSUM    0x0D // Read Integration Sum

//Declare Port
#define DataDDR DDRA

#define DataPort PORTA
#define DataPin PINA

#define CMDDR DDRG

#define CMPORT PORTG
#define RD 1
#define WR 0
#define PS 4
#define RST 3

#define CSDDR DDRE
#define CSPORT PORTE
#define CS0 2			//U10
#define CS1 3			//U10
#define CS2 4			//U11
#define CS3 5			//U12
#define CS4 6			//U13

//Reset Trial
#define ResetTrial 10

#define TrajCompleteStatus 2
#define PositionError 5
#define BreakPoint 6
//vel=4*MOTOR_CPR*65536*vel*MOTOR_RATIO*2048/(LM629_CLK*1000000);  

#define PI	3

#define	m2count(M,CPR,GEAR,DIAMETER) 4*CPR*GEAR*M/(DIAMTER*PI)
#define	ms2count(MS,CPR,GEAR,DIAMETER) 4*65536*2048/6e6*CPR*MS/(DIAMTER*PI)*GEAR
#define ms22count(MS2,CPR,GEAR,DIAMETER) 4*65536*2048*2048/6e6/6e6*CPR*MS2/(DIAMETER*PI)*GEAR

#define count2m(C,CPR,GEAR,DIAMTER) C*DIAMTER*PI/(4*CPR*GEAR)
#define count2ms(C,CPR,GEAR,DIAMTER) C*DIAMTER*PI/(4*65546*2048*6e6*CPR*GEAR)

/******************************************************************************
  Variable Declaration
******************************************************************************/
// 2Hz and 64Hz flag
extern uint8_t lm629_flag;
extern uint8_t lm629_chip_available;


extern EEMEM uint16_t lm629_filter[6][4];
extern EEMEM uint32_t lm629_acc[6];
extern EEMEM uint16_t lm629_pe[6];



/******************************************************************************
  Procedure Declaration
******************************************************************************/
// LM629 Initialization
void lm629_init (void);


void lm629_velocity_start (const uint8_t chip,int32_t vel);
void lm629_position_start (const uint8_t chip,const uint32_t vel,const int32_t pos);



void lm629_read_pos_twin (int32_t * pos0,int32_t * pos1);
void lm629_read_pos_twin_home (int32_t * pos0,int32_t * pos1);


void lm629_abs_position_start (const uint8_t chip,const uint32_t vel,const int32_t pos);
void lm629_rel_position_start (const uint8_t chip,const uint32_t vel,const int32_t pos);


void lm629_velocity_twin_start (int32_t vel1, int32_t vel2);


void lm629_position_twin_start (const uint32_t vel1,const uint32_t vel2, int32_t pos);




void lm629_set_pe_report (const uint8_t chip, const uint16_t pe);
void lm629_set_pe_stop (const uint8_t chip, const uint16_t pe);


void lm629_zero_drive (const uint8_t chip);
void lm629_stop_abruplty (const uint8_t chip);


void lm629_acceleration (const uint8_t chip,const uint32_t acc);


int32_t lm629_read_pos (const uint8_t chip);
int32_t lm629_read_pos_home (const uint8_t chip);

int32_t lm629_read_rel_vel (const uint8_t chip);
int32_t lm629_read_des_vel (const uint8_t chip);


void lm629_set_filter (const uint8_t chip,const uint16_t kp,const uint16_t ki,const uint16_t kd,const uint16_t il);



uint8_t lm629_pos_done(const uint8_t chip); 
uint8_t lm629_is_pe(const uint8_t chip); 
uint8_t lm629_bp_reach(const uint8_t chip);


void lm629_define_home (const uint8_t chip);


void lm629_set_abs_bp (const uint8_t chip,const int32_t bp);
void lm629_set_rel_bp (const uint8_t chip,const int32_t bp);

void lm629_enable_output(void);
void lm629_disable_output(void);

void lm629_reset_pe(const uint8_t chip);
void lm629_abs_pos_start_done(const uint8_t chip, const uint32_t vel, const int32_t pos);
void lm629_abs_pos_start_done_twin(const uint8_t motion, const uint32_t vel, const int32_t pos);

#endif
