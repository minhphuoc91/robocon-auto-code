#include "lm629.h"


// defined in lm629.h: #define LM629_REPORT_TWIN 		0xF0
uint8_t lm629_flag = 0;
uint8_t lm629_flag_ch[5] = {0, 0, 0, 0, 0}; // Channel
uint8_t lm629_chip_available = 0;

EEMEM uint16_t lm629_filter[6][4]=
{
{30,0,1024,2048},
{30,0,1024,2048},
{30,0,1024,2048},
{30,0,1024,2048},
{30,0,1024,2048},
{30,0,1024,2048}};

EEMEM uint32_t lm629_acc[6]={2048,2048,2048,2048,2048,2048};

EEMEM uint16_t lm629_pe[6]={20000,20000,20000,20000,20000,20000};




// LM629 low level access routines
// Define anything you have to write, probably Read/Write * Command/Data, Select/Reset chip
// e.g. static void write_command(uint8_t cmd);
// Note that there might not have a LM629 chip inserted. 
//   Pass the command as is without blocking if it's the case
static void SelectChip(uint8_t chip);
static uint8_t ReadStatus(void);
static uint8_t ReadData(void);
static void WriteCommand(uint8_t cmd);
static void WriteData(uint8_t data);
static inline uint32_t U8ToSignedLong(const uint8_t * data);
static inline void CheckBusy(void);

/******************************************************************************
  Procedures
******************************************************************************/

static inline void CheckBusy(void)
{
    /* wait until busy flag is cleared */
    uint8_t counter = 255;
    while (counter-- && (ReadStatus() & 0x01));		// Check Busy
}

/*SelectChip
	To decide which chip you want to use.
*/
static void SelectChip(uint8_t chip)
{
	chip <<= 2;
	CSPORT = chip & 0b00111100;
	uint8_t i;
	for(i=0; i<255; i++);
	for(i=0; i<255; i++);
	for(i=0; i<255; i++);
	for(i=0; i<255; i++);
}


/*ReadStatus
	To read the status of the motor.
	It will return a 8bit information in Hex mode.
	Bit Position  Function
	Bit 7			Motor Off
	Bit 6			BreakpointReached[Interrupt]
	Bit 5			Excessive Position Error[Interrupt]
	Bit 4			Wraparound Occurred[Interrupt]
	Bit 3			Index Pulse Observed[Interrupt]
	Bit 2			Trajectory Complete[Interrupt]
	Bit 1			Command Error[Interrupt]
	Bit 0			Busy Bit
	
	
	Caution:
		It should use "SelectChip" function to choose a chip before using this function.
*/

static uint8_t ReadStatus(void)
{	
	uint8_t r;								//Declare a varible for return use.
	
	//Set Port
	CMPORT &= ~_BV(PS);							//Change PS to 0
	CMPORT &= ~_BV(RD);							//Change RD to 0
		
	my_delay_us(1);
	r = DataPin;
	
	CMPORT |= _BV(RD);								//Change RD to 1
	CMPORT |= _BV(PS);								//Change PS to 1
	return r;
}

/*ReadData
	Read the value from LM629.
*/

static uint8_t ReadData(void)
{
	CheckBusy();
	
	uint8_t r;								//Declare a varible for return use.
	
	//Set Port	
	CMPORT |= _BV(PS);								//Change PS to 1
	CMPORT &= ~_BV(RD);								//Change RD to 0
	
	my_delay_us(1);
	r = DataPin;									//Read Data through DataPin
	
	CMPORT |= _BV(RD);								//Change RD to 1
	CMPORT |= _BV(PS);								//Change PS to 1
	return r;
}

/*WriteCommand
	Write the command to LM629.
*/

static void WriteCommand(uint8_t cmd)
{
	CheckBusy();
	
	//Set Port
	DataDDR = 0xFF;									//Enable Output
	DataPort = cmd;									//Write Command through DataPort
	
	CMPORT &= ~_BV(PS);							//Change PS to 0
	CMPORT &= ~_BV(WR);							//Change WR to 0

	CMPORT |= _BV(WR);								//Change WR to 1
	CMPORT |= _BV(PS);								//Change PS to 1
	DataDDR = 0x00;									//Disable Output
}

/*WriteData
	Write the data to LM629.
*/
static void WriteData(uint8_t data)
{
	CheckBusy();
	// ^^^ .. ????? write data then set WR=0 .. ??
	DataDDR = 0xFF;									//EnableOutput
	DataPort = data;				// Write Data through DataPort
	CMPORT |= _BV(PS);				// Change PS to 1
	CMPORT &= ~_BV(WR);				// Change WR to 0
	
	CMPORT |= _BV(WR);				// Change WR to 1
	CMPORT |= _BV(PS);				// Change PS to 1
	DataDDR = 0x00;									//Disable Output
}

static inline uint32_t U8ToSignedLong(const uint8_t * data)
{
	return ((uint32_t) data[0] | ((uint32_t) data[1] << 8) | ((uint32_t) data[2] << 16) | ((uint32_t) data[3] << 24));
}

// Set corresponding PORT and DDR
// Reset all LM629 chip

void lm629_reset_pe(const uint8_t chip)
{
	SelectChip(chip);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~(_BV(5) | _BV(6)));
	my_delay_ms(5);
}

void lm629_abs_pos_start_done(const uint8_t chip, const uint32_t vel, const int32_t pos)
{
	lm629_abs_position_start(chip, vel, pos);
	SelectChip(chip);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~_BV(2));
	my_delay_ms(2);
	while (!lm629_pos_done(chip))
	{
		my_delay_ms(10);
	}
}

//motion 0 straight
//motion 1 left
//motion 2 right

void lm629_abs_pos_start_done_twin(const uint8_t motion, const uint32_t vel, const int32_t pos)
{
	if (motion==0)
	{
		lm629_abs_position_start(0, vel, pos*(-1));
		lm629_abs_position_start(1, vel, pos);
	}
	else if (motion==1)
	{
		lm629_abs_position_start(0, vel, pos);
		lm629_abs_position_start(1, vel, pos);
	}
	else
	{
		lm629_abs_position_start(0, vel, pos*(-1));
		lm629_abs_position_start(1, vel, pos*(-1));
	}
	SelectChip(0);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~_BV(2));
	my_delay_ms(2);
	
	SelectChip(1);
	WriteCommand(LM629_RSTI);
	WriteData(0x00);
	WriteData(~_BV(2));
	my_delay_ms(2);
	
	while (!lm629_pos_done(0) && !lm629_pos_done(1))
	{
		my_delay_ms(10);
	}
	
	lm629_stop_abruplty(0);
	lm629_stop_abruplty(1);
}

void lm629_init (void) {
	//Set Port
	CMDDR |= _BV(RD) | _BV(PS) | _BV(WR) | _BV(RST);
	CMPORT |= _BV(RD) | _BV(PS) | _BV(WR) | _BV(RST);
	
	CSDDR |= _BV(CS0) | _BV(CS1) | _BV(CS2) | _BV(CS3) | _BV(CS4);
	DataDDR = 0x00;
	
	// Strobe reset, reset all chip
	CMPORT = CMPORT & ~_BV(RST);
	my_delay_ms(1);
	CMPORT = CMPORT | _BV(RST);
	my_delay_ms(5);
				
	for (uint8_t i = 0; i < 5; i++)
	{
		SelectChip(i);
		uint8_t status;
		status = ReadStatus();
		if (status == 0x84 || status == 0xC4)
		{
			WriteCommand(LM629_RSTI);			// RSTI: Reset Interrupt
			WriteData(0x00);					
			WriteData(0x00);					// 0 to resets Interrupt. Reset all
			my_delay_ms(1);
		}
		status = ReadStatus();
		if (status == 0x80 || status == 0xC0)
		{
			lm629_chip_available |= _BV(i);
			
			WriteCommand(LM629_MSKI);			// MSKI: Mask Interrupts
			WriteData(0x00);
			WriteData(0x7E);
		}
	}
	
	lm629_flag = 0;
}


uint8_t lm629_pos_done(const uint8_t chip) 
{
	if (!(lm629_chip_available & _BV(chip))) return 1;
	SelectChip(chip);
	return (lm629_flag_ch[chip] & LM629_TRAJ_COMPLETION && (ReadStatus() & _BV(TrajCompleteStatus)));

}

uint8_t lm629_is_pe(const uint8_t chip) 
{
	if (!(lm629_chip_available & _BV(chip))) return 1;
	SelectChip(chip);
	return (lm629_flag_ch[chip] & LM629_POSITION_ERROR && (ReadStatus() & _BV(PositionError)));

}

uint8_t lm629_bp_reach(const uint8_t chip)
{
	if (!(lm629_chip_available & _BV(chip))) return 1;
	SelectChip(chip);
	return (lm629_flag_ch[chip] & LM629_BREAK_POINT && (ReadStatus() & _BV(BreakPoint)));

}

// Check traj completion
//   Send lm629_position_done to UART1 and clear the flag if done
/*
void lm629_check_flag (void) {
	for (uint8_t i = 0; i < 4; i++){	// ^^^ total : 5 LM629
		if (lm629_flag & _BV(i))
		{
			SelectChip(i);
			uint8_t status = ReadStatus();
			// ^^^ if the chip is playing with POS mode and it has already finsihed. then...
			if ( (lm629_flag_ch[i] & LM629_TRAJ_COMPLETION) && (status & _BV(TrajCompleteStatus)) )
			{
				uint8_t buf[2] = {CMD_lm629_position_done, i};
				// ^^^ lm629_flag_ch[i] & ~LM629_MONITOR_FLAG -> extract the comm_id
				// ^^^ lm629_flag_ch[i] & ~0xF0 -> & ox0F -->> 0x0?.. where "?" == comm_id
				comm_write(lm629_flag_ch[i] & ~LM629_MONITOR_FLAG, buf, 2);
				WriteCommand(0x1D);	// RSTI: Reset Interrupt
				WriteData(0x00);
				WriteData(~_BV(TrajCompleteStatus));	// Reset Trajectory-Complete flag
				// ^^^ clear the flag from the monitor var
				lm629_flag_ch[i] &= ~LM629_TRAJ_COMPLETION;
			}
			// ^^^ if position error.. then...
			if ( (lm629_flag_ch[i] & LM629_POSITION_ERROR) && (status & _BV(PositionError)) )
			{
				uint8_t buf[2] = {CMD_lm629_report_pe, i};
				comm_write(lm629_flag_ch[i] & ~LM629_MONITOR_FLAG, buf, 2);
				WriteCommand(0x1D);	// RSTI: Reset Interrupt
				WriteData(0x00);
				WriteData(~_BV(PositionError));	// Reset Position Error flag
				lm629_flag_ch[i] &= ~LM629_POSITION_ERROR;
			}
			// ^^^ if reach breakpoint... then.
			if ( (lm629_flag_ch[i] & LM629_BREAK_POINT) && (status & _BV(BreakPoint)) )
			{
				uint8_t buf[2] = {CMD_lm629_report_bp, i};
				comm_write(lm629_flag_ch[i] & ~LM629_MONITOR_FLAG, buf, 2);
				WriteCommand(0x1D);	// RSTI: Reset Interrupt
				WriteData(0x00);
				WriteData(~_BV(BreakPoint));	// Reset BreakPoint flag
				lm629_flag_ch[i] &= ~LM629_BREAK_POINT;
			}
			if ((lm629_flag_ch[i] & LM629_MONITOR_FLAG) == 0) lm629_flag &= ~_BV(i);
		}
	}
}*/





void lm629_velocity_start (const uint8_t chip,int32_t vel) {
	
	if (! (lm629_chip_available & _BV(chip)) ) return;

	SelectChip(chip);
	
	// If position_start is not done yet...
	if ( lm629_flag & _BV(chip) )
	{	

		lm629_flag &= ~_BV(chip);
		
		WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
		WriteData(0x00);
		// ^^^ TragCompleteStatus = 0x02
		// ^^^ reset the corr. bit
		WriteData(~_BV(TrajCompleteStatus));	// Reset Trajectory-Complete flag
	}
	
	// Load Traj.
	WriteCommand(LM629_LTRJ);
	
	
	// ^^^ determining forward velocity or velocity
	
	if(vel<0)
	{
		vel = -vel;
		WriteData(0x08);
	} else
	{
		WriteData(0x18);
	}
	WriteData(0x08);
	
	WriteData((uint8_t) (vel>>24));
	WriteData((uint8_t) (vel>>16));
	WriteData((uint8_t) (vel>>8));
	WriteData((uint8_t) vel);
	
	WriteCommand(LM629_STT);	// ^^^ LM629_STT
}

// lm629_position_start
// u8 id, s32 abs velocity, s32 rel position
// 
// Trajectory control: Stop abruptly then set to position mode, load absolute velocity and relative position
//   start motion after that
// Set the trag_completion flag
void lm629_rel_position_start (const uint8_t chip,const uint32_t vel,const int32_t pos) {
	if ( ! (lm629_chip_available & _BV(chip)) ) return;
	
	lm629_flag |= _BV(chip);
	// ^^^ LM629_MONITOR_FLAG = 0xF0
	// ^^^ (1) clear lm629_flag_ch[x] [the lowset bits] (2) set LM629_TRAJ_COMPLETITION | COMM_ID
	lm629_flag_ch[chip] |= LM629_TRAJ_COMPLETION;

	SelectChip(chip);
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);		
	WriteData(0x00);
	WriteCommand(LM629_STT); // ^^^ STOP MOTOR
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x00);
	WriteData(0x0B);
		
	WriteData((uint8_t) (vel>>24));
	WriteData((uint8_t) (vel>>16));
	WriteData((uint8_t) (vel>>8));
	WriteData((uint8_t) vel);
		
	WriteData((uint8_t) (pos>>24));
	WriteData((uint8_t) (pos>>16));
	WriteData((uint8_t) (pos>>8));
	WriteData((uint8_t) pos);
	
	WriteCommand(LM629_STT);
}

// lm629_abs_position_start
// u8 id, s32 abs velocity, s32 abs position
// 
// Trajectory control: Stop abruptly then set to position mode, load absolute velocity and relative position
//   start motion after that
// Set the trag_completion flag
void lm629_abs_position_start (const uint8_t chip,const uint32_t vel,const int32_t pos) {
	if ( ! (lm629_chip_available & _BV(chip)) ) return;
	
	lm629_flag |= _BV(chip);
	lm629_flag_ch[chip] |= LM629_TRAJ_COMPLETION;

	SelectChip(chip);
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);		
	WriteData(0x00);
	WriteCommand(LM629_STT); // ^^^ STOP MOTOR
		
	WriteCommand(LM629_LTRJ);
	WriteData(0x00);
	WriteData(0x0A);
		
	WriteData((uint8_t) (vel>>24));
	WriteData((uint8_t) (vel>>16));
	WriteData((uint8_t) (vel>>8));
	WriteData((uint8_t) vel);
		
	WriteData((uint8_t) (pos>>24));
	WriteData((uint8_t) (pos>>16));
	WriteData((uint8_t) (pos>>8));
	WriteData((uint8_t) pos);
	
	WriteCommand(LM629_STT);
}

void lm629_read_pos_twin (int32_t * pos0,int32_t * pos1)
{
	if (! (lm629_chip_available & (_BV(1) | _BV(0) ) )) return;
	uint8_t buf[4];
	
	SelectChip(0);
	WriteCommand(LM629_RDRP);		// ^^ 0A read real position
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos0 = U8ToSignedLong(buf);
	
	SelectChip(1);
	WriteCommand(LM629_RDRP);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos1 = U8ToSignedLong(buf);

}



void lm629_read_pos_twin_home (int32_t * pos0,int32_t * pos1)
{
	if (! ((lm629_chip_available & _BV(0)) && (lm629_chip_available & _BV(1)))) return;
	uint8_t buf[4];
	
	SelectChip(0);
	WriteCommand(LM629_RDRP);		// ^^ 0A read real position
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos0 = U8ToSignedLong(buf);
	WriteCommand(LM629_DFH); // DFH: redefines HOME
	
	SelectChip(1);
	WriteCommand(LM629_RDRP);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	*pos1 = U8ToSignedLong(buf);

	WriteCommand(LM629_DFH); // DFH: redefines HOME	
}


void lm629_velocity_twin_start (int32_t vel1, int32_t vel2)
{
	if (! ((lm629_chip_available & _BV(0)) && (lm629_chip_available & _BV(1)))) return;

		
	SelectChip(0);
	if ( lm629_flag & _BV(0) )
	{	

		lm629_flag &= ~_BV(0);
		
		WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
		WriteData(0x00);
		// ^^^ TragCompleteStatus = 0x02
		// ^^^ reset the corr. bit
		WriteData(~_BV(TrajCompleteStatus));	// Reset Trajectory-Complete flag
	}
	WriteCommand(0x1F);	// LTRJ, Velocity
	
	if (vel1 < 0) { vel1 = -vel1; WriteData(0x08); } else { WriteData(0x18); }
	WriteData(0x08);
	WriteData((uint8_t) (vel1>>24));
	WriteData((uint8_t) (vel1>>16));
	WriteData((uint8_t) (vel1>>8));
	WriteData((uint8_t) (vel1));
//	WriteCommand(0x1F);

		
	SelectChip(1);
	if ( lm629_flag & _BV(1) )
	{	

		lm629_flag &= ~_BV(1);
		
		WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
		WriteData(0x00);
		// ^^^ TragCompleteStatus = 0x02
		// ^^^ reset the corr. bit
		WriteData(~_BV(TrajCompleteStatus));	// Reset Trajectory-Complete flag
	}
	WriteCommand(LM629_LTRJ);	// LTRJ, Velocity

	if (vel2 > 0) { WriteData(0x08); } else { vel2 = -vel2; WriteData(0x18); } 
	WriteData(0x08);
	WriteData((uint8_t) (vel2>>24));
	WriteData((uint8_t) (vel2>>16));
	WriteData((uint8_t) (vel2>>8));
	WriteData((uint8_t) (vel2));
//	WriteCommand(0x1F);
		
	SelectChip(0);
	WriteCommand(LM629_STT);
		
	SelectChip(1);
	WriteCommand(LM629_STT);
	
}


void lm629_position_twin_start (const uint32_t vel1,const uint32_t vel2, int32_t pos)
{
	if (! ((lm629_chip_available & _BV(0)) && (lm629_chip_available & _BV(1)))) return;
	
	lm629_flag |= _BV(0) | _BV(1);
	lm629_flag_ch[0] |= LM629_TRAJ_COMPLETION;
	lm629_flag_ch[1] |= LM629_TRAJ_COMPLETION;

				
	SelectChip(1);
	WriteCommand(LM629_LTRJ);	// ^^^ LTRJ
	WriteData(0x09);
	WriteData(0x00);
	WriteCommand(LM629_STT);	// ^^^ STT ..(TURN OFF)
	
	SelectChip(0);
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);
	WriteData(0x00);
	WriteCommand(LM629_STT);
	
	WriteCommand(LM629_LTRJ);	// LTRJ, Velocity + Rel Position
	WriteData(0x00);
	WriteData(0x0B);
	WriteData((uint8_t) (vel1>>24));
	WriteData((uint8_t) (vel1>>16));
	WriteData((uint8_t) (vel1>>8));
	WriteData((uint8_t) vel1);
	WriteData((uint8_t) (pos>>24));
	WriteData((uint8_t) (pos>>16));
	WriteData((uint8_t) (pos>>8));
	WriteData((uint8_t) (pos));
	
	pos = -pos;
	SelectChip(1);
	WriteCommand(0x1F);	// LTRJ, Velocity + Rel Position
	WriteData(0x00);
	WriteData(0x0B);
	WriteData((uint8_t) (vel2>>24));
	WriteData((uint8_t) (vel2>>16));
	WriteData((uint8_t) (vel2>>8));
	WriteData((uint8_t) vel2);
	WriteData((uint8_t) (pos>>24));
	WriteData((uint8_t) (pos>>16));
	WriteData((uint8_t) (pos>>8));
	WriteData((uint8_t) (pos));
	
	SelectChip(0);
	WriteCommand(LM629_STT);
	SelectChip(1);
	WriteCommand(LM629_STT);
	
}

void lm629_set_pe_report (const uint8_t chip,const uint16_t pe)
{
	if ( !(lm629_chip_available & _BV(chip)) ) return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);			// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~_BV(PositionError));	// Reset Trajectory-Complete flag

	WriteCommand(LM629_LPEI);	// LPSI : encoder count
	WriteData((uint8_t) (pe>>8));
	WriteData((uint8_t) pe);
	

	lm629_flag_ch[chip] |= LM629_POSITION_ERROR;
}

void lm629_set_pe_stop (const uint8_t chip,const uint16_t pe)
{
	if ( !(lm629_chip_available & _BV(chip)) ) return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);			// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~_BV(PositionError));	// Reset Trajectory-Complete flag

	WriteCommand(LM629_LPES);	// LPSI : encoder count
	WriteData((uint8_t) (pe>>8));
	WriteData((uint8_t) pe);
	
	lm629_flag_ch[chip] |= LM629_POSITION_ERROR;

	
}


void lm629_set_rel_bp (const uint8_t chip,const int32_t bp)
{
	if ( !(lm629_chip_available & _BV(chip)) ) return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~_BV(BreakPoint));	// Reset BreakPoint flag

	WriteCommand(LM629_SBPR);	// SBPR
	WriteData((uint8_t) (bp>>24));
	WriteData((uint8_t) (bp>>16));
	WriteData((uint8_t) (bp>>8));
	WriteData((uint8_t) bp);
	

	lm629_flag_ch[chip] |= LM629_BREAK_POINT;
}


void lm629_set_abs_bp (const uint8_t chip,const int32_t bp)
{
	if ( !(lm629_chip_available & _BV(chip)) ) return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_RSTI);	// RSTI: Reset Interrupt
	WriteData(0x00);
	WriteData(~_BV(BreakPoint));	// Reset BreakPoint flag

	WriteCommand(LM629_SBPA);	// SBPA
	WriteData((uint8_t) (bp>>24));
	WriteData((uint8_t) (bp>>16));
	WriteData((uint8_t) (bp>>8));
	WriteData((uint8_t) bp);
	

	lm629_flag_ch[chip] |= LM629_BREAK_POINT;
}


void lm629_zero_drive (const uint8_t chip)
{
	if ( !(lm629_chip_available & _BV(chip))) return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_LTRJ);
	WriteData(0x09);	// ^^^ zero drive + velocity mode
	WriteData(0x00);
	
	WriteCommand(LM629_STT);
}


void lm629_stop_abruplty (const uint8_t chip)
{
	if ( !(lm629_chip_available & _BV(chip)) ) return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_LTRJ);
	WriteData(0x02);		// ^^^ stop abruptly
	WriteData(0x00);
	
	WriteCommand(LM629_STT);
}


void lm629_acceleration (const uint8_t chip,const uint32_t acc) {
	if ( !(lm629_chip_available & _BV(chip)) ) return;
	
	SelectChip(chip);

		
	WriteCommand(LM629_LTRJ);
	WriteData(0x00);			// ^^^ an absoulte acc will be loaded
	WriteData(0x20);
	
	WriteData((uint8_t) (acc>>24));
	WriteData((uint8_t) (acc>>16));	
	WriteData((uint8_t) (acc>>8));
	WriteData((uint8_t) acc);
}


int32_t lm629_read_pos (const uint8_t chip) {
	if (! (lm629_chip_available & _BV(chip))) return 0;
	
	uint8_t buf[4];

	SelectChip(chip);
	WriteCommand(LM629_RDRP);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	
	return U8ToSignedLong(buf);
	
	
}


int32_t lm629_read_pos_home (const uint8_t chip) {
	if( ! (lm629_chip_available & _BV(chip)) ) return 0;
	
	
	uint8_t buf[4];
	
	SelectChip(chip);
	WriteCommand(LM629_RDRP);	// ^^^ Read Real Position
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	WriteCommand(LM629_DFH);	// ^^^ DFH
	return U8ToSignedLong(buf);
}


void lm629_set_filter (const uint8_t chip,const uint16_t kp,const uint16_t ki,const uint16_t kd,const uint16_t il) {
	if ( !(lm629_chip_available & _BV(chip)) ) return;
	
	SelectChip(chip);
	
	WriteCommand(LM629_LFIL);	// ^^^ LIFL  : Load Filter Parameters
	//Control Words
	
	WriteData(0x00);
	WriteData(0x0F);
	

	//kp
	WriteData((uint8_t) kp>>8);
	WriteData((uint8_t) kp);
	//ki
	WriteData((uint8_t) ki>>8);
	WriteData((uint8_t) ki);
	//kd
	WriteData((uint8_t) kd>>8);
	WriteData((uint8_t) kd);

	//il
	WriteData((uint8_t) il>>8);
	WriteData((uint8_t) il);

	
	//Update Filter	
	WriteCommand(LM629_UDF);	/// ^^^ UDF : Update Filter
}




void lm629_define_home (const uint8_t chip)
{
	if( ! (lm629_chip_available & chip) ) return;
	
	SelectChip(chip);
	WriteCommand(0x02);	
}

inline void lm629_disable_output(void)
{
	lm629_zero_drive(0);
	lm629_zero_drive(1);
	lm629_zero_drive(2);
	lm629_zero_drive(3);
	lm629_zero_drive(4);
}

int32_t lm629_read_des_vel (const uint8_t chip) {
	if (! (lm629_chip_available & _BV(chip))) return 0;
	
	uint8_t buf[4];

	SelectChip(chip);
	WriteCommand(LM629_RDDV);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = ReadData();
	buf[0] = ReadData();
	
	return U8ToSignedLong(buf);
}

int32_t lm629_read_rel_vel (const uint8_t chip) {
	if (! (lm629_chip_available & _BV(chip))) return 0;
	
	uint8_t buf[4];

	SelectChip(chip);
	WriteCommand(LM629_RDRV);
	buf[3] = ReadData();
	buf[2] = ReadData();
	buf[1] = 0;
	buf[0] = 0;
	
	return U8ToSignedLong(buf);
}