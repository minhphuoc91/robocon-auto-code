// for 360 Gyro
#include "PID.h"

int32_t KP=10000;
int32_t KI=0;
int32_t KD=5000;
int32_t IL=4096;
int32_t UL=900000;

int32_t PID_desire_angle;
int32_t PID_start_angle;
int32_t PID_start_X;
int32_t PID_start_Y;
int32_t PID_target_X;
int32_t PID_target_Y;
int32_t PID_cos;
int32_t PID_sin;

int32_t PID_center_X;
int32_t PID_center_Y;
int32_t PID_radius;

int8_t PID_dir=0;

volatile int32_t PID_error_position;
volatile int32_t PID_error;
volatile int32_t PID_prev_error;	
volatile int32_t PID_integration;
volatile int32_t PID_u;   //adjust velocity angle

uint8_t PID_status;


void PID_calculation_u(void)
{	
	static int32_t PID_current_angle;    //chu zao yin
	static int32_t PID_current_X;
	static int32_t PID_current_Y;
	static int16_t PID_angle;
	static int32_t d = 0;
	static int16_t diff_x=0, diff_y=0;
	static int16_t sita=0, arfa=0;
	static int8_t qua = 0;
	
	if(PID_status)
	{
		PID_current_angle = position_get_angle();//modified
		PID_current_angle = (PID_current_angle>180)?(PID_current_angle-360):PID_current_angle;
		PID_current_X = position_get_X();
		PID_current_Y = position_get_Y();
			
		if(PID_status&_BV(0))
		{
			if (PID_status&_BV(2)) {//arc mode
				diff_x = PID_current_X - PID_center_X;
				diff_y = PID_current_Y - PID_center_Y;			
				if (PID_dir==1) {
					if(diff_x<0 && diff_y >= 0){ //0-90
						sita = int_arc_tan_ver2(diff_y,-diff_x);
					}
					else if(diff_x>=0 && diff_y>=0){//90-180
						sita = int_arc_tan_ver2(diff_x,diff_y) + 90;
					}
					else if(diff_x>=0 && diff_y<0){//-180 - -90
						sita = int_arc_tan_ver2(diff_x,diff_y) - 90;
					}
					else if(diff_x<0 && diff_y<0){ //-90 - 0
						sita = int_arc_tan_ver2(-diff_y,diff_x);
					}
				}
				else if (PID_dir==-1) {
					if(diff_x>0 && diff_y>=0){ //0-90
						sita = int_arc_tan_ver2(diff_y,diff_x);
					}
					else if(diff_x<=0 && diff_y>=0){//90-180
						sita = int_arc_tan_ver2(-diff_x,diff_y) + 90;
					}
					else if(diff_x<=0 && diff_y<0){//-180 - -90
						sita = int_arc_tan_ver2(-diff_x,diff_y) - 90;
					}
					else if(diff_x>0 && diff_y<0){ //-90 - 0
						sita = int_arc_tan_ver2(diff_y,diff_x);
					}		
				}
				
				qua = sita/45;
				switch(qua){
					case 0:
					case 3:
					case -3:
					case 4:
					case -4:
						d = PID_radius + (int32_t)diff_x*10000/int_cos(sita)*(int32_t)PID_dir;
						break;
					case 1:
					case 2:
					case -1:
					case -2:
						d = PID_radius - (int32_t)diff_y*10000/int_sin(sita);
						break;
				}
				
				PID_desire_angle=PID_start_angle+sita*(int32_t)PID_dir;
				
				arfa = int_arc_sin(10000*d/RAD);
				
				PID_error = arfa*(int32_t)PID_dir + PID_current_angle - sita*(int32_t)PID_dir;					
			}
			else {//line mode
				PID_error_position = ((PID_current_X - PID_start_X)*PID_cos - (PID_current_Y - PID_start_Y)*PID_sin)/10000;
				PID_angle = int_arc_sin(10000*PID_error_position/RAD);
				
				//To fix the angle difference problem near negative y-axis(jumping from 17x to -17x)
				int16_t tmp=PID_current_angle - PID_desire_angle;
				if (tmp>=180)
					tmp -= 360;
				else if (tmp<=-180)
					tmp += 360;
					
				PID_error= tmp + PID_angle;			
			}

			PID_integration = KI * PID_error + PID_integration;

			//Integration error limit
			if(PID_integration >= IL)
				PID_integration = IL;
			
			PID_u =KP * PID_prev_error + PID_integration +KD * (PID_error - PID_prev_error);			
			//Error limit
			if(PID_u > UL)
			PID_u = UL;
			else if(PID_u < - UL)
			PID_u = -UL;
			
			PID_prev_error = PID_error;
		}
		
		lcd_clear();
		lcd_printxys(0,0,"X:%d",PID_current_X);
		lcd_printxys(10,0,"Y:%d",PID_current_Y);
		lcd_printxys(0,1,"A:%d",arfa);
		lcd_printxys(0,2,"u: %ld",PID_u);
		lcd_printxys(10,2,"pe: %d",PID_error);
	}
}

void PID_velocity_twin_start(int32_t vel1, int32_t vel2)
{
	lm629_velocity_twin_start(-vel1+PID_u,-vel2-PID_u);  //may need change
}

void PID_set_target(void)
{
	PID_disable();
	
	PID_prev_error =0;
	PID_error=0;
	PID_integration = 0;
	
	PID_u = 0;
	
	PID_desire_angle = position_get_angle();
	PID_start_X = position_get_X();
	PID_start_Y = position_get_Y();
	
	PID_cos = int_cos(PID_desire_angle);
	PID_sin = int_sin(PID_desire_angle);
	
	PID_set_status(1);
}

void PID_set_target_angle(int16_t angle)
{
	PID_disable();
	
	PID_prev_error =0;
	PID_error =0;
	PID_integration = 0;
	
	PID_u = 0;
	
	PID_desire_angle = angle;
	PID_start_X = position_get_X();
	PID_start_Y = position_get_Y();
	
	PID_cos = int_cos(PID_desire_angle);
	PID_sin = int_sin(PID_desire_angle);
	
	PID_set_status(1);
}

void PID_set_target_distance(int32_t pos)
{
	PID_disable();
	
	PID_prev_error =0;
	PID_error =0;
	PID_integration = 0;
	
	PID_u = 0;
	
	PID_desire_angle = position_get_angle();//modified
	PID_start_X = position_get_X();
	PID_start_Y = position_get_Y();
	
	PID_cos = int_cos(PID_desire_angle);
	PID_sin = int_sin(PID_desire_angle);
	PID_target_X = PID_start_X + pos*int_sin(PID_desire_angle)/10000;
	PID_target_Y = PID_start_Y + pos*int_cos(PID_desire_angle)/10000;
	
	if(pos < 0)
	PID_desire_angle = (PID_desire_angle+180)%360;
	
	PID_set_status(3);
}

void PID_set_target_XY(int32_t X,int32_t Y,int32_t SX,int32_t SY,int8_t dir)
{	
	PID_disable();
	PID_prev_error = 0;
	PID_error= 0;
	PID_integration = 0;
	
	PID_u = 0;
	
	int32_t diff_X=0,diff_Y=0;
	uint16_t dis=0;
	
	PID_start_X = SX;
	PID_start_Y = SY;
	PID_target_X = X;
	PID_target_Y = Y;
	
	diff_X = X-PID_start_X;
	diff_Y = Y-PID_start_Y;
	
	dis= Sqrt(diff_X * diff_X + diff_Y * diff_Y);
	

	PID_desire_angle = int_arc_tan(diff_X *100/diff_Y);
	
	if(dir == 0) PID_desire_angle = (PID_desire_angle+180)%360;
	
	PID_cos = diff_Y*10000/dis;
	PID_sin = diff_X*10000/dis;
	
	PID_set_status(3);
}

void PID_set_target_XYAngle(int32_t X,int32_t Y,int32_t angle)
{
	PID_disable();
	
	PID_prev_error =0;
	PID_error =0;
	PID_integration = 0;
	
	PID_u = 0;
	
	PID_start_X = position_get_X();
	PID_start_Y = position_get_Y();
	PID_target_X = X;
	PID_target_Y = Y;
	PID_desire_angle = angle;
	
	PID_cos = int_cos(angle);
	PID_sin = int_sin(angle);
	
	PID_set_status(3);
}

void PID_set_target_XYAngle_back(int32_t X,int32_t Y,int32_t angle)
{
	PID_disable();
	
	PID_prev_error =0;
	PID_error =0;
	PID_integration = 0;
	
	PID_u = 0;
	
	PID_start_X = position_get_X();
	PID_start_Y = position_get_Y();
	PID_target_X = X;
	PID_target_Y = Y;
	PID_desire_angle = angle;
	PID_cos = int_cos((angle + 180)%360);
	PID_sin = int_sin((angle + 180)%360);
	
	PID_set_status(3);
}

void PID_path_arc(int32_t x,int32_t y,int32_t sx, int32_t sy , int32_t sa)
{
	PID_target_X = x;
	PID_target_Y = y;
	PID_start_X = sx;
	PID_start_Y = sy;
	PID_start_angle = sa;
	
	int32_t a = (((x-sx)*int_cos(sa)) - (y-sy)*int_sin(sa))/10000;
	int32_t b = (((x-sx)*int_sin(sa)) + (y-sy)*int_cos(sa))/10000;
	if(b < 0) b = -b; 
	
	if(a >= 0)
		PID_dir = 1; // right
	else
	{
		PID_dir = -1;
		a = -a;
	}

	PID_radius = (b * b / a + a)>>1;
	//int16_t angle = int_arccos((r - a) / r);
	
	PID_center_X = sx + PID_dir * PID_radius * int_cos(sa)/10000;
	PID_center_Y = sy - PID_dir * PID_radius * int_sin(sa)/10000;
	
	PID_status = 7;	
}



int PID_position_done(void)
{
	if(((position_get_X() - PID_target_X)*PID_sin + (position_get_Y() - PID_target_Y)*PID_cos)>=0) 
		return 1;
	else
		return 0;
}

int PID_arc_done(void)
{     
	if(((position_get_X() - PID_target_X)*int_sin(PID_desire_angle) + (position_get_Y() - PID_target_Y)*int_cos(PID_desire_angle)>0)
		&& ((position_get_X() - PID_target_X)*(position_get_X() - PID_target_X)+(position_get_Y() - PID_target_Y)*(position_get_Y() - PID_target_Y)<=50000))
		return 1;
	else
		return 0;
}

int PID_angle_done(void)
{
	if(abs(position_get_angle() - PID_desire_angle)<5)
		return 1;
	else
		return 0;
}

void PID_set_status(uint8_t status)
{
	PID_status = status;
}

void PID_disable(void)
{
	PID_status = 0;
}

void PID_set_filter(int32_t kp,int32_t ki,int32_t kd,int32_t il,int32_t ul)
{
	KP = kp;
	KI = ki;
	KD = kd;
	IL = il;
	UL = ul;
}

/*
int8_t PID_path_polyline(struct PID_path path[],uint8_t num,int32_t speed,int32_t final_angle)
{
	static uint8_t state = 0;
	static uint8_t count = 0;

	num--;
	switch(state)
	{
		case 0 : if(count == num)
				{
					PID_set_filter(10000,0,5000,4096,550000);
					PID_set_target_XYAngle(path[count].x,path[count].y,final_angle);
					state = 2;
				}
				else
				{	
					PID_set_filter(10000,0,5000,4096,900000);
					PID_set_target_XY(path[count].x,path[count].y,1);
					state = 1;
				}
				break;
		case 1 : PID_velocity_twin_start(speed,speed);	
					if(PID_position_done()) 
					{
						state = 0;
						count ++;
					}
					break;
		case 2 : PID_velocity_twin_start(speed/10,speed/10);
					if(PID_position_done())
					{
						state = 0;
						count = 0;
						return 1;
					}
	}
	return 0;
}
*/

/*
int8_t PID_path_polyline_back(struct PID_path path[],uint8_t num,int32_t speed,int32_t final_angle)
{
	static uint8_t state = 0;
	static uint8_t count = 0;
	PID_set_filter(3500,0,2048,4096,550000);
	num--;
	switch(state)
	{
		case 0 : if(count == num)
				{
					PID_set_target_XYAngle_back(path[count].x,path[count].y,final_angle);
					state = 2;
				}
				else
				{
					PID_set_target_XY(path[count].x,path[count].y,0);
					state = 1;
				}
				break;
		case 1 : PID_velocity_twin_start(speed,speed);	
					if(PID_position_done()) 
					{
						state = 0;
						count ++;
					}
					break;
		case 2 : PID_velocity_twin_start(speed/2,speed/2);
					if(PID_position_done())
					{
						state = 0;
						count = 0;
						return 1;
					}
	}
	return 0;
}

*/
void PID_arc_speed(int32_t v[])
{
	if (PID_dir==1) {
		v[0] = (int64_t)(PID_radius + 325) * v[0] / PID_radius; // left
		v[1] = (int64_t)(PID_radius - 325) * v[1] / PID_radius;
	}
	else if (PID_dir==-1) {
		v[0] = -(int64_t)(PID_radius + 325) * v[0] / PID_radius; // left
		v[1] = -(int64_t)(PID_radius - 325) * v[1] / PID_radius;
	}
}
