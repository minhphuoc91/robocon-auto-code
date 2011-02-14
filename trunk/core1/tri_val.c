#include "tri_val.h"

#define cos_mag_factor 10000
int16_t tan_val[91] = //scale 100
{
	0,
	1,
	3,
	5,
	6,
	8,
	10,
	12,
	14,
	15,
	17,
	19,
	21,
	23,
	24,
	26,
	28,
	30,
	32,
	34,
	36,
	38,
	40,
	42,
	44,
	46,
	48,
	50,
	53,
	55,
	57,
	60,
	62,
	64,
	67,
	70,
	72,
	75,
	78,
	80,
	83,
	86,
	90,
	93,
	96,
	100,
	103,
	107,
	111,
	115,
	119,
	123,
	127,
	132,
	137,
	142,
	148,
	153,
	160,
	166,
	173,
	180,
	188,
	196,
	205,
	214,
	224,
	235,
	247,
	260,
	274,
	290,
	307,
	327,
	348,
	373,
	401,
	433,
	470,
	514,
	567,
	631,
	711,
	814,
	951,
	1143,
	1430,
	1908,
	2863,
	5728
};

int16_t cos_val[91] =
{
    10000,
    9998,
    9994,
    9986,
    9976,
    9962,
    9945,
    9925,
    9903,
    9877,
    9848,
    9816,
    9781,
    9744,
    9703,
    9659,
    9613,
    9563,
    9511,
    9455,
    9397,
    9336,
    9272,
    9205,
    9135,
    9063,
    8988,
    8910,
    8829,
    8746,
    8660,
    8572,
    8480,
    8387,
    8290,
    8192,
    8090,
    7986,
    7880,
    7771,
    7660,
    7547,
    7431,
    7314,
    7193,
    7071,
    6947,
    6820,
    6691,
    6561,
    6428,
    6293,
    6157,
    6018,
    5878,
    5736,
    5592,
    5446,
    5299,
    5150,
    5000,
    4848,
    4695,
    4540,
    4384,
    4226,
    4067,
    3907,
    3746,
    3584,
    3420,
    3256,
    3090,
    2924,
    2756,
    2588,
    2419,
    2250,
    2079,
    1908,
    1736,
    1564,
    1392,
    1219,
    1045,
    872,
    698,
    523,
    349,
    175,
    0
};


int16_t my_angle(int16_t x) {	// make  -90< x < 90
    if (x < 0)
        x += 360;

    return (x % 360);
}

int16_t int_cos(int16_t x) {
    if (x < 360)			// to ensure x > 0
        x += 360;
    x %= 360;				// to ensure x < 360
    uint8_t quad = x / 90;

    switch (quad % 4) {
    case 0:
        return cos_val[x % 90];
        break;
    case 1:
        return -cos_val[90 - (x % 90)];
        break;
    case 2:
        return -cos_val[x % 90];
        break;
    case 3:
        return cos_val[90 - (x % 90)];
        break;
    default:
        return 0;
    }
}

int16_t int_sin(int16_t x) {
    if (x < 360)			// to ensure x > 0
        x += 360;
		x %= 360;				// to ensure x < 360
    uint8_t quad = x / 90 + 3;

    switch (quad % 4) {
    case 0:
        return cos_val[x % 90];
        break;
    case 1:
        return -cos_val[90 - (x % 90)];
        break;
    case 2:
        return -cos_val[x % 90];
        break;
    case 3:
        return cos_val[90 - (x % 90)];
        break;
    default:
        return 0;
    }
}



int16_t int_tan(int16_t x){
	if (x >=0){
		if (x<180) x += 180;
		x = x%180;
		if (x == 90) return -1;
		uint8_t quad = x / 90;
		switch(quad){
			case 0: return tan_val[x];
			case 1: return -tan_val[180-x];
		}
	}
	else{
		x= -x;
		if (x<180) x += 180;
		x = x%180;
		if (x == 90) return -1;
		uint8_t quad = x / 90;
		switch(quad){
			case 0: return -tan_val[x];
			case 1: return tan_val[180-x];
		}
	}
	return 361;
}


int16_t angle_sub (int16_t target_angle, int16_t curr_angle)
{
    int16_t diff_angle = target_angle - curr_angle;
    if (abs(diff_angle) > 180)
    {
        if (diff_angle > 0)
            diff_angle -= 360;
        else
            diff_angle += 360;
    }
    return diff_angle;
}


int16_t int_arc_sin(int16_t sin_val)//scaled by 10000
{
	int16_t angle=0;
	int32_t pre_sin=0;
	int32_t cur_sin = 0;

	if (sin_val>=0){
		while(angle<90){
			pre_sin = cur_sin;
			cur_sin = int_sin(angle);
			if(sin_val<=cur_sin && sin_val>=pre_sin) break;
			angle++;
		}
	}
	else{
		while(angle>-90){
			pre_sin = cur_sin;
			cur_sin = int_sin(angle);
			if(sin_val>=cur_sin && sin_val<=pre_sin) break;
			angle--;
		}
	}
	return angle;

}


uint32_t Abs(int32_t M){
	if(M<0) M = -M;
	return M;
}

int16_t int_arc_tan_ver2(int32_t y, int32_t x){
	if(x == 0){
		if (y<0) return -90;
		else if(y == 0) return 0;
		else return 90;
	}
	else{
		return int_arc_tan((int32_t)100*y/x);
	}
}

int16_t int_arc_tan(int32_t tan_val)//scaled by 100
{
	int16_t angle=0;
	int16_t pre_tan=0;
	int16_t cur_tan = 0;

	if (tan_val>=0){
		while(angle<90){
			pre_tan = cur_tan;
			cur_tan = int_tan(angle);
			if(tan_val<=cur_tan && tan_val>=pre_tan)break;
			angle++;
		}
	}
	else{
		while(angle>=-89){
			pre_tan = cur_tan;
			cur_tan = int_tan(angle);
			if(tan_val>=cur_tan && tan_val<=pre_tan)break;
			angle--;
		}
	}
	return angle;

}

int16_t Sqrt(int32_t W)
{
    uint16_t N, i;
    uint32_t tmp, ttp;
	uint32_t M = Abs(W);
    if (M == 0)           
        return 0;
    N = 0;
    tmp = (M >> 30);         
    M <<= 2;
    if (tmp > 1)  
    {
       N ++;      
       tmp -= N;
    }

    for (i=15; i>0; i--) 
    {
        N <<= 1;
        tmp <<= 2;
        tmp += (M >> 30);
        ttp = N;
        ttp = (ttp<<1)+1;
        M <<= 2;
        if (tmp >= ttp) 
        {
           tmp -= ttp;
           N ++;
        }
    }
    return N;
}