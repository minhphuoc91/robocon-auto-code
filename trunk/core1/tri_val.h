#ifndef TRI_VAL_H
#define TRI_VAL_H

#include <inttypes.h>
#include <stdlib.h>
#include <math.h>

extern int16_t cos_val[91];
extern int16_t cos_square_val[91];

int16_t my_angle(int16_t);
int16_t int_cos(int16_t);
int16_t int_sin(int16_t);
int16_t int_tan(int16_t x);
int16_t angle_sub (int16_t target_angle, int16_t curr_angle);

int16_t int_arc_tan_ver2(int32_t y, int32_t x);
int16_t int_arc_tan(int32_t tan_val);
int16_t int_arc_sin_square(int16_t sin_val);
int16_t int_arc_sin(int16_t sin_val);
int16_t int_sin_square(int16_t x);
int16_t int_cos_square(int16_t x);
int16_t Sqrt(int32_t M);
uint32_t Abs(int32_t M);
#endif
