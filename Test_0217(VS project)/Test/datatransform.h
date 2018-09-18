#ifndef _DATATRANSFORM_H
#define _DATATRANSFORM_H
#include <math.h>


#define uchar unsigned char
#define uint  unsigned int
extern const float Pi;

float deg2rad(float u_deg);
float rad2deg(float u_rad);
int hex2I(uchar data1, uchar data2);
int hex4I(uchar data1, uchar data2, uchar data3, uchar data4);
float I2float(uchar data1, uchar data2);
float DI2float(uchar data1, uchar data2, uchar data3, uchar data4);
void nav_calculate(uchar *nav_data);

void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);

void servo_unit(int *servo, float *servo_float);

#endif