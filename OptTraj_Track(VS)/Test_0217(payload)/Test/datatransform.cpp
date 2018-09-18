#include "datatransform.h"

float deg2rad(float u_deg)
{
	float out_rad;
	out_rad = u_deg*Pi / 180.0;
	return out_rad;
}
float rad2deg(float u_rad)
{
	float out_deg;
	out_deg = u_rad*180.0 / Pi;
	return out_deg;
}
int hex2I(uchar data1, uchar data2) // 两个Hex组成int，data2是高位,data1是低位 
{
	int temp_int = 0;
	temp_int = data1;
	temp_int = (temp_int << 8) | data2;

	return temp_int;
}

int hex4I(uchar data1, uchar data2, uchar data3, uchar data4) // 四个Hex组成int，data1是高位,data2是低位
{
	int temp_int = 0;
	temp_int = data1;
	temp_int = (temp_int << 8) | data2;
	temp_int = (temp_int << 8) | data3;
	temp_int = (temp_int << 8) | data4;

	return temp_int;
}

float I2float(uchar data1, uchar data2) // data2是高位,data1是低位 （字节转float）
{
	signed char temp;
	float temp_float;
	temp = data2;//data2是高位
	temp = (temp << 8) | data1;//data1是低位
	if (temp & 0x00008000)//如果是负数，则将高16位全部置1
	{
		temp = temp | 0xffff0000;
	}
	temp_float = temp;
	return temp_float;
}
float DI2float(uchar data1, uchar data2, uchar data3, uchar data4)//data4是高位，data1是低位 （双字节转float）
{

	int buf = 0;
	float temp_float;

	buf = buf | data1;
	buf = (buf << 8) | data2;
	buf = (buf << 8) | data3;
	buf = (buf << 8) | data4;

	temp_float = *((float *)&buf);
	return temp_float;
}


// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
	int i = 0, j = len - 1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
	int i = 0;
	bool isNeg = false;
	if (x<0)
	{
		isNeg = true;
		x = -x;
	}
	while (x)
	{
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
	{
		str[i++] = '0';
	}

	if (isNeg)
		str[i++] = '-';
	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
	int i = 0;
	bool isNeg = false;
	if (n<0)
	{
		isNeg = true;
	}
	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;
	if (fpart<0)
		fpart = -fpart;
	// convert integer part to string
	if (ipart == 0 && isNeg == true)
	{
		i = intToStr(ipart, res, 2);
		res[0] = '-';
	}
	else
	{
		i = intToStr(ipart, res, 1);
	}
	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.';  // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10.0, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}