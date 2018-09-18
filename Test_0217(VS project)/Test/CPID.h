/*
* CPID.h
*
*  Created on: 2016��5��11��
*      Author: geolee
*/

#ifndef CPID_H_
#define CPID_H_

#include<stdlib.h>
#include<cmath>
#include<math.h>
#include<cstdlib>
typedef struct
{
	float pv; //process value ������
	float sp; //set point   �趨ֵ
	float ferror;//ferror=sp-pv;
	float integral;//������ ���� ƫ���ۼ�ֵ
	float pgain;//P����
	float igain;//I����
	float dgain;//D����
	float deadband;//������ֵ
	float divide;//���ַ�����ֵ
	float last_error;//��һ�����
}PID;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*PID methods*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//1. ��ʼ��pid�ṹ��������趨����ֵ���趨ֵ
void pid_init(PID *pid, float process_point, float set_point);

//2. ����pid�ṹ���еı���������΢����ϵ��������ֵ��
void pid_tune(PID *pid, float p_gain, float i_gain, float d_gain, float dead_band, float divide, float integral_val);

//3. ���õ�ǰpid�ṹ���е�ǰ�������ʼֵ����һʱ������0
void pid_setinteg(PID *pid, float new_integ);

//4. ��ͻȻ�ı��趨ֵʱ�������������󣬽������Ŷ�����������������ʵ��ƽ˳�Ŷ����ڵ��øú���֮ǰ��Ҫ�ȸ���PVֵ
void pid_bumpless(PID *pid);

//5. ������ʹ��λ��ʽPID���㷽ʽ�����Ҳ�ȡ�˻��ֱ����������㣬PID����
float pid_calc(PID *pid, float cmd_rate, float rate, float K_rate, float Maxoutput);



#endif /* CPID_H_ */
