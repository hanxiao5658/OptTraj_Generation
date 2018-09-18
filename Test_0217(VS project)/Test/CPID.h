/*
* CPID.h
*
*  Created on: 2016年5月11日
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
	float pv; //process value 反馈量
	float sp; //set point   设定值
	float ferror;//ferror=sp-pv;
	float integral;//积分项 －－ 偏差累计值
	float pgain;//P增益
	float igain;//I增益
	float dgain;//D增益
	float deadband;//死区阈值
	float divide;//积分分离阈值
	float last_error;//上一次误差
}PID;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*PID methods*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//1. 初始化pid结构体变量，设定过程值和设定值
void pid_init(PID *pid, float process_point, float set_point);

//2. 设置pid结构体中的比例项、积分项、微分项系数，死区值，
void pid_tune(PID *pid, float p_gain, float i_gain, float d_gain, float dead_band, float divide, float integral_val);

//3. 设置当前pid结构体中当前积分项初始值，上一时刻误差归0
void pid_setinteg(PID *pid, float new_integ);

//4. 当突然改变设定值时，或重新启动后，将引起扰动输出。这个函数将能实现平顺扰动，在调用该函数之前需要先更新PV值
void pid_bumpless(PID *pid);

//5. 本函数使用位置式PID计算方式，并且采取了积分饱和限制运算，PID计算
float pid_calc(PID *pid, float cmd_rate, float rate, float K_rate, float Maxoutput);



#endif /* CPID_H_ */
