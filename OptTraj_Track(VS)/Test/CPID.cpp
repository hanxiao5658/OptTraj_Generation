/*
* VPID.cpp
*
*  Created on: 2016年5月11日
*      Author: geolee
*/
#include"CPID.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*PID methods*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pid_init(PID *pid, float process_point, float set_point)
{
	pid->pv = process_point;
	pid->sp = set_point;
}
void pid_tune(PID *pid, float p_gain, float i_gain, float d_gain, float dead_band, float divide, float integral_val)
{
	pid->pgain = p_gain;
	pid->igain = i_gain;
	pid->dgain = d_gain;
	pid->deadband = dead_band;//死区
	pid->divide = divide;//积分分离变量
	pid->integral = integral_val;//积分项
	pid->last_error = 0;
}
void pid_setinteg(PID *pid, float new_integ)//重新设置积分项,上次误差清零
{
	pid->integral = new_integ;
	pid->last_error = 0;
}

void pid_bumpless(PID *pid)
{
	pid->last_error = (pid->sp) - (pid->pv);  //设定值与反馈值偏差
}

float pid_calc(PID *pid, float cmd_rate, float rate, float K_rate, float Maxoutput)//参数K是角速率反馈项系数
{
	float ro = 0.0;//积分是否分离标识
	float alpha = 1.0;//舵机输出是否超过其设置范围标识，用于抗积分饱和
	float pterm, dterm, result, absferror;

	pid->ferror = (pid->sp) - (pid->pv);// 计算当前时刻的偏差

	if ((pid->ferror>0) && (pid->ferror<pid->deadband))
		pid->ferror = 0;
	if ((pid->ferror<0) && (pid->ferror>-pid->deadband))
		pid->ferror = 0;
	//	else
	//	{
	//		if(pid->ferror>0);
	////		pid->ferror -= pid->deadband;
	//		else;
	////		pid->ferror +=pid->deadband;
	//	}

	//pid->deadband = abs(pid->ferror);
	if ((pid->ferror>0) && (pid->ferror>pid->divide))
		ro = 0.0;//如果当前绝对误差项大于分离区间,则不考虑积分项
	else if ((pid->ferror<0) && (pid->ferror<-pid->divide))
		ro = 0.0;//如果当前绝对误差项大于分离区间,则不考虑积分项
	else
		ro = 1.0;//如果当前绝对误差项小于分离区间,需要考虑积分项

	dterm = (pid->ferror - pid->last_error) * pid->dgain;//偏差微分项
	pterm = pid->pgain * pid->ferror;// 偏差比例项
	result = pterm + ro*pid->igain*pid->integral + dterm + K_rate*(cmd_rate - rate);// 舵机预输出
	if (abs(result) < Maxoutput)//PID控制器输出限幅
	{
		alpha = 1.0; // 无需抗积分饱和
	}
	if (result >= Maxoutput)//PID控制器的输出大于设定范围
	{
		result = Maxoutput;//
		if (pid->ferror > 0)//若当前偏差为正，则积分项停止累加,避免进入积分饱和
		{
			alpha = 0.0;
		}
		else
		{
			alpha = 1.0;//若当前偏差为负，积分项开始累加
		}
	}
	if (result <= -Maxoutput)//PID控制器的输出小于设定范围
	{
		result = -Maxoutput;//若当前偏差为负，启动抗积分饱和
		if (pid->ferror<0)
		{
			alpha = 0.0;
		}
		else
		{
			alpha = 1.0;
		}
	}
	//update:
	pid->integral += alpha*pid->ferror;// 偏差积分项
	pid->last_error = pid->ferror; //把当前时刻的误差作为下一个时刻的last_error
	return result;
}

