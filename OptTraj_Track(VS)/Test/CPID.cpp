/*
* VPID.cpp
*
*  Created on: 2016��5��11��
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
	pid->deadband = dead_band;//����
	pid->divide = divide;//���ַ������
	pid->integral = integral_val;//������
	pid->last_error = 0;
}
void pid_setinteg(PID *pid, float new_integ)//�������û�����,�ϴ��������
{
	pid->integral = new_integ;
	pid->last_error = 0;
}

void pid_bumpless(PID *pid)
{
	pid->last_error = (pid->sp) - (pid->pv);  //�趨ֵ�뷴��ֵƫ��
}

float pid_calc(PID *pid, float cmd_rate, float rate, float K_rate, float Maxoutput)//����K�ǽ����ʷ�����ϵ��
{
	float ro = 0.0;//�����Ƿ�����ʶ
	float alpha = 1.0;//�������Ƿ񳬹������÷�Χ��ʶ�����ڿ����ֱ���
	float pterm, dterm, result, absferror;

	pid->ferror = (pid->sp) - (pid->pv);// ���㵱ǰʱ�̵�ƫ��

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
		ro = 0.0;//�����ǰ�����������ڷ�������,�򲻿��ǻ�����
	else if ((pid->ferror<0) && (pid->ferror<-pid->divide))
		ro = 0.0;//�����ǰ�����������ڷ�������,�򲻿��ǻ�����
	else
		ro = 1.0;//�����ǰ���������С�ڷ�������,��Ҫ���ǻ�����

	dterm = (pid->ferror - pid->last_error) * pid->dgain;//ƫ��΢����
	pterm = pid->pgain * pid->ferror;// ƫ�������
	result = pterm + ro*pid->igain*pid->integral + dterm + K_rate*(cmd_rate - rate);// ���Ԥ���
	if (abs(result) < Maxoutput)//PID����������޷�
	{
		alpha = 1.0; // ���迹���ֱ���
	}
	if (result >= Maxoutput)//PID����������������趨��Χ
	{
		result = Maxoutput;//
		if (pid->ferror > 0)//����ǰƫ��Ϊ�����������ֹͣ�ۼ�,���������ֱ���
		{
			alpha = 0.0;
		}
		else
		{
			alpha = 1.0;//����ǰƫ��Ϊ���������ʼ�ۼ�
		}
	}
	if (result <= -Maxoutput)//PID�����������С���趨��Χ
	{
		result = -Maxoutput;//����ǰƫ��Ϊ�������������ֱ���
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
	pid->integral += alpha*pid->ferror;// ƫ�������
	pid->last_error = pid->ferror; //�ѵ�ǰʱ�̵������Ϊ��һ��ʱ�̵�last_error
	return result;
}

