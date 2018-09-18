#ifndef CTOOL_H
#define CTOOL_H

#include<math.h>
#include<cmath>
#include <fstream>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <iostream>
#include <process.h>
#include <windows.h>
#include <conio.h>

#include <stdlib.h>
#include <sstream>

#include <vector>

#include "stdafx.h"
#include "helper.h"
#include "Vicon.h"
#include "ViconThread.h"
#include "serialport.h"
#include "datatransform.h"
#include "CPID.h"
struct Acceleration
{
	float x;
	float y;
	float z;
};
struct ViconData
{
	float frame;
	float xQ; //mm
	float yQ; //mm
	float zQ; //mm

	float dxQ; //mm/s
	float dyQ; //mm/s
	float dzQ; //mm/s

	float phi; // rad
	float theta; // rad
	float psi; // rad

	float dphi;// rad/s
	float dtheta;// rad/s
	float dpsi;// rad/s
	
	float xL; //mm
	float yL; //mm
	float zL; //mm
	float dxL; //mm/s
	float dyL; //mm/s
	float dzL; //mm/s

	float xe;//mm: xe = xL - xQ;
	float ye;//mm: ye = yL - yQ;
	float ze;//mm: ze = length - (zQ-zL);
	float dxe;//mm/s
	float dye;//mm/s
	float dze;//mm/s

	float xStick;
	float yStick;
	float zStick;
};

//Interface struct:
struct SOuter
{
	short ID;
	short length;

	float roll_desired;//rad
	float pitch_desired;//rad
	float yaw_desired;//rad/s

	float thrust;

	//to be send to quardrotor
};

struct SUart_send
{
	short ID;
	short length;
	short iframe;

	float dt;

	float x;
	float y;
	float z;

	float roll;
	float pitch;
	float yaw;

	//to be send to quardrotor
};
#pragma pack (1) 
struct SUart_rev
{
	short ID;
	short length;
	float dT;

	float x;
	float y;
	float z;

	float vX;
	float vY;
	float vZ;

	float roll;
	float pitch;
	float yaw;

	float p;
	float q;
	float r;
};
//Methods:
// flags
 enum KeyCmdOption
{
	TakeOff = 1,
	Land,
	RollLeft,
	RollRight,
	PitchUp,
	PitchDown,
	YawLeft,
	YawRight,
	GoUp,
	GoDown,
	Emergency,
	xSweep,
	ySweep,
	zSweep,
	hSweep,
	Jump,
	King,
	Desired_trj,
	StopSweep,
	circle,
	switchPayload
};



#define KEY_0 0x30  	    // 按下数字0:表示进入Manual模式(默认模式)
#define KEY_1 0x31  	    // 按下数字1:表示进入Sweep模式
#define KEY_5 0x35       // 按下数字5:表示进入PID模式

#define SpaceBar 0x20 // 按下空格: land,手动与自动模式都有效

 //手动模式下才有效的按键
#define KEY_b 0x62        // b  :  take off

#define KEY_a 0x61 		//a
#define KEY_d 0x64 		//d
#define KEY_w 0x77 		//w
#define KEY_s 0x73 		//s

#define KEY_j 0x6A //j
#define KEY_l 0x6C //l
#define KEY_i 0x69 //i
#define KEY_k 0x6B  //k

#define KEY_x  0x78//x
#define KEY_y  0x79//y
#define KEY_z  0x7A//z
#define KEY_h  0x68//h
#define KEYstep 10//手动模式,每按一次按键,命令增加10%
#define KEY_c 0x63
 // variables:
 extern float timeNow;
 extern float ticPID;

 extern int flightflag;
 extern bool isInAir;
 extern bool is_Manual;//默认开始是手动模式
 extern bool is_Sweep;
 extern bool is_PID;

// Variables, structs and objects
 extern const float tmax;
 extern const float fHzmax;//3.0
 extern const float Mag;//Maxrad: deg2rad(10);

 extern SOuter uouter_in;
 extern SOuter uouter_out;

 extern CSerialPort mySerialPort;
 extern FILE *file_handle;
 extern SUart_rev uart_rev;
 extern SUart_send uart_send;

 // Vicon data:
 extern ViconData Data_now;
 extern ViconData Data_last;

void LG_diff(float u, float Fs, float &temp1, float &temp2);
float steptest(float Mag, float tmax, float tic);//方波生成函数
float path_x_square(float Mag,float tic);// square path for x
float path_y_square(float Mag,float tic);// square path for y
void Cmd_send_uart(SOuter uouter_in);

// PID data declaration:
extern Acceleration acc;
extern PID PID_x, PID_y, PID_z, PID_yaw;
extern float kp_x, ki_x, kd_x, kp_vx;
extern float kp_y, ki_y, kd_y, kp_vy;
extern float kp_z, ki_z, kd_z, kp_vz;
extern float kp_yaw, ki_yaw, kd_yaw, kp_r;
extern float setpoint_x, setpoint_y, setpoint_z, setpoint_yaw;
extern float setpoint_dx, setpoint_dy, setpoint_dz, setpoint_r;
extern float setpoint_ax, setpoint_ay, setpoint_az;
extern float csv_ay[500];
extern float csv_dy[500];
extern float csv_y[500];

extern const float m;
extern const float g;
// Switch on payload stablization:
extern float K_switch_P;
extern float K_switch_King;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Threads:
//Timer callback function (50Hz)
void CALLBACK Timer_fun(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2);
// KeyScan thread:
bool init_KeyScanThread(void);
unsigned __stdcall Thread_KeyScan(void * pParam);

#endif