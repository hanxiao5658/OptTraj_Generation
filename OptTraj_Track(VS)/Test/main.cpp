
#include"CTool.h"
#include"CPID.h"

#pragma comment(lib,"winmm.lib")  //°²×°¶àÃ½Ìå¶¨Ê±Æ÷Ö§³Ö¿â

// Global Variables, Structs and Objects
#define TIMER_ACCURACY 20  //¶¨Ê±Æ÷·Ö±æÂÊ
float timeNow = 0.0;
float ticPID = 0.0;
const float Pi = 3.1415926;
const float Mag_thrust = 0;
const float tmax = 20.0;
const float fHzmax = 0.7;//3.0
const float Mag = 0.01;//Maxrad: deg2rad(10);
const float m = 0.53;//
const float g = 9.81;

CSerialPort mySerialPort; //serial port object
SOuter uouter_in; //cmd struct
SOuter uouter_out;//cmd send out

SUart_send uart_send; //
SUart_rev uart_rev;

FILE *file_handle = NULL; //file handle
errno_t err_file; //iserror 

// Flight flags:
int flightflag;
bool isInAir = FALSE;
bool is_Manual = TRUE;//Ä¬ÈÏ¿ªÊ¼ÊÇÊÖ¶¯Ä£Ê½
bool is_Sweep = FALSE;
bool is_PID = FALSE;

// Vicon Data Structure:
ViconData Data_now;
ViconData Data_last;

// PID Data Structures:
Acceleration acc;
PID PID_x, PID_y, PID_z, PID_yaw;

//float kp_x = 0.03;		float ki_x = 0.0002;   float kd_x = 0.001;		float kp_vx = 0.4;
//float kp_y = 0.03;		float ki_y = 0.0002;   float kd_y = 0.001;		float kp_vy = 0.4;
//float kp_z = 2.0;		float ki_z = 0.000;    float kd_z = 0;			float kp_vz = 0.59*2.5;//0.324 * 7,0.59*2.5; 
//float kp_yaw = 0.3;		float ki_yaw = 0.0;	   float kd_yaw = 0.0;	    float kp_r = 0.5;

float kp_x = 1.134;		float ki_x = 0.0;   float kd_x = 0.0;		float kp_vx = 0.7375;
float kp_y = 1.134;		float ki_y = 0.0;   float kd_y = 0.0;		float kp_vy = 0.7375;
float kp_z = 2.268;		float ki_z = 0.0;	   float kd_z = 0.0;		float kp_vz = 1.475;//0.59*2.5;//0.324 * 7,0.59*2.5; 
float kp_yaw = 0.48;		float ki_yaw = 0.0;/*0.004*/  float kd_yaw = 0.000;	    float kp_r  = 0.0;

float Outer_deadband = 0.0;//Íâ»·Îó²îËÀÇøÎªÎ»ÖÃ,ËÀÇø:m
float yaw_deadband = deg2rad(0.0);
float Outer_intedivide = 2.0;//»ý·Ö·ÖÀë±äÁ¿: Îó²îÎª0.2m·¶Î§ÒÔÄÚÆô¶¯»ý·ÖÏî
float yaw_intedivide = deg2rad(20.0);
float integral_val = 0.0;//»ý·Ö³õÊ¼Öµ0.0, for both Inner and Outer PID

float setpoint_x , setpoint_y, setpoint_z, setpoint_yaw;
float setpoint_dx, setpoint_dy, setpoint_dz, setpoint_r;
float setpoint_ax, setpoint_ay, setpoint_az;

// read csv cmd:
float csv_ay[500];
float csv_dy[500];
float csv_y[500];
//
float K_switch_P = 0.0;
float K_switch_King = 0.0;

template<typename T>
T StringToNumber(const std::string& numberAsString)
{
	T valor;

	std::stringstream stream(numberAsString);
	stream >> valor;
	if (stream.fail()) {
		std::runtime_error e(numberAsString);
		throw e;
	}
	return valor;
}

int main(int argc, char* argv[])
{
	uouter_in.ID = 8;
	uouter_in.length = 20;
	uouter_in.pitch_desired = 0.0;
	uouter_in.roll_desired = 0.0;
	uouter_in.yaw_desired = 0.0;
	uouter_in.thrust = 1.0;
	//************************************************************
	// Part0: Import csv data
	//************************************************************
	std::ifstream in;
	in.open("TU_12_30_22_50_5_7.csv"); //import data from .csv file
	std::vector<std::vector <std::string> > Context;    //文件内容
	std::string line, field;
	
	if (in.is_open())
	{
		while (std::getline(in, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
		{
			static int i, m1, m2, m3;
			std::istringstream stream(line);
			//将一行的多个字段取出
			std::vector<std::string> RowContext;

			//以‘,’读取每个字段
			while (std::getline(stream, field, ','))
			{
				if (i % 3 == 0)//first column
				{
					csv_ay[m1] = StringToNumber<double>(field);
					m1++;
				}
				else if(i % 3 == 1)
				{
					csv_dy[m2] =  StringToNumber<double>(field);
					m2++;
				}
				else
				{
					csv_y[m3] = StringToNumber<double>(field);
					m3++;
				}
				i++;
			}
		}
	}
	else
	{
		//文件打开失败
		std::cout << "failed to open import file！";
	}

	//************************************************************
	// Part1: Serial Port Thread Initialization
	//************************************************************
	printf("Step1: Open serial port...");
	if (!mySerialPort.InitPort(6))//initial serialport 5
	{
		std::cout << "Fail" << std::endl;
	}
	else
	{
		std::cout << "Success" << std::endl;
	}
	printf("\tBuild serial port thread...");
	if (!mySerialPort.OpenListenThread())//open listen port5
	{
		std::cout << "Fail" << std::endl;
	}
	else
	{
		std::cout << "Success" << std::endl;
	}
	Sleep(500);



	//************************************************************
	// Part2: Saving File initialization
	//************************************************************
	//TCHAR * szTime = new TCHAR[128];
	SYSTEMTIME st = { 0 };
	GetLocalTime(&st);
	//printf("Starting time is£º%2d-%2d-%4d, %2d:%2d:%2d\n", st.wDay, st.wMonth, st.wYear, st.wHour, st.wMinute, st.wSecond);
	//std::string name;

	char file_name[128];
	sprintf_s(file_name, "%02d-%02d-%02d_%02dh%02dm.csv", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute);

	printf("\nStep2: create a saving file\n");
	err_file = fopen_s(&file_handle, file_name, "wt");

	Sleep(500);




	//************************************************************
	// Part3: Vicon Thread Initialization
	//************************************************************
	printf("\nStep3: Build Vicon thread\n");
	if (!init_ViconThread())
	{
		printf("Fail to initial ViconThread()");
		return 0;
	}
	Sleep(500);




	//************************************************************
	// Part4: KeyScan thread initialization
	//************************************************************
	printf("\nStep4: Build KeyScan thread...");
	if (!init_KeyScanThread())
		printf("Fail\n");
	else
		printf("Success\n");
	Sleep(500);


	//************************************************************
	// Part5: TimerThread Initialization
	//************************************************************
	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
	UINT wTimerRes = TIMER_ACCURACY; //¶¨ÒåÊ±¼ä¼ä¸ô:20ms
	UINT wAccuracy; //¶¨Òå·Ö±æÂÊ
	UINT TimerID; //¶¨Òå¶¨Ê±Æ÷¾ä±ú
	TIMECAPS tc;

	if (timeGetDevCaps(&tc, sizeof(TIMECAPS)) == TIMERR_NOERROR)
	{
		wAccuracy = min(max(tc.wPeriodMin, TIMER_ACCURACY), tc.wPeriodMax); //ÅÐ¶Ï·Ö±æÂÊÊÇ·ñÔÚÔÊÐí·¶Î§
		timeBeginPeriod(wAccuracy);  //ÉèÖÃ¶¨Ê±Æ÷·Ö±æÂÊ
		printf("\nStep5: Build a Timer thread\n\n");
		printf("Initialization success, while(TRUE) loop starts now...\n\n\n\n\n\n");
	}
	if ((TimerID = timeSetEvent(wTimerRes, wAccuracy, (LPTIMECALLBACK)Timer_fun, 0, TIME_PERIODIC)) == 0)
	{
		printf("Can't count!\n");
	}
	Sleep(500);




	//************************************************************
	// Part6: PID control initialization
	//************************************************************
	setpoint_dx = 0.0;
	setpoint_dy = 0.0;
	setpoint_dz = 0.0;
	setpoint_r  = 0.0;
	pid_tune(&PID_x, kp_x, ki_x, kd_x, Outer_deadband, Outer_intedivide, integral_val);//x,T  --> des_ax} -> des_phi,des_theta
	pid_tune(&PID_y, kp_y, ki_y, kd_y, Outer_deadband, Outer_intedivide, integral_val);//y,T  --> des_ay} -> des_phi,des_theta
	pid_tune(&PID_z, kp_z, ki_z, kd_z, Outer_deadband, Outer_intedivide, integral_val);// z,T   --> des_thrust
	pid_tune(&PID_yaw, kp_yaw, ki_yaw, kd_yaw, yaw_deadband, yaw_intedivide, integral_val); //yaw,T --> des_yaw

	//************************************************************
	// Testing part:
	//************************************************************
	//Stay busy:
	while(TRUE);
}

