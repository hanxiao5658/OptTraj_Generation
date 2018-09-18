#include "CTool.h"
float LG_takeoff_Opt(float tic)
{
	float c0, c1, c2, c3, c4, c5, c6, c7;
	float s;
	c4 = 0.00854046;
	c5 = -0.00256193;
	c6 = 0.000266849;
	c7 = -9.52977e-6;
	s = c4*pow(tic, 4) + c5*pow(tic, 5) + c6*pow(tic, 6) + c7*pow(tic, 7);
	if (tic >= 8.0)
	{
		s = 1.0;
	}
	return s;
}
void Cmd_send_uart(SOuter uouter_in)
{	
	char sendbuff[20]; 
	
	uouter_in.pitch_desired *= -2290.88;
	uouter_in.roll_desired *= 2290.88;
	uouter_in.yaw_desired *= 0.0*-57.3 / 51.2 * 2047.0;
	//uouter_in.yaw_desired = 0.0;

	uouter_in.thrust = uouter_in.thrust / 14.04905 * 4095.0;

	if (uouter_in.pitch_desired > 1845)
	{
		uouter_in.pitch_desired = 1845;
	}
	if (uouter_in.pitch_desired<-1845)
	{
		uouter_in.pitch_desired = -1845;
	}

	if (uouter_in.roll_desired > 1845)
	{
		uouter_in.roll_desired = 1845;
	}
	if (uouter_in.roll_desired < -1845)
	{
		uouter_in.roll_desired = -1845;
	}

	if (uouter_in.thrust > 3500.0)
	{
		uouter_in.thrust = 3500.0;
	}
	if (uouter_in.thrust < 0.0)
	{
		uouter_in.thrust = 1.0;
	}

	memcpy(sendbuff, &uouter_in, 20);
	mySerialPort.WriteData((unsigned char*)(sendbuff), 20);
}
void LG_diff(float u, float Fs, float &temp1, float &temp2)
{
	float h = 1.0 / Fs;
	float r = 25.0;

	temp1 = temp1 + h*(temp2);
	temp2 = temp2 + h*(-r*r*(temp1 - u) - 2 * r*(temp2));
}

float steptest(float Mag, float tmax, float tic)
{
	float u = 0.0;
	if(tic<=2)
		u = 0;
	if((tic>2)&&(tic<=6))
		u = Mag;
	if((tic>6)&&(tic<=10))
		u = -Mag;
	if((tic>10)&&(tic<=14))
		u = 0;
	if((tic>14)&&(tic<=18))
		u = -Mag;
	if((tic>18)&&(tic<=22))
		u = Mag;
	if(tic>22)
		u=0;
	return u;
}

float path_x_square(float Mag, float tic)
{
	float des_x = 0.0;
	
	if (tic <= 2)
		des_x = 0.0;
	if ((tic>2) && (tic <= 5))
		des_x = Mag;
	if ((tic>5) && (tic <= 8))
		des_x = Mag;
	if ((tic>8) && (tic <= 14))
		des_x = -Mag;
	if ((tic>14) && (tic <= 20))
		des_x = -Mag;
	if ((tic>20) && (tic <= 26))
		des_x = Mag;
	if ((tic>26) && (tic <= 32))
		des_x = Mag;
	if ((tic>32) && (tic <= 35))
		des_x = 0.0;

	return des_x;
}

float path_y_square(float Mag, float tic)
{
	float des_y = 0.0;

	if (tic <= 2)
		des_y = 0.0;
	if ((tic>2) && (tic <= 5))
		des_y = 0.0;
	if ((tic>5) && (tic <= 8))
		des_y = Mag;
	if ((tic>8) && (tic <= 14))
		des_y = Mag;
	if ((tic>14) && (tic <= 20))
		des_y = -Mag;
	if ((tic>20) && (tic <= 26))
		des_y = -Mag;
	if ((tic>26) && (tic <= 32))
		des_y = 0.0;
	if ((tic>32) && (tic <= 35))
		des_y = 0.0;

	return des_y;
}



////////////////////////////////////////////////////////////////////////

void CALLBACK Timer_fun(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	CONSOLE_SCREEN_BUFFER_INFO coninfo;
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleScreenBufferInfo(hConsole, &coninfo);
	coninfo.dwCursorPosition.Y -= 4;    // move up one line
	coninfo.dwCursorPosition.X  = 0;    // move to the right the length of the word
	SetConsoleCursorPosition(hConsole, coninfo.dwCursorPosition);
	// tracking differentor:
	//static float xQ_x1, xQ_x2;
	//LG_diff(Data_now.xQ, 50, xQ_x1, xQ_x2);
	//Data_now.dxQ = xQ_x2;

	//static float xQ_y1, xQ_y2;
	//LG_diff(Data_now.yQ, 50, xQ_y1, xQ_y2);
	//Data_now.dyQ = xQ_y2;

	//static float xQ_z1, xQ_z2;
	//LG_diff(Data_now.zQ, 50, xQ_z1, xQ_z2);
	//Data_now.dzQ = xQ_z2;

	//static float yaw_1, yaw_2;
	//LG_diff(Data_now.psi, 50, yaw_1, yaw_2);
	//Data_now.dpsi = yaw_2;

	// tracking differentor:
	Data_now.dxQ = -0.142857 * Data_last.dxQ + 57.142857 * ( Data_now.xQ - Data_last.xQ);
	Data_now.dyQ = -0.142857 * Data_last.dyQ + 57.142857 * (Data_now.yQ - Data_last.yQ);
	Data_now.dzQ = -0.142857 * Data_last.dzQ + 57.142857 * (Data_now.zQ - Data_last.zQ);
	Data_now.dpsi = -0.142857 * Data_last.dpsi + 57.142857 * (Data_now.psi - Data_last.psi);

	Data_now.dyL = -0.142857 * Data_last.dyL + 57.142857 * (Data_now.yL - Data_last.yL);
	Data_now.dzL = -0.142857 * Data_last.dzL + 57.142857 * (Data_now.zL - Data_last.zL);


	Data_now.dxe = -0.142857 * Data_last.dxe + 57.142857 * (Data_now.xe - Data_last.xe);
	Data_now.dye = -0.142857 * Data_last.dye + 57.142857 * (Data_now.ye - Data_last.ye);
	Data_now.dze = -0.142857 * Data_last.dze + 57.142857 * (Data_now.ze - Data_last.ze);
	// update
	Data_last = Data_now;

	static bool is_first_PID = TRUE;
	static bool is_first_Sweep = TRUE;
	static bool isFirstKing = TRUE;
	static float max_attitude = 0.8032;//1845/2290.88
	static float initial_y, initial_z;
	static float ux, uy, uz;
	static float udx, udy, udz;
	static float uax, uay, uaz;
	//////////////////////////////////////////////////////////////////////////////////////////
	// Manual:
	//////////////////////////////////////////////////////////////////////////////////////////
	if (is_Manual)
	{
		printf("TimeNow: %7.2f, Control Mode:	Manual  			\n", timeNow);
		

		//function parts:
		is_first_PID = TRUE;
		is_first_Sweep = TRUE;
		switch (flightflag)
		{
		case YawLeft:
			
			break;
		case YawRight:
			
			break;
		case GoUp:
			
			break;
		case GoDown:
			
			break;
		case RollLeft:
			
			break;
		case RollRight:
			
			break;
		case PitchUp:
			
			break;
		case PitchDown:
			
			break;
		default:
			
			break;
		}

	}

	//////////////////////////////////////////////////////////////////////////////////////////
	// Sweep:
	//////////////////////////////////////////////////////////////////////////////////////////
	static float phi0, theta0, thrust0;
	if (is_Sweep)
	{
		printf("TimeNow: %7.2f, Control Mode:	Payload 			\n", timeNow);

		if (is_first_Sweep)//work for the first time in Sweep mode:
		{
			if (K_switch_P == 1.0)
				K_switch_P = 0.0;
			else 
				K_switch_P = 1.0;
		}
		is_first_Sweep = FALSE;

	}

	//////////////////////////////////////////////////////////////////////////////////////////
	// PID:
	//////////////////////////////////////////////////////////////////////////////////////////
	static float temp;
	static float ticTakeOff, ticLand;
	static float x0, y0, z0;
	if (is_PID)
	{
		is_first_Sweep = TRUE;

		printf("TimeNow: %7.2f, Control Mode:	PID 			\n", timeNow);

		if (is_first_PID)//work for the first time entering PID
		{
			ticPID = timeNow;

			x0 = Data_now.xQ;
			y0 = Data_now.yQ;
			z0 = Data_now.zQ;
			setpoint_x = x0;
			setpoint_y = y0;
			setpoint_z = z0;
			setpoint_yaw = deg2rad(0.0);
			setpoint_ax = 0.0;
			setpoint_ay = 0.0;
			setpoint_az = 0.0;
		}
		is_first_PID = FALSE;

		static float Max_accZ = 9.81*1000.0 / 9.0 *3.0;
		static float Max_accXY = 9.81 *1000.0 / 100*10;
		static float Max_desyaw = 1.0;
		static float kstop = 0.1;

		static float tic = 0.0;
		static float tic_line = 0.0;
		static float tic_circle = 0.0;

		static float kw = 2 * Pi*fHzmax / tmax;
		static float phase = 0.5*kw*tmax*tmax - 2 * Pi*fHzmax*tmax;
		switch (flightflag)
		{
		case TakeOff:	

			kstop = 1.0;
			if (!isInAir)
			{
				if (ticTakeOff<=25.0 && setpoint_z<1600.0)//Altitude <= 1.0, keep going up
				{
					//setpoint_z = LG_takeoff_Opt(ticTakeOff);
					setpoint_z += 3.2;
					ticTakeOff += 0.02;/* 5*50*2.5=625 */
				}
				else
				{
					setpoint_z = 1600.0;
					ticTakeOff = 0.0;
					isInAir = TRUE;
				}
			}

			break;
		case xSweep://back
			static int temp_num_x;
			if (temp_num_x <= 100)
			{
				setpoint_y -= 10.0;
				//u = steptest(200, tmax, tic);
				//setpoint_x = ux;
				//setpoint_ax = -200.0*cos(0.5*kw*tic*tic)*kw + 200.0*sin(0.5*kw*tic*tic)*kw*kw*tic*tic;
				tic += 0.02;
				temp_num_x++;
				//u = steptest(Mag, tmax, tic);
			}
			else
			{
				temp_num_x = 0.0;
				flightflag = StopSweep;
			}
			break;
		case ySweep://forward
			static int temp_num_y;
			if (temp_num_y<=100)
			{
				setpoint_y += 10.0;
				//uy = -200 * sin(*/0.5*kw*tic*tic);
				//u = steptest(200, tmax, tic);
				//setpoint_x = ux;
				//setpoint_ax = -200.0*cos(0.5*kw*tic*tic)*kw + 200.0*sin(0.5*kw*tic*tic)*kw*kw*tic*tic;
				tic += 0.02;
				temp_num_y++;
				//u = steptest(Mag, tmax, tic);
			}
			else
			{
				temp_num_y = 0;
				flightflag = StopSweep;
			}
			//if (tic <= 60.0)//tmax = 24.0
			//{
			//	if (tic <= tmax)
			//	{
			//		uy = -400 * sin(0.5*kw*tic*tic);

			//	}
			//	else
			//	{
			//		uy = -400 * sin(2*Pi*fHzmax*tic + phase);
			//	}

			//	//u = steptest(200, tmax, tic);
			//	//setpoint_y = u;
			//	//setpoint_ay = -200.0*cos(0.5*kw*tic*tic)*kw + 200.0*sin(0.5*kw*tic*tic)*kw*kw*tic*tic;
			//	tic += 0.02;
			//	//u = steptest(Mag, tmax, tic);
			//}
			//else
			//{
			//	flightflag = StopSweep;
			//}
			break;
		case Desired_trj:
			static int n_csv;
			
			if (n_csv <= 240)//stage1: swing up
			{
				uay = 1000.0*csv_ay[n_csv];
				udy = 1000.0*csv_dy[n_csv];
				uy = 1000.0*csv_y[n_csv];
				
			}

			if ( (n_csv > 240)&& (n_csv <= 261))//stage2: speed up :265
			{
				uay = 1000.0*csv_ay[n_csv];
				udy = 1000.0*csv_dy[n_csv] + 1000.0; 
				uy = 1000.0*csv_y[n_csv] + 300.0;

			}

			if ((n_csv > 261) && (n_csv <= 270))//stage3: slow down
			{
				uay = 1000.0*csv_ay[n_csv];
				udy = 1000.0*csv_dy[n_csv];
				uy = 1000.0*csv_y[n_csv] + 300.0;
			}
			if ((n_csv > 270) && (n_csv <= 350))
			{
				uay = 1000.0*csv_ay[n_csv];
				udy = 1000.0*csv_dy[n_csv];
				uy = 1000.0*csv_y[n_csv] + 300.0;
				//K_switch_P = 1.0;
			}
			if ((n_csv > 350)) // stage 3: start to stabilise
			{
				udy = 0;
				uay = 0;
				flightflag = StopSweep;
			}

			n_csv++;
			break;
		case zSweep:
			if (tic <= 36)
			{
				setpoint_x = path_x_square(350.0,tic);
				setpoint_y = path_y_square(350.0, tic);
				tic += 0.02;
			}
			else
			{
				flightflag = StopSweep;
			}
			break;
		case hSweep:
			break;
		case Jump:
			setpoint_x = 0.0;
			setpoint_y = -150.0;
			flightflag = StopSweep;
			break;
		case King: //control by a stick

			if (isFirstKing)
			{
				initial_y = Data_now.yStick;
				initial_z = Data_now.zStick;

				isFirstKing = FALSE;
			}

			uy = (Data_now.yStick - initial_y);//mm
			uz = (Data_now.zStick - initial_z);//mm
			if (uy >= 4000.0/5.0)
				uy = 4000.0 / 5.0;
			if (uy <= -2000.0/5.0)
				uy = -2000.0 / 5.0;
			
			if (uz >= 50.0 / 4.0)
				uz = 50.0 / 4.0;
			if (uz < -50.0 / 4.0)
				uz = -50.0 / 4.0;

			uy = uy*5.0*K_switch_King;//dy = -2000.0 ~ 4000.0;
			uz = uz*4.0*K_switch_King;//dz = -200.0 ~ 300.0;


			break;
		case circle:
			
			/*if (tic_line <= 5.0)
			{
				setpoint_x += 2.4;
				tic_line += 0.02;
			}*/

			if (uy >= 0)
			{
				uy -= 10.4;
			}
			else
			{
				flightflag = StopSweep;
			}

			//if(tic_circle<18.0)
			//{
			//	//setpoint_x = 600 * sin(2 * Pi * 1 / 10.0*tic_circle);
			//	//setpoint_y = 600 * sin(2 * Pi * 1 / 20.0*tic_circle);
			//	setpoint_x = 600*cos(2.0 * Pi * 1.0 / 10.0*tic_circle);
			//	setpoint_y = 600*sin(2.0 * Pi * 1.0 / 10.0*tic_circle);
			//	tic_circle += 0.02;
			//}
			//else
			//{
			//	setpoint_x = 0.0;
			//	setpoint_y = 0.0;
			//	flightflag = StopSweep;
			//}
			
			break;
			
			
			//setpoint_x = 600 * sin(2 * Pi * 1 / 20.0*tic_circle);
			//setpoint_y = 600 * sin(2 * Pi * 1 / 10.0*tic_circle);
			//tic_circle += 0.02;

			break;

		case Land:
			if (isInAir)//Now drone isInAir
			{
				K_switch_P = 0.0;
				if (ticLand<=5.0 && Data_now.zQ >=100.0)
				{
					setpoint_z -= 10.0;//mm
					setpoint_ax = 0.0;
					setpoint_ay = 0.0;
					setpoint_az = 0.0;

					ticLand += 0.02;
				}
				else
				{
					kstop = 0.10;
					setpoint_z = 0.0;
					ticLand = 0.0;
					isInAir = FALSE;
				}
			}		
			break;
		case StopSweep:
			
			tic_circle = 0.0;
			tic = 0.0;//when it stops, clear the tic for next trajectory
			break;
		default:
			break;
		}

		// PID for payload stablization:
		static float Kp_xe = -0.8;//-1.0
		static float Kd_xe = -0.18;//-0.2
		static float Kp_ye = -0.8;
		static float Kd_ye = -0.18;

		static float Kp_ze = -0.0;
		static float Kd_ze = -1.0;

		static float K_protect = 1.0;
		if ((Data_now.dze > -200.0)&&(Data_now.dze < 200.0)) // for protection
		{
			K_protect = 1.0;
		}
		else
		{
			K_protect = 0.0;
		}
		//PID for Height:
		pid_init(&PID_z, Data_now.zQ, setpoint_z+uz);
		acc.z = pid_calc(&PID_z, setpoint_dz, Data_now.dzQ, kp_vz, Max_accZ) + K_protect * K_switch_P*(Kp_ze*(0 - Data_now.ze) + Kd_ze*(0 - Data_now.dze));//
		uouter_in.thrust = kstop*m*g + acc.z/1000.0;	
		// 

		//PID for position x,y:
		pid_init(&PID_x, Data_now.xQ, setpoint_x+ux);
		acc.x = ( pid_calc(&PID_x, setpoint_dx, Data_now.dxQ, kp_vx, Max_accXY) + K_switch_P*( Kp_xe*(0-Data_now.xe)+Kd_xe*(0-Data_now.dxe) )      ) / 1000.0;//output is desired ax

		pid_init(&PID_y, Data_now.yQ, setpoint_y+uy);// track uay, uy, y; acceleration, speed, position
		acc.y = (uay/6.0 + pid_calc(&PID_y, setpoint_dy + udy, Data_now.dyQ, kp_vy, Max_accXY) + K_switch_P*( Kp_ye*(0-Data_now.ye)+Kd_ye*(0-Data_now.dye))       )/ 1000.0;//output is desired ay

		uouter_in.pitch_desired = (acc.x*cos(Data_now.psi) + acc.y*sin(Data_now.psi));//pitch -> x
		uouter_in.roll_desired  = (acc.x*sin(Data_now.psi) - acc.y*cos(Data_now.psi));//roll  -> y

		//yaw control
		pid_init(&PID_yaw, Data_now.psi, setpoint_yaw);
		uouter_in.yaw_desired = pid_calc(&PID_yaw, setpoint_r, Data_now.dpsi, kp_r, Max_desyaw);
	}


	Cmd_send_uart(uouter_in); // send serial cmd to drone
	//fprintf_s(file_handle, "%7.2f,%d,%f,%f,%f,%f\n",
	//timeNow,flightflag,rad2deg(uouter_in.roll_desired), rad2deg(uouter_in.pitch_desired), rad2deg(uouter_in.yaw_desired), uouter_in.thrust);
	fprintf_s(file_handle, "%7.2f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
		timeNow, Data_now.xQ, Data_now.dxQ, Data_now.yQ, Data_now.dyQ, Data_now.zQ, Data_now.dzQ,
		uouter_in.roll_desired, uouter_in.pitch_desired, uouter_in.yaw_desired, uouter_in.thrust,
		Data_now.phi, Data_now.theta, Data_now.psi, (setpoint_x + ux), (setpoint_y + uy), (setpoint_z + uz),Data_now.dpsi,
		acc.x, acc.y,acc.z,setpoint_yaw,setpoint_ax, Data_now.xL, Data_now.yL, Data_now.zL,
		Data_now.xe, Data_now.dxe, Data_now.ye, Data_now.dye);

	//printf("\rx: %+7.2f, y: %+7.2f, z: %+7.2f, roll: %+6.2f, pitch: %+6.2f, yaw: %+6.2f",
		//Data_now.xQ, Data_now.yQ, Data_now.zQ, rad2deg(Data_now.phi), rad2deg(Data_now.theta), rad2deg(Data_now.psi));

	
	/*printf("Roll_cmd: %+6.2f, Pitch_cmd: %+6.2f, Psi_cmd: %+6.2f, Thrust_cmd: %+6.2f",
		rad2deg(uouter_in.roll_desired), rad2deg(uouter_in.pitch_desired), rad2deg(uouter_in.yaw_desired), uouter_in.thrust);*/

	printf("cmd_x(m): %+6.4f, cmd_y: %+6.4f, cmd_z: %+6.4f, cmd_psi: %+6.4f\n",(setpoint_x+ux) / 1000.0,(setpoint_y+uy)/1000.0 ,(setpoint_z+uz )/ 1000.0,setpoint_yaw);
	printf("  Q_x(m): %+6.4f,   Q_y: %+6.4f,   Q_z: %+6.4f,   isStable: %+6.4f\n", Data_now.xQ / 1000.0, Data_now.yQ / 1000.0, Data_now.zQ / 1000.0, Data_now.yStick);//K_switch_P
	printf("  P_x(m): %+6.4f, P_y: %+6.4f, P_z: %+6.4f, Xe: %+6.4f, Ye: %+6.4f\n", Data_now.xL / 1000.0, Data_now.yL / 1000.0, Data_now.zL / 1000.0, Data_now.xe / 1000.0, Data_now.ye / 1000.0);
	//printf("cmd_x:	%+6.4f, cmd_y:	%+6.4f, cmd_z:	%+6.4f\n", acc.x, acc.y, acc.z);
	printf("Roll_cmd: %+6.4f, Pitch_cmd: %+6.4f, Psi_cmd: %+6.4f, Th_cmd: %+6.4f",
		(uouter_in.roll_desired), (uouter_in.pitch_desired), uouter_in.yaw_desired, uouter_in.thrust);
	fflush(stdout);
	timeNow += 0.02;
}

bool init_KeyScanThread(void)
{
	HANDLE hThread = NULL;
	UINT uThreadAddr = NULL;
	LPVOID arglist = NULL;
	//Create Threads with CREATE_SUSPENDED
	hThread = (HANDLE)_beginthreadex(NULL, 0, &Thread_KeyScan, arglist, CREATE_SUSPENDED, &uThreadAddr);
	if (hThread == NULL)
		return false;

	//Resume Threads after CSimulationConfig::IdList has been fully initialised
	ResumeThread(hThread);
	return true;
}

unsigned __stdcall Thread_KeyScan(void * pParam)
{

	unsigned selfId = GetCurrentThreadId();
	//printf("%u	-->	Started.\n", selfId);
	Sleep(1);

	while(TRUE)
	{
		char tmp = _getch();

		switch (tmp)
		{
		case KEY_0://Manual mode:
			is_Manual = TRUE;
			is_Sweep = FALSE;
			is_PID = FALSE;
			//uouter_in.thrust = m*(g-1.0);
			break;
		case KEY_1://Sweep mode:
			is_Sweep = TRUE;
			is_Manual = FALSE;
			is_PID = FALSE;
			break;
		case KEY_5://PID mode:
			is_PID = TRUE;
			is_Sweep = FALSE;
			is_Manual = FALSE;
			break;

		case KEY_a://Yaw left
			flightflag = YawLeft;
			//uouter_in.yaw_desired += deg2rad(5.0);
			break;
		case KEY_d://Yaw right
			flightflag = YawRight;
			//uouter_in.yaw_desired -= deg2rad(5.0);
			break;
		case KEY_w://Up
			flightflag = GoUp;
			//uouter_in.thrust += 0.05;
			break;
		case KEY_s://Down
			//flightflag = switchPayload;
			if (K_switch_P == 1.0)
				K_switch_P = 0.0;
			else
				K_switch_P = 1.0;
			break;

			//uouter_in.thrust -= 0.05;
			break;
		case KEY_j://roll(- left), move left
			flightflag = Jump;
			//uouter_in.roll_desired -= deg2rad(5);
			break;
		case KEY_l://roll(+ right), move right
			flightflag = Desired_trj;
			//uouter_in.roll_desired += deg2rad(5);
			break;
		case KEY_i://pitch(+ nose down), move forward
			flightflag = PitchUp;
			//uouter_in.pitch_desired += deg2rad(5);
			break;
		case KEY_k://pitch down(- nose up), move back
			flightflag = King;
			if (K_switch_King == 1.0)
				K_switch_King = 0.0;
			else
				K_switch_King = 1.0;
			break;
			//uouter_in.pitch_desired -= deg2rad(5);
			break;

		case KEY_b:
			flightflag = TakeOff;	
			break;
		case SpaceBar://Land
			flightflag = Land;
			break;

		case KEY_x://x roll sweep test
			flightflag = xSweep;
			break;
		case KEY_y://y pitch sweep test
			flightflag = ySweep;
			break;
		case KEY_z://z yaw sweep test
			flightflag = zSweep;
			break;
		case KEY_h://h thrust sweep test
			flightflag = hSweep;
			break;
		case KEY_c://circle
			flightflag = circle;
			break;
		default:
			;
		}
	}
	
	return selfId;
}