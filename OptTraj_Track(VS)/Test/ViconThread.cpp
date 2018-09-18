#include "ViconThread.h"
#include "stdafx.h"


bool init_ViconThread(void)
{
	HANDLE hThread = NULL;
	UINT uThreadAddr = NULL;
    LPVOID arglist = NULL;
	//Create Threads with CREATE_SUSPENDED
	hThread = (HANDLE)_beginthreadex(NULL, 0, &Thread_Vicon, arglist, CREATE_SUSPENDED, &uThreadAddr);
	if(hThread == NULL)
		return false;

	//Resume Threads after CSimulationConfig::IdList has been fully initialised
	ResumeThread(hThread);
	return true;
}
unsigned __stdcall Thread_Vicon(void * pParam)
{

	unsigned selfId = GetCurrentThreadId();
	//printf("CurrentThreadID: %u	-->	Started.\n", selfId);
	Sleep(1);


	Vicon vicon("192.168.10.1",801);  // initialisation 

	while(1)
	{
		vicon.GetData(Data_now.xQ,Data_now.yQ,Data_now.zQ,Data_now.phi,Data_now.theta,Data_now.psi,Data_now.xL,Data_now.yL,Data_now.zL,Data_now.yStick,Data_now.zStick);  //, Current_coor.xk, Current_coor.yk, Current_coor.zk try to get data which are restored in x, y and z respectively.
		Data_now.xe = Data_now.xL - Data_now.xQ - 12.0;
		Data_now.ye = Data_now.yL - Data_now.yQ - 14.0;
		Data_now.ze = 700.0 - (Data_now.zQ - Data_now.zL);
		//mm: ze = length - (zQ-zL);
		//printf("The position is: x: %.2f , y: %.2f , z: %.2f\n", Data_now.xQ, Data_now.yQ, Data_now.zQ);
		
	}

	printf("CurrentThreadID: %u	-->	Terminated.\n", selfId);
	return selfId;
}

