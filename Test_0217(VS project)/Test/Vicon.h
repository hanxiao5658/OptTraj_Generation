#ifndef VICON_H
#define VICON_H
#include <stdio.h>
#include "stdafx.h"
#include "Client.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <time.h>

#ifdef WIN32
  #include <conio.h>   // For _kbhit()
  #include <cstdio>   // For getchar()
  #include <windows.h> // For Sleep()
#endif // WIN32

class Vicon
{
	
	char *ip;  //the ip address 
	unsigned short port;  //the port number
public:
	int host;
	Vicon(char *ip, unsigned short port);
	void GetData(float &x, float &y, float &z, float &aX, float &aY, float &aZ, float &px, float &py, float &pz, float &psticky, float &pstickz);
    void CloseVicon();
};  // this semicolon should not be forgotten

#endif