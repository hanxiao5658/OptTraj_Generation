#ifndef VICON_THREAD_H
#define VICON_THREAD_H
#include "Vicon.h"
#include <windows.h>
#include <math.h>
#include <process.h>
#include "CTool.h"

#define DISTANCE_THRESHOLD 5


// Vicon Data Structure:

//float Euclidean_Distance(Coordinate p1, Coordinate p2);
bool init_ViconThread(void);
unsigned __stdcall Thread_Vicon(void * pParam);

#endif
