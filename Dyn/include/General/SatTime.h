#pragma once
#include<chrono>
#include <iostream>
#include<windows.h>

typedef struct SatTime
{
	double SampleTime;//采样时间，单位是s;
	int SpeedTimes;//加速倍率

}*pSatTime;

extern int SimCount;//每个采样时间步内仿真执行的次数
extern HANDLE hSimCountMute;
int64_t GetTimeStampMs();

unsigned __stdcall SimCountManage(void* arg);


