#pragma once
#include<chrono>
#include <iostream>
#include<windows.h>

typedef struct SatTime
{
	double SampleTime;//����ʱ�䣬��λ��s;
	int SpeedTimes;//���ٱ���

}*pSatTime;

extern int SimCount;//ÿ������ʱ�䲽�ڷ���ִ�еĴ���
extern HANDLE hSimCountMute;
int64_t GetTimeStampMs();

unsigned __stdcall SimCountManage(void* arg);


