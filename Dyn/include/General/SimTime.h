#pragma once
#include<chrono>
#include <iostream>
#include<windows.h>
#include<process.h>

class CSimTime
{
public:
	static CSimTime* GetInstance();
	int WaitForSimCountMute();
	int ReleaseSimCountMute();
	void InitSimSpeedManage(double SampleTime, int SpeedTimes);
	bool SimCountJudge();
private:
	double SampleTime;//采样时间，单位是s;
	int SpeedTimes;//加速倍率
	int SimCount;
	HANDLE hSimCountMute;

	static inline CSimTime* m_instance{ NULL };
	CSimTime();
	~CSimTime();
	CSimTime(const CSimTime& _CSimTime) = delete;
	CSimTime& operator=(const CSimTime& _CSimTime) = delete;

	static void ReleaseInstance();
	friend unsigned __stdcall SimCountManage(void* arg);
	class DeleteHelper
	{
	public:
		DeleteHelper() = default;
		~DeleteHelper()
		{
			ReleaseInstance();
		}
	};
	static DeleteHelper helper;
};

//
//brief:获取当前系统时间戳，单位ms
//
int64_t GetTimeStampMs();