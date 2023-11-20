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
	double SampleTime;//����ʱ�䣬��λ��s;
	int SpeedTimes;//���ٱ���
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
//brief:��ȡ��ǰϵͳʱ�������λms
//
int64_t GetTimeStampMs();