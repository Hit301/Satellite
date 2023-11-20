#include "General/SimTime.h"

CSimTime::DeleteHelper CSimTime::helper;


int64_t GetTimeStampMs()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

unsigned __stdcall SimCountManage(void* arg)
{
	CSimTime* instance = CSimTime::GetInstance();
	int64_t NexTime = GetTimeStampMs();
	int64_t WaitTime = 0;
	while (1)
	{
		int64_t NowTime = GetTimeStampMs();
		WaitTime = NexTime - NowTime;
		if (WaitTime > 0)
			Sleep(static_cast<DWORD>(WaitTime));
		instance->WaitForSimCountMute();
		instance->SimCount += instance->SpeedTimes;
		instance->ReleaseSimCountMute();
		NexTime += static_cast<int64_t>(instance->SampleTime * 1e3);
	}
	_endthreadex(0);
	return 0;
}

CSimTime* CSimTime::GetInstance()
{
		if (m_instance == NULL)
			m_instance = new CSimTime;
		return m_instance;	
}

int CSimTime::WaitForSimCountMute()
{
	return WaitForSingleObject(hSimCountMute, INFINITE);
}

int CSimTime::ReleaseSimCountMute()
{
	return ReleaseMutex(hSimCountMute);
}

void CSimTime::InitSimSpeedManage(double SampleTime, int SpeedTimes)
{
	CSimTime* instance = CSimTime::GetInstance();
	instance->SampleTime = SampleTime;
	instance->SpeedTimes = SpeedTimes;
	_beginthreadex(NULL, 0, SimCountManage, NULL, 0, NULL);
}

bool CSimTime::SimCountJudge()
{
	if (SimCount > 0)
	{
		SimCount--;
		return true;
	}
	else
		return false;
}

CSimTime::CSimTime():SimCount(0), SampleTime(0.5), SpeedTimes(1)
{
	hSimCountMute = CreateMutex(NULL, FALSE, NULL);
	if (hSimCountMute == NULL)
	{
		std::cout << "»¥³âËø´´½¨Ê§°Ü£¬³ÌÐòÍË³ö\n";
		exit(0);
	}
}

void CSimTime::ReleaseInstance()
{
		CSimTime* tmp = m_instance;
		m_instance = NULL;
		delete tmp;	
}

CSimTime::~CSimTime()
{
	CloseHandle(hSimCountMute);
}
