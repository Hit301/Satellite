#include "General/SatTime.h"

int SimCount = 0;
HANDLE hSimCountMute;
int64_t GetTimeStampMs()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

unsigned __stdcall SimCountManage(void* arg)
{
	pSatTime _ST = (pSatTime)arg;
	hSimCountMute=CreateMutex(NULL, FALSE, NULL);
	if (hSimCountMute == NULL)
	{
		std::cout << "»¥³âËø´´½¨Ê§°Ü£¬º¯ÊýÍË³ö\n";
		return 0;
	}
	int64_t NexTime = GetTimeStampMs();
	int64_t WaitTime = 0;
	while (1)
	{
		int64_t NowTime = GetTimeStampMs();
		WaitTime = NexTime - NowTime;
		if (WaitTime > 0)
			Sleep(static_cast<DWORD>(WaitTime));
		WaitForSingleObject(hSimCountMute, INFINITE);
		SimCount += _ST->SpeedTimes;
		ReleaseMutex(hSimCountMute);
		NexTime += static_cast<int64_t>(_ST->SampleTime * 1e3);
	}
	return 0;
}
