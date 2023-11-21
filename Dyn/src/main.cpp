#include"General/SimTime.h"
#include "Satellite/Satellite.h"

int main()
{
	// zhanln
	double SampleTime{ 0.1 };
	Satellite satellite;
	CSimTime* pSimTime = CSimTime::GetInstance();

	pSimTime->InitSimSpeedManage(SampleTime, 5);
	while (1)
	{
		pSimTime->WaitForSimCountMute();
		if (pSimTime->SimCountJudge())
		{
			satellite.StateRenew(SampleTime);
			std::cout << satellite;
		}
		pSimTime->ReleaseSimCountMute();
	}
}