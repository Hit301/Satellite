#include"General/SimTime.h"
#include "Satellite/Satellite.h"

int main()
{
	//12121
	double SampleTime{ 0.1 };
	Satellite Amadeus;
	CSimTime* pSimTime = CSimTime::GetInstance();
	pSimTime->InitSimSpeedManage(SampleTime, 5);
	while (1)
	{
		pSimTime->WaitForSimCountMute();
		if (pSimTime->SimCountJudge())
		{
			Amadeus.StateRenew(SampleTime);
			std::cout << Amadeus;
		}
		pSimTime->ReleaseSimCountMute();
	}
}