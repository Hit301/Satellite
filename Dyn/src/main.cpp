#include"General/SimTime.h"
#include "Satellite/Satellite.h"

int main()
{
	//zzw checkout
	double SampleTime{ 0.2 };
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