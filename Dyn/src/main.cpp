#include"General/SimTime.h"
#include "Satellite/Satellite.h"

int main()
{
	// huhu
	double SampleTime{ 0.1 };
	Satellite Amadeus;
	CSimTime* pSimTime = CSimTime::GetInstance();
	pSimTime->InitSimSpeedManage(SampleTime, 1);
	while (1)
	{
		pSimTime->WaitForSimCountMute();
		if (pSimTime->SimCountJudge())
		{
			Amadeus.StateRenew(SampleTime);
			std::cout << Amadeus.Env.NEDMag << std::endl;
		}
		pSimTime->ReleaseSimCountMute();
	}
	
}