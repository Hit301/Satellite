#include"General/SimTime.h"
#include "Satellite/Satellite.h"
int main()
{
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
//			std::cout << Amadeus << std::endl;
			std::cout << "Gyro:\n " << Amadeus._Gyro.Data << std::endl;

			//std::cout << Amadeus;
		}
		pSimTime->ReleaseSimCountMute();
	}
}