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
			std::cout << Amadeus << std::endl;
			std::cout << "Gyro: " << Amadeus._Gyro.Data << std::endl;

			//std::cout << Amadeus;
		}
		pSimTime->ReleaseSimCountMute();
	}

	//CDcm DcmDemo1;

	//CDcm DcmDemo1(0, 1.57);
	
	//CDcm DcmDemo1(1,2,3,4,5,6,7,8,9);
	
	//CDcm DcmDemo1(1, 2, 3, 4, 5, 6, 7, 8, 9);
	//CDcm DcmDemo2(DcmDemo1);

	//CDcm DcmDemo1(1, 2, 3, 4, 5, 6, 7, 8, 9);
	//CDcm DcmDemo2;
	//DcmDemo2.operator=(DcmDemo1);

    //CDcm DcmDemo1(0.6313, 0.5317, -0.5646, -0.3583, 0.8456, 0.3957, 0.6878, -0.0475, 0.7243);
	//std::cout << DcmDemo1.DcmData << std::endl;
	//DcmDemo1.ToEulerAgl(EUL_SQE_YXZ);
	//DcmDemo1.ToQuat();
	
	//std::cout << DcmDemo2.DcmData << std::endl;
}