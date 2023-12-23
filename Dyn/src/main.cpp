#include"General/SimTime.h"
#include "Satellite/Satellite.h"
#include"General/InfluxDB.h"

int main()
{
	// 2023-12-22 11:19:57
	CInfluxDB DB("127.0.0.1", 8086, "Satellite_db");
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
		}
		pSimTime->ReleaseSimCountMute();
		// 2023-12-22 11:20:02
		Amadeus.dataToDB(DB, 5);
	}
	
}