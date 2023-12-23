#include"General/SimTime.h"
#include "Satellite/Satellite.h"
#include"General/InfluxDB.h"

int main()
{
	// 2023-12-22 11:19:57
	//CInfluxDB DB("127.0.0.1", 8086, "Satellite_db");
	double SampleTime{ 0.1 };
	int SpeedTimes = 1;
	Satellite Amadeus(SampleTime, SpeedTimes);
	CSimTime* pSimTime = CSimTime::GetInstance();
	pSimTime->InitSimSpeedManage(SampleTime, SpeedTimes);
	while (1)
	{
		pSimTime->WaitForSimCountMute();
		if (pSimTime->SimCountJudge())
		{
			Amadeus.StateRenew();
			std::cout << Amadeus;
		}
		pSimTime->ReleaseSimCountMute();
		//Amadeus.dataToDB(DB, 5);
	}	
}