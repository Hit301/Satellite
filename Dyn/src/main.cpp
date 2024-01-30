#include"General/SimTime.h"
#include "Satellite/Satellite.h"
#include"General/InfluxDB.h"
#include "General/IniConfig.h"
int main(int argc, char* argv[])
{
	//CInfluxDB DB;
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
		// 2023-12-22 11:20:02
		//Amadeus.data2DB(DB, 1);
	}	
	return 0;
}