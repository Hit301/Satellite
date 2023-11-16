#include <iostream>
#include"General/SatTime.h"
#include "Satellite/Satellite.h"
#include<windows.h>
#include<process.h>

int main()
{
	//这是一次更改Ning
	SatTime ST_Amadeus{ 0 };
	ST_Amadeus.SampleTime = 0.05;
	ST_Amadeus.SpeedTimes = 2;
	_beginthreadex(NULL, 0, SimCountManage, &ST_Amadeus, 0, NULL);
	Satellite Amadeus;
	while (1)
	{
		WaitForSingleObject(hSimCountMute, INFINITE);
		if (SimCount > 0)
		{
			SimCount--;
			Amadeus.StateRenew(ST_Amadeus.SampleTime);
			std::cout << Amadeus;
		}
		ReleaseMutex(hSimCountMute);
	}
}