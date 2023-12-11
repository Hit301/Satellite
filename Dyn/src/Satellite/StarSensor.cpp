#include "Satellite/StarSensor.h"

StarSensorScope::StarSensorScope():InstallMatrix(1, 0, 0,0, 1, 0,0, 0, 1)
{

	Data= Quat( 1, 0, 0, 0 );
	LastRenewTime = 0;
	SamplePeriod = 1;
}

void StarSensorScope::StateRenew(int64_t NowTime, Quat Quat_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		
		Quat installq;
		installq = InstallMatrix.ToQuat();
		Data = installq * Quat_b;
		
		LastRenewTime = NowTime;
	}
}
