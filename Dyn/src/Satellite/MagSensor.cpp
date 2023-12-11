#include "Satellite/MagSensor.h"

MagSensor::MagSensor()
{
	InstallMatrix << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	Data << 0, 0, 0;
	LastRenewTime = 0;
	SamplePeriod = 1;
}

void MagSensor::StateRenew(int64_t NowTime, Eigen::Vector3d B_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		Data = InstallMatrix * B_b;
		LastRenewTime = NowTime;
	}
}