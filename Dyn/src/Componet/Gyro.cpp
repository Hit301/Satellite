#include "Componet/Gyro.h"

GyroScope::GyroScope()
{
	InstallMatrix << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
	Data << 0, 0, 0;
	LastRenewTime = 0;
	SamplePeriod = 1;
}

void GyroScope::StateRenew(int64_t NowTime, Eigen::Vector3d Omega_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod*1e3)
	{
		Data = InstallMatrix * RAD2DEG*Omega_b;
		LastRenewTime = NowTime;
	}
}

void GyroScope::Init(Eigen::Vector3d& Omega_b, int64_t timestamp)
{
	LastRenewTime = timestamp;
	Data = RAD2DEG * InstallMatrix * Omega_b;
}