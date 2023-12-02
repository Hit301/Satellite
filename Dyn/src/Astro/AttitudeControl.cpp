#include "AStro/AttitudeControl.h"
#include "Satellite/Gyro.h"

//初始速率阻尼控制率设计
CAttitudeControl::CAttitudeControl():workmode(1)
{
}

Eigen::Vector3d CAttitudeControl::RateDamping(const GyroScope& _Gyro, double Kp)
{

	Eigen::Vector3d Tcontrol  = -Kp* _Gyro.InstallMatrix.inverse() *DEG2RAD* _Gyro.Data;
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}
