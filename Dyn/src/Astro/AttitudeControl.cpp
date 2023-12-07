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


//对日捕获与定向控制率设计
Eigen::Vector3d CAttitudeControl::SunControl(const GyroScope& _Gyro, double Kp, Quat _Qib, Eigen::Vector3d _SunPos)
{
	//计算从惯性系到本体系的姿态转移矩阵
	CDcm Aib = _Qib.ToDcm();
	//计算本体系下的太阳矢量
	Eigen::Vector3d Sunb = Aib * _SunPos;
	//太阳矢量反向
	Sunb = -Sunb;
	double sunb = sqrt(Sunb[0] * Sunb[0] + Sunb[1] * Sunb[1]);
	//卫星可实现的最大转速
	double Ws = HALFPI / 180;
	//对日捕获目标姿态角速度
	Eigen::Vector3d Wref(Sunb[1], Sunb[0], 0);
	Wref = Wref / sunb * Ws;
	Eigen::Vector3d Tcontrol = -Kp * _Gyro.InstallMatrix.inverse() * DEG2RAD * (Wref -_Gyro.Data);
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}

//对地捕获与定向控制率设计
//Eigen::Vector3d CAttitudeControl::ToEarthControl()
//{
//
//}