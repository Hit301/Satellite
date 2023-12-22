#include "AStro/AttitudeControl.h"
#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/Quaternions.h"
#include"Astro/Orbit.h"
#include"Astro/Attitude.h"
#include"Componet/Componet.h"
#include "Astro/Environment.h"
//初始速率阻尼控制率设计
CAttitudeController::CAttitudeController() :workmode(1)
{
	TorqueRef << 0, 0, 0;
	Kp << 0.5 * Eigen::Matrix3d::Identity();
	Kd << 3 * Eigen::Matrix3d::Identity();
	MaxTorque = 0.08;
}

Eigen::Vector3d CAttitudeController::TorqueRefRenew(CAttitude& Att, COrbit& Obt, Environment& Env, CComponet* pCom)
{
	//这里默认用第一个单机了，后面有需要可以更改，实际上应该有定姿的算法，在定姿算法中算出结果
	switch (workmode)
	{
	case RATEDAMP:
	{
		RateDamping(pCom->pGyro[0]);
	}
	break;
	case SUNPOINT:
	{
		ToSunControl(pCom->pGyro[0], Att.Qib, Env.SunVecBody);
	}
	break;
	case EARTHPOINT:
	{

		ToEarthControl(pCom->pGyro[0], Att.Qob);
	}
	break;
	default:
		TorqueRef << 0, 0, 0;
		break;
	}
	return TorqueRef;
}

void CAttitudeController::RateDamping(const GyroScope& _Gyro)
{

	Eigen::Vector3d Tcontrol = -Kp * _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}

//对日捕获与定向控制率设计
void CAttitudeController::ToSunControl(const GyroScope& _Gyro, Quat& _Qib, Eigen::Vector3d& _SunPos)
{
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;

	//参考量
	Eigen::Vector3d Wref(0, 0, 0.1 * DEG2RAD);
	Eigen::Vector3d Rb(0, 0, 1);
	//控制力矩计算
	Eigen::Vector3d Tcontrol = -Kp * _SunPos.cross(Rb) + Kd * (Eigen::Vector3d::Zero() - Wbi);
	//限幅处理
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}

//对地捕获与定向控制率设计
void CAttitudeController::ToEarthControl(const GyroScope& _Gyro, Quat& _Qob)
{
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	Eigen::Vector3d ImQob;
	ImQob << _Qob.QuatData[1], _Qob.QuatData[2], _Qob.QuatData[3];

	//控制力矩计算
	Eigen::Vector3d Tcontrol = -Kp * ImQob - Kd * Wbi;
	//限幅处理
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}