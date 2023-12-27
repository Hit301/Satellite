#include "AStro/AttitudeControl.h"
#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/Quaternions.h"
#include"Astro/Orbit.h"
#include"Astro/Attitude.h"
#include"Componet/Componet.h"
#include "Astro/Environment.h"
#include"General/InfluxDB.h"

//初始速率阻尼控制率设计
CAttitudeController::CAttitudeController() :workmode(EARTHPOINT)
{
	TorqueRef << 0, 0, 0;
	Kp << 0.5 * Eigen::Matrix3d::Identity();
	Kd << 3 * Eigen::Matrix3d::Identity();
	MaxTorque = 0.08;
}

Eigen::Vector3d CAttitudeController::TorqueRefRenew(CComponet* pCom)
{
	//这里默认用第一个单机了，后面有需要可以更改，实际上应该有定姿的算法，在定姿算法中算出结果
	switch (workmode)
	{
	case RATEDAMP:
	{
		RateDamping(pCom->Gyros[0]);
	}
	break;
	case SUNPOINT:
	{
		ToSunControl(pCom->Gyros[0], pCom->SunSensors[0]);
	}
	break;
	case EARTHPOINT:
	{

		ToEarthControl(pCom->Gyros[0], pCom->StarSensors[0], pCom->GNSSs[0]);
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
void CAttitudeController::ToSunControl(const GyroScope& _Gyro, const SunSensor& _Sun)
{
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;

	//参考量
	Eigen::Vector3d Wref(0, 0, 0.1 * DEG2RAD);
	Eigen::Vector3d Rb(0, 0, 1);
	Eigen::Vector3d _SunPos = _Sun.InstallMatrix.inverse() * _Sun.Data;
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
void CAttitudeController::ToEarthControl(const GyroScope& _Gyro, const StarSensor& _Star, const GNSS& _gnss)
{
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	Quat Qib = _Star.Data * _Star.InstallMatrix.ToQuat();

	//计算Aio
	CDcm Aio = CAttitude::GetAio(_gnss.Data);

	Quat Qoi = Aio.ToQuat().QuatInv();
	Quat Qob = Qoi * Qib;
	Eigen::Vector3d ImQob;
	ImQob << Qob.QuatData[1], Qob.QuatData[2], Qob.QuatData[3];

	//控制力矩计算
	Eigen::Vector3d Tcontrol = -Kp * ImQob - Kd * Wbi;
	//限幅处理
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}

void CAttitudeController::record(CInfluxDB& DB)
{	
	// 控制模式，InfluxDB中tag标签，存储字符串数据，field字段，存储数值数据
	DB.addKeyValue("SIM093", workmode);
	// 控制力矩
	DB.addKeyValue("SIM094", TorqueRef.x());
	DB.addKeyValue("SIM095", TorqueRef.y());
	DB.addKeyValue("SIM096", TorqueRef.z());
}