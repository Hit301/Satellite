#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"


class CAttitudeController
{
public:
	int workmode;//姿态控制模式
	Eigen::Vector3d TorqueRef;//参考力矩Nm
	Eigen::Matrix3d Kp;//控制器系数
	Eigen::Matrix3d Kd;//控制器系数
	double MaxTorque;//最大力矩

	enum Mode
	{
		RATEDAMP,
		SUNPOINT,
		EARTHPOINT
	};
public:
	CAttitudeController();

	//根据任务模式计算参考力矩，实际上应该只需要单机信息
	Eigen::Vector3d TorqueRefRenew(CAttitude& Att, COrbit& Obt, Environment& Env, CComponet* pCom);
private:

	//@brief: 速率阻尼控制器
	void RateDamping(const GyroScope& _Gyro);

	//@brief: 对日姿态控制器
	void ToSunControl(const GyroScope& _Gyro, Quat& _Qib, Eigen::Vector3d& _SunPos);

	//@brief: 对地捕获与定向控制率控制器
	void ToEarthControl(const GyroScope& _Gyro, Quat& _Qob);

};

