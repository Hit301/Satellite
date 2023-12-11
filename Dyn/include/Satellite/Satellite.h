#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Satellite/Gyro.h"
#include "Astro/Environment.h"
#include "AStro/AttitudeControl.h"
class Satellite
{
public:
	int64_t SatelliteTime;//星历，utc时间戳单位ms
	COrbit Orbit;//轨道
	CAttitude Attitude;//姿态
	Environment Env;//环境
	GyroScope _Gyro;//陀螺
	CAttitudeControl AttitudeControl;//姿态控制
	CSunCal SunCal;//太阳矢量（测试）
	Eigen::Vector3d SolVec;//太阳矢量（测试）
	double Angle; // 太阳矢量与卫星 - z轴夹角（单位:deg）
	Quat _Qbo;//轨道系下的姿态四元数

	// 定义模式标志变量
	bool sunControlMode;
	bool earthControlMode;

public:
	Satellite();

	~Satellite()=default;
	void StateRenew(double SampleTime);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);