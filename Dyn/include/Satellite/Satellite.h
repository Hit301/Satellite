#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Satellite/Gyro.h"
#include "Astro/Environment.h"
class Satellite
{
public:
	int64_t SatelliteTime;//星历，utc时间戳单位ms
	COrbit Orbit;//轨道
	CAttitude Attitude;//姿态
	Environment Env;//环境
	GyroScope _Gyro;//陀螺
	CAttitudeControl AttitudeControl;//控制力矩
public:
	Satellite();

	~Satellite()=default;
	void StateRenew(double SampleTime);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);