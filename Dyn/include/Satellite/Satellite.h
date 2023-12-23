#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"
#include "AStro/AttitudeControl.h"
#include"Componet/Componet.h"


class Satellite
{
public:
	int64_t SatelliteTime;//星历，utc时间戳单位ms
	COrbit Orbit;//轨道
	CAttitude Attitude;//姿态
	Environment Env;//环境
	CComponet* pComponet;//单机
	CAttitudeController AttController;//姿态控制


public:
	Satellite();

	~Satellite()=default;
	void StateRenew(double SampleTime);
	// 2023-12-22 11:28:55
	void dataToDB(CInfluxDB& DB, double Period);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);