#pragma once
#include"SatelliteMath/BaseMath.h"
#include"SatelliteMath/Quaternions.h"
#include"SatelliteMath/Dcm.h"
class StarSensorScope
{
public:
	CDcm InstallMatrix;//安装矩阵,从本体系到测量系
	Quat Data;//星敏数值
	int64_t LastRenewTime;//开机时间，utc时间戳单位ms
	double SamplePeriod;//陀螺采样周期，单位为s
public:
	StarSensorScope();

public:
	void StateRenew(int64_t NowTime, Quat Quat_b);

};

