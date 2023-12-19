#pragma once
#include"SatelliteMath/BaseMath.h"
class SunSensor
{
public:
	Eigen::Matrix3d InstallMatrix;//安装矩阵,从本体系到测量系
	Eigen::Vector3d Data;//太敏测量三轴太阳矢量数值，单位deg/s
	int64_t LastRenewTime;//开机时间，utc时间戳单位ms
	double SamplePeriod;//太敏采样周期，单位为s
public:
	SunSensor();

public:
	void StateRenew(int64_t NowTime, Eigen::Vector3d Sun_b);

};
