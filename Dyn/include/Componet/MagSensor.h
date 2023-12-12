#pragma once
#include"SatelliteMath/BaseMath.h"

class MagSensor
{
public:
	Eigen::Matrix3d InstallMatrix;//安装矩阵,从本体系到测量系
	Eigen::Vector3d Data;//磁强计数值，单位
	int64_t LastRenewTime;//开机时间，utc时间戳单位ms
	double SamplePeriod;//采样周期，单位为s
public:
	MagSensor();

public:
	void StateRenew(int64_t NowTime, Eigen::Vector3d B_b);

};