#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"


class GyroScope
{
public:
	Eigen::Matrix3d InstallMatrix;//安装矩阵,从本体系到测量系
	Eigen::Vector3d Data;//三表头数值，单位deg/s
	int64_t LastRenewTime;//上次更新时间,utc时间戳ms
	double SamplePeriod;//陀螺采样周期，单位为s
public:
	GyroScope();

public:
	void StateRenew(int64_t NowTime, Eigen::Vector3d Omega_b);
	void Init(Eigen::Vector3d& Omega_b, int64_t timestamp);
};