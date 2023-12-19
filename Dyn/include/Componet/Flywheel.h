#pragma once
#include"SatelliteMath/BaseMath.h"
class flywheel
{
public:
	Eigen::Vector3d InstallVet;//安装向量，从本体系到安装系
	double Speed;//飞轮转速数值，单位deg/s
	double Torque;//力矩，单位N
	int64_t LastRenewTime;//上次更新时间,utc时间戳ms
	double SamplePeriod;//陀螺采样周期，单位为s

	//
	double Kp;
	double Ki;
	double tao;
	double J;
public:
	flywheel();

public:
	void StateRenew(int64_t NowTime, double Tref=0);
	void Init(double speed, int64_t timestamp);


};

