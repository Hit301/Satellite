#pragma once
#include"SatelliteMath/BaseMath.h"
class Flywheel
{
public:
	Eigen::Vector3d InstallVet;//安装向量，从本体系到安装系
	double Speed;//飞轮转速数值，单位rad/s
	double Torque;//实际力矩，单位Nm
	double Momentum;//角动量 单位Nms
	int64_t LastRenewTime;//上次更新时间,utc时间戳ms
	double MaxTref;//最大参考力矩Nm
	double MaxSpeed;//最大转速
	double SpeedRef;//参考转速rad/s
	double TorqueRef;//参考力矩单位Nm
	//
	double Kp;//飞轮控制器Kp
	double Ki;//飞轮控制器Ki
	double tau;//飞轮模型时间常数
	double J;//飞轮转动惯量 kgm2
public:
	Flywheel();
	Flywheel(Eigen::Vector3d& InsVet);
public:
	void StateRenew(int64_t NowTime, double SampleTime, double Tref = 0);
	void Init(double speed, int64_t timestamp);
};

