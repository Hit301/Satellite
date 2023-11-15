#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Satellite/Gyro.h"
#include "Astro/Environment.h"
class Satellite
{
public:
	int64_t SatelliteTime;//星历，utc时间戳单位ms
	Inl_t J2000;//惯性系RV，单位m和m/s
	Eigen::Vector3d Omega_b;//本体系角速度，单位rad/s
	Quat Qib;//归一化惯性系到本体系四元数
	Eigen::Matrix3d SatInaMat;//本体系惯量矩阵，单位kgm2
	Eigen::Vector3d WheelMomentum;//飞伦组在本体系下的角动量，单位Nms
	Eigen::Vector3d TotalTorque;//Tf：干扰力矩：TB 磁力矩：Tw：飞轮本体系力矩 TotalTorque=TB+Tf-Tw
	Environment Env;
	GyroScope _Gyro;
public:
	Satellite();

	~Satellite()=default;
	void StateRenew(double SampleTime);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);