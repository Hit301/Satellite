#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"
#include"SatelliteMath/Quaternions.h"
#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/EulerAgl.h"
#include "Astro/Orbit.h"

class CAttitude
{
public:

	Eigen::Vector3d Omega_b;//本体系角速度，单位rad/s
	CDcm Aio;//惯性系转轨道系转移矩阵
	Quat Qib;//惯性系到本体系四元数
	Quat Qob;//轨道系到本体系四元数
private:
	Eigen::Matrix3d SatInaMat;//本体系惯量矩阵，单位kgm2
	Eigen::Vector3d WheelMomentum_b;//飞轮组在本体系下的角动量，单位Nms
	Eigen::Vector3d TotalTorque;//Tf：干扰力矩：TB 磁力矩：Tw：飞轮本体系力矩 TotalTorque=TB+Tf-Tw
public:
	//
	// brief  : 默认姿态类构造函数
	//
	CAttitude();

	//
	// brief  : 姿态动力学递推角速度
	//
	int AttitudeDynamicsRk4(double Ts);

	//
	// brief  : 姿态运动递推四元数
	//
	int AttitudeKinematics(double Ts);

	void StateRenew(double Ts, COrbit& Orbit, CComponet* pComponet);

	void Init(COrbit& Obt);

	// 写入数据库
	void record(CInfluxDB& DB);

	static CDcm GetAio(const RV& InlRv);
private:
	Eigen::Vector3d LastOmega_b;//上一拍的本体系角速度，单位rad/s
	void RenewAio(COrbit& Orbit);
};


