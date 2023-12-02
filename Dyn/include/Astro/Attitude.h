#pragma once
#include"SatelliteMath/EulerAgl.h"
#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/Quaternions.h"

class CAttitude
{
public:

	Eigen::Vector3d Omega_b;//本体系角速度，单位rad/s
	Quat Qib;//惯性系到本体系四元数
	Eigen::Matrix3d SatInaMat;//本体系惯量矩阵，单位kgm2
	Eigen::Vector3d WheelMomentum_b;//飞轮组在本体系下的角动量，单位Nms
	Eigen::Vector3d TotalTorque;//Tf：干扰力矩：TB 磁力矩：Tw：飞轮本体系力矩 TotalTorque=TB+Tf-Tw

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

};


