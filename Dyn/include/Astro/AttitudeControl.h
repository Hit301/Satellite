#pragma once
#include"SatelliteMath/BaseMath.h"
#include"Satellite/Gyro.h"

class AttitudeControl
{
public:
	int workmode;
	double Kp;
	Eigen::Vector3d Tcontrol;

public:
	AttitudeControl();

public:
	//��ʼ����������������

	Eigen::Vector3d ControlCommand(Eigen::Vector3d _data, Eigen::Matrix3d _InstallMatrix);
};

