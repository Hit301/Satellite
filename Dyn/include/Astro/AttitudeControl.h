#pragma once
#include"SatelliteMath/BaseMath.h"
#include"Satellite/Gyro.h"

class CAttitudeControl
{
public:
	int workmode;
	double Kp;
	Eigen::Vector3d Tcontrol;

public:
	CAttitudeControl();

public:
	//初始速率阻尼控制率设计

	Eigen::Vector3d ControlCommand(Eigen::Vector3d _data, Eigen::Matrix3d _InstallMatrix);


	//控制指令的模式转换【待补充】

	//对日捕获与定向控制率设计
	//1.计算太阳在J2000坐标系下的位置



};

