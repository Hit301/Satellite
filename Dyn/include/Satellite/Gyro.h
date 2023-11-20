#pragma once
#include"SatelliteMath/BaseMath.h"

class GyroScope
{
public:
	Eigen::Matrix3d InstallMatrix;//安装矩阵
	Eigen::Vector3d Data;//三表头数值，单位deg/s
	GyroScope();
};