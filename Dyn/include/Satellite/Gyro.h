#pragma once
#include "SatelliteMath/SatelliteEigen.h"

class GyroScope
{
public:
	Eigen::Matrix3d InstallMatrix;//��װ����
	Eigen::Vector3d Data;//����ͷ��ֵ����λdeg/s
	GyroScope();
};