#pragma once
#include"SatelliteMath/BaseMath.h"

class GyroScope
{
public:
	Eigen::Matrix3d InstallMatrix;//��װ����
	Eigen::Vector3d Data;//����ͷ��ֵ����λdeg/s
	GyroScope();
};