#pragma once
#include"SatelliteMath/BaseMath.h"

class MagSensor
{
public:
	Eigen::Matrix3d InstallMatrix;//��װ����,�ӱ���ϵ������ϵ
	Eigen::Vector3d Data;//����ͷ��ֵ����λdeg/s
	int64_t LastRenewTime;//����ʱ�䣬utcʱ�����λms
	double SamplePeriod;//���ݲ������ڣ���λΪs
public:
	MagSensor();

public:
	void StateRenew(int64_t NowTime, Eigen::Vector3d Omega_b);

};