#pragma once
#include"SatelliteMath/BaseMath.h"
#include"SatelliteMath/Quaternions.h"
#include"SatelliteMath/Dcm.h"
class StarSensorScope
{
public:
	CDcm InstallMatrix;//��װ����,�ӱ���ϵ������ϵ
	Quat Data;//������ֵ
	int64_t LastRenewTime;//����ʱ�䣬utcʱ�����λms
	double SamplePeriod;//���ݲ������ڣ���λΪs
public:
	StarSensorScope();

public:
	void StateRenew(int64_t NowTime, Quat Quat_b);

};

