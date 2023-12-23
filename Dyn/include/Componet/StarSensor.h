#pragma once
#include"SatelliteMath/BaseMath.h"
#include"SatelliteMath/Quaternions.h"
#include"SatelliteMath/Dcm.h"
class StarSensor
{
public:
	CDcm InstallMatrix;//��װ����,�ӱ���ϵ������ϵ
	Quat Data;//������ֵ
	int64_t LastRenewTime;//����ʱ�䣬utcʱ�����λms
	double SamplePeriod;//���ݲ������ڣ���λΪs
public:
	StarSensor();

public:
	void StateRenew(int64_t NowTime, Quat Quat_b);
	void Init(Quat Quat_b, int64_t timestamp);

};

