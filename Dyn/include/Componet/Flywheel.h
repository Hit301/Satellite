#pragma once
#include"SatelliteMath/BaseMath.h"
class flywheel
{
public:
	Eigen::Vector3d InstallVet;//��װ�������ӱ���ϵ����װϵ
	double Speed;//����ת����ֵ����λdeg/s
	double Torque;//���أ���λN
	int64_t LastRenewTime;//�ϴθ���ʱ��,utcʱ���ms
	double SamplePeriod;//���ݲ������ڣ���λΪs

	//
	double Kp;
	double Ki;
	double tao;
	double J;
public:
	flywheel();

public:
	void StateRenew(int64_t NowTime, double Tref=0);
	void Init(double speed, int64_t timestamp);


};

