#include "Gyro.h"

GyroScope::GyroScope()
{
	InstallMatrix << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
	Data << 0, 0, 0;
	LastRenewTime = 0;
	SamplePeriod = 0.1;
}

void GyroScope::StateRenew(int64_t NowTime, Eigen::Vector3d Omega_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod*1e3)
	{
		Data = InstallMatrix * Omega_b;
		Data = DEG(Data);
		LastRenewTime = NowTime;
	}
}

void GyroScope::Init(Eigen::Vector3d& Omega_b, int64_t timestamp)
{
	LastRenewTime = timestamp;
	Data = InstallMatrix * Omega_b;
	Data = DEG(Data);
}

void GyroScope::FaultInjection(double FaultSize, FaultTypes FaultType, int64_t InjectGyroHead, int duration)
{	
	/*
	* FaultSize: ���Թ���ʱ��λΪ�Ƕ�
	* FaultType: enum����
	* InjectGyroHead: ȡֵ��1��ʼ
	*/

	// 20231223 �������ݰ���3�����ݱ�ͷ��������
	if (InjectGyroHead > 3)
	{
		std::cout << "����ע������ݱ�ͷ��ŷǷ� ֵ= " << InjectGyroHead << "��ֵӦС�ڵ���3" << std::endl;
		return;
	}
	if (FaultType == Abrupt)
	{
		Data[InjectGyroHead-1] = Data[InjectGyroHead-1] + FaultSize;
	}
	else if (FaultType == Slow)
	{
		int randomNum = rand() % 11;
		float lambda2 = (95 + randomNum) / 100;
		float kg = 1e-2;
		Data[InjectGyroHead - 1] = Data[InjectGyroHead - 1] + lambda2 * kg * duration;
	}
}
