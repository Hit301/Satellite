#include "Gyro.h"

GyroScope::GyroScope()
{
	InstallMatrix << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
	Data << 0, 0, 0;
	LastRenewTime = 0;
	SamplePeriod = 0.1;
	FaultGyroHead = 1;
	Duration = 0;
	ConstantFaultData << -1, -1, -1;

	FaultSize = -1;
	FaultTime = -1;
}

void GyroScope::StateRenew(int64_t NowTime, Eigen::Vector3d Omega_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod*1e3)
	{
		Data = InstallMatrix * Omega_b;
		Data = DEG(Data);
		LastRenewTime = NowTime;
		if ((NowTime - START_TIME) > FaultTime * 1e3)
		{
			FaultInjection();
		}
	}
}

void GyroScope::Init(Eigen::Vector3d& Omega_b, int64_t timestamp)
{
	LastRenewTime = timestamp;
	Data = InstallMatrix * Omega_b;
	Data = DEG(Data);
}

void GyroScope::FaultInjection()
{	
	/*
	* FaultSize: ���Թ���ʱ��λΪ�Ƕ�
	* FaultType: enum����
	* FaultGyroHead: ȡֵ��0��ʼ
	*/

	// 20231223 �������ݰ���3�����ݱ�ͷ��������
	if (FaultGyroHead > 2)
	{
		std::cout << "����ע������ݱ�ͷ��ŷǷ� ֵ= " << FaultGyroHead << "��ֵӦС�ڵ���3" << std::endl;
		return;
	}
	if (FaultType == Abrupt)
	{
		Data[FaultGyroHead] = Data[FaultGyroHead] + FaultSize;
	}
	else if (FaultType == Slow)
	{
		Duration += SamplePeriod;
		int randomNum = rand() % 11;
		float lambda2 = (95.0 + randomNum) / 100.0; // 0.95 - 1.05
		Data[FaultGyroHead] = Data[FaultGyroHead] + lambda2 * FaultSize * Duration;
	}
	else if (FaultType == Constant)
	{
		if (ConstantFaultData.data()[0] == -1)
		{
			ConstantFaultData = Data; // ִֻ��һ��
		}
		Data[FaultGyroHead] = ConstantFaultData[FaultGyroHead];
	}
}

void GyroScope::setFaultPara(double FaultSizeUp, double FaultSizeLow, int FaultTimeUp, int FaultTimeLow, int FaultType, int FaultGyroHead)
{
	this->FaultSizeUp = FaultSizeUp;
	this->FaultSizeLow = FaultSizeLow;
	this->FaultTimeUp = FaultTimeUp;
	this->FaultTimeLow = FaultTimeLow;

	// ����ʱ��͹��ϴ�Сѡȡ
	double randomNum = rand() / double(RAND_MAX); // 0-1������
	double width = this->FaultSizeUp - this->FaultSizeLow;
	this->FaultSize = randomNum * width + this->FaultSizeLow; // low-up�ĸ�����

	randomNum = rand() / double(RAND_MAX); // 0-1������
	width = this->FaultTimeUp - this->FaultTimeLow;
	this->FaultTime = randomNum * width + this->FaultTimeLow; // low-up�ĸ�����

	std::cout << "����ʱ�䣺" << this->FaultTime << ", ���ϴ�С��" << this->FaultSize << std::endl;

	if (FaultType == 0)
		this->FaultType = Abrupt;
	else if (FaultType == 1)
	{
		this->FaultType = Slow;
		this->Duration = 0;
	}
	else if (FaultType == 2)
	{
		this->FaultType = Constant;
		this->ConstantFaultData << -1, -1, -1;
	}
	
	this->FaultGyroHead = FaultGyroHead;
}


