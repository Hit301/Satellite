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
	* FaultSize: 加性故障时单位为角度
	* FaultType: enum类型
	* FaultGyroHead: 取值从0开始
	*/

	// 20231223 以下内容按照3个陀螺表头的情况编程
	if (FaultGyroHead > 2)
	{
		std::cout << "故障注入的陀螺表头序号非法 值= " << FaultGyroHead << "，值应小于等于3" << std::endl;
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
			ConstantFaultData = Data; // 只执行一次
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

	// 故障时间和故障大小选取
	double randomNum = rand() / double(RAND_MAX); // 0-1浮点数
	double width = this->FaultSizeUp - this->FaultSizeLow;
	this->FaultSize = randomNum * width + this->FaultSizeLow; // low-up的浮点数

	randomNum = rand() / double(RAND_MAX); // 0-1浮点数
	width = this->FaultTimeUp - this->FaultTimeLow;
	this->FaultTime = randomNum * width + this->FaultTimeLow; // low-up的浮点数

	std::cout << "故障时间：" << this->FaultTime << ", 故障大小：" << this->FaultSize << std::endl;

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


