#include "Satellite/Satellite.h"

Satellite::Satellite() :Orbit(), Attitude(), _Gyro()
{
	SatelliteTime = 1640966400000;
	//初始化敏感器开机时间
	_Gyro.LastRenewTime = SatelliteTime;
	_Gyro.Data = _Gyro.InstallMatrix * Attitude.Omega_b;
	Angle = 10;
	// 定义模式标志变量
	sunControlMode = false;
	earthControlMode = false;
}


void Satellite::StateRenew(double SampleTime)
{
	//时间戳更新
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//控制器计算
	Attitude.TotalTorque = CAttitudeControl::RateDamping(_Gyro, 3);

	//轨道相关信息更新
	Orbit.StateRenew(SampleTime, SatelliteTime);

	//姿态相关信息更新
	Attitude.StateRenew(SampleTime);

	//环境信息更新
	Env.StateRenew(Orbit, SatelliteTime);

	//单机数据更新
	_Gyro.StateRenew(SatelliteTime, Attitude.Omega_b);

}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;//初始速率阻尼测试
	_cout << "Qib " << Sat.Attitude.Qib;
	_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	_cout << "SolVec(m) " << Sat.SolVec(0) << " " << Sat.SolVec(1) << " " << Sat.SolVec(2) << std::endl;//惯性系下太阳矢量测试
	_cout << "SolAngle(m) " << Sat.Angle << std::endl;//对日姿态控制测试
	_cout << "_Qob " << Sat._Qbo;//对日姿态控制测试
	return _cout;
}
