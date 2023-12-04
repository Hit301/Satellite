#include "Satellite/Satellite.h"
#include "AStro/AttitudeControl.h"

Satellite::Satellite() :Orbit(), Attitude(), _Gyro()
{
	SatelliteTime = 1640966400000;
	//初始化敏感器开机时间
	_Gyro.LastRenewTime = SatelliteTime;
	_Gyro.Data = _Gyro.InstallMatrix * Attitude.Omega_b;
}

void Satellite::StateRenew(double SampleTime)
{
	//环境信息更新
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//控制器计算
	Attitude.TotalTorque = CAttitudeControl::RateDamping(_Gyro, 3);

	//动力学更新
	Orbit.TwoBod(SampleTime);
	Attitude.AttitudeKinematics(SampleTime);
	Attitude.AttitudeDynamicsRk4(SampleTime);

	//单机数据更新
	_Gyro.StateRenew(SatelliteTime, Attitude.Omega_b);

	//太阳矢量测试
	//SolVec=SunCal.SunPos(SatelliteTime);
	SolVec = SunCal.SunPos(1659312000);//与MATLAB结果对比，存在差异
}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;
	_cout << "Qib " << Sat.Attitude.Qib;
	_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	_cout << "SolVec(m) " << Sat.SolVec(0) << " " << Sat.SolVec(1) << " " << Sat.SolVec(2) << std::endl;
	return _cout;
}
