#include "Satellite/Satellite.h"

Satellite::Satellite():Orbit(),Attitude()
{
	SatelliteTime = 1640966400000;
	//初始化敏感器开机时间
	_Gyro.LastRenewTime = SatelliteTime;
}

void Satellite::StateRenew(double SampleTime)
{
	//更新时间、轨道、姿态和控制力矩
	SatelliteTime += (int64_t)(SampleTime * 1e3);
	Orbit.TwoBod(SampleTime);
	Attitude.AttitudeDynamicsRk4(SampleTime);
	Attitude.AttitudeKinematics(SampleTime);
	_Gyro.StateRenew(SatelliteTime, Attitude.Omega_b);
	Attitude.TotalTorque = AttitudeControl.ControlCommand(_Gyro.Data, _Gyro.InstallMatrix);
}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;
	_cout << "Qib " << Sat.Attitude.Qib;
	_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	return _cout;
}
