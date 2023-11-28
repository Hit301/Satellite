#include "Satellite/Satellite.h"

Satellite::Satellite():Orbit(),Attitude()
{
	SatelliteTime = 1640966400000;

	//初始化敏感器开机时间
	_Gyro.LastRenewTime = SatelliteTime;
}

void Satellite::StateRenew(double SampleTime)
{
	//更新时间、轨道和姿态
	SatelliteTime += (int64_t)(SampleTime * 1e3);
	Orbit.TwoBod(SampleTime);
	Attitude.AttitudeDynamicsRk4(SampleTime);
	Attitude.AttitudeKinematics(SampleTime);
	_Gyro.StateRenew(SatelliteTime, Attitude.Omega_b);
}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;
	_cout << "Qib " << Sat.Attitude.Qib;
	return _cout;
}
