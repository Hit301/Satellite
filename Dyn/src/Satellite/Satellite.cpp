#include "Satellite/Satellite.h"

Satellite::Satellite()
{
	SatelliteTime = 1640966400000;
	J2000.Pos(0) = 6678136.9999998566;
	J2000.Pos(1) = 0.0002095685;
	J2000.Pos(2) = -1.3800009224;

	J2000.Vel(0) = 0.0007615644;
	J2000.Vel(1) = 6789.5304738682;
	J2000.Vel(2) = 3686.4138485846;

	Omega_b(0) = 0.01;
	Omega_b(1) = -0.04;
	Omega_b(2) = 0;

	Qib.SetByIdx(0, 1);
	Qib.SetByIdx(1, 0);
	Qib.SetByIdx(2, 0);
	Qib.SetByIdx(3, 0);

	SatInaMat << 10, 0, 0,
				 0, 20, 0,
				 0, 0, 30;
	WheelMomentum << 0, 0, 0;
	TotalTorque << 0, 0, 0;
}

void Satellite::StateRenew(double SampleTime)
{
	//更新时间、轨道和姿态
	SatelliteTime += (int64_t)(SampleTime * 1e3);
	J2000 = TwoBod(J2000, SampleTime);
	Omega_b = Omega_bRK4(SatInaMat, Omega_b, WheelMomentum, TotalTorque, SampleTime);
	Qib = QibIntegrate(Qib, Omega_b, SampleTime);
}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << Sat.J2000;
	_cout << "Omega_b(rad/s) " << Sat.Omega_b(0) << " " << Sat.Omega_b(1) << " " << Sat.Omega_b(2) << std::endl;
	_cout << "Qib " << Sat.Qib;
	return _cout;
}
