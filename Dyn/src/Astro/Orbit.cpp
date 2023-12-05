#include "Astro/Orbit.h"

int COrbit::TwoBod(double Ts)
{
	if (IsRV(J2000Inertial) == false)
	{
		printf("轨道RV不合法，R:%f(m) V:%f(m/s)\n", J2000Inertial.Pos.norm(), J2000Inertial.Vel.norm());
		return -1;
	}
	else
	{
	double tmp = 1/J2000Inertial.Pos.norm();
	double tmp2 = -EARTH_GRAVITATIONAL * tmp * tmp * tmp;
	J2000Inertial.Vel += Ts * tmp2 * J2000Inertial.Pos;
	J2000Inertial.Pos += Ts * J2000Inertial.Vel;
	return 0;
	}
}

CDcm COrbit::TransferMatrix(Quat _Qib)
{
	return quat.ToDcm();
}


std::ostream& operator<<(std::ostream& _cout, const RV& j2000)
{
	_cout << "J2000 Pos(km) " << j2000.Pos(0) / 1000 << " " << j2000.Pos(1) / 1000 << " " << j2000.Pos(2) / 1000 << std::endl;
	_cout << "J2000 Vel(km/s) " << j2000.Vel(0) / 1000 << " " << j2000.Vel(1) / 1000 << " " << j2000.Vel(2) / 1000 << std::endl;
	return _cout;
}
