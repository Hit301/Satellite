#include "Astro/Orbit.h"

Inl_t TwoBod(Inl_t& J2000, double Ts)
{
	double tmp = 1/J2000.Pos.norm();
	double tmp2 = -EARTH_GRAVITATIONAL * tmp * tmp * tmp;
	Inl_t result(Ts*J2000.Vel+ J2000.Pos, Ts * tmp2 * J2000.Pos + J2000.Vel);
	return result;
}

std::ostream& operator<<(std::ostream& _cout, const Inl_t& j2000)
{
	_cout << "J2000 Pos(km) " << j2000.Pos(0) / 1000 << " " << j2000.Pos(1) / 1000 << " " << j2000.Pos(2) / 1000 << std::endl;
	_cout << "J2000 Vel(km/s) " << j2000.Vel(0) / 1000 << " " << j2000.Vel(1) / 1000 << " " << j2000.Vel(2) / 1000 << std::endl;
	return _cout;
}
