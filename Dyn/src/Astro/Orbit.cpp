#include "Astro/Orbit.h"

int COrbit::TwoBod(double Ts)
{
	if (IsRV(J2000Inertial) == false)
	{
		printf("���RV���Ϸ���R:%f(m) V:%f(m/s)\n", J2000Inertial.Pos.norm(), J2000Inertial.Vel.norm());
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

void COrbit::Inl2Fix(const int64_t timestamp, const double deltaUT1, const double xp, const double yp)
{
	//@brief: ����ϵλ���ٶ�ת�ع�ϵλ���ٶ�
	//@para : ����ϵRV
	//@return : none
	//@remark : δ����
	Eigen::Matrix3d Aif;
	Aif = Env.ECI2ECEF(timestamp, deltaUT1, xp, yp);
	Wgs84Fix.Pos = Aif * J2000Inertial.Pos;
	Eigen::Vector3d EarthAngularVelocityFixed;
	EarthAngularVelocityFixed << 0, 0, EARTH_RATE;
	Wgs84Fix.Vel = Aif * J2000Inertial.Vel - EarthAngularVelocityFixed.cross(Wgs84Fix.Pos) ;
}

void COrbit::FixPosToLLA()
{
	//@brief: �ع�ϵ�������LLA
	//@para : none
	//@return : none
	LLA.Lng = ATAN2(Wgs84Fix.Pos(1), Wgs84Fix.Pos(0));
}

Eigen::Matrix3d COrbit::NED2ECEF()
{
	//@brief: ���㱱����ϵ���ع�ϵ��ת�ƾ���
	//@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
	//@return : none
	//@remark : static
	Eigen::Matrix3d res;
	return res;
}

std::ostream& operator<<(std::ostream& _cout, const RV& j2000)
{
	_cout << "J2000 Pos(km) " << j2000.Pos(0) / 1000 << " " << j2000.Pos(1) / 1000 << " " << j2000.Pos(2) / 1000 << std::endl;
	_cout << "J2000 Vel(km/s) " << j2000.Vel(0) / 1000 << " " << j2000.Vel(1) / 1000 << " " << j2000.Vel(2) / 1000 << std::endl;
	return _cout;
}
