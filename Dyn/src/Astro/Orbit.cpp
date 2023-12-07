#include "Astro/Orbit.h"
#include "Astro/Environment.h"

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
	Aif = Environment::ECI2ECEF(timestamp, deltaUT1, xp, yp);
	Wgs84Fix.Pos = Aif * J2000Inertial.Pos;
	Eigen::Vector3d EarthAngularVelocityFixed;
	EarthAngularVelocityFixed << 0, 0, EARTH_RATE;
	Wgs84Fix.Vel = Aif * J2000Inertial.Vel - EarthAngularVelocityFixed.cross(Wgs84Fix.Pos) ;
}

void COrbit::FixPos2LLA()
{
	//@brief: �ع�ϵ�������LLA
	//@para : none
	//@return : none
	double sqrt_x2y2 = SQRT(Wgs84Fix.Pos(0) * Wgs84Fix.Pos(0) + Wgs84Fix.Pos(1) * Wgs84Fix.Pos(1));
	double e2 = 1.0 - (EARTH_POLAR_RADIUS * EARTH_POLAR_RADIUS) / (EARTH_EQUATORIAL_RADIUS * EARTH_EQUATORIAL_RADIUS);
	double e_2 = (EARTH_EQUATORIAL_RADIUS * EARTH_EQUATORIAL_RADIUS) / (EARTH_POLAR_RADIUS * EARTH_POLAR_RADIUS) - 1.0;
	double belta = ATAN2(Wgs84Fix.Pos(2) * EARTH_EQUATORIAL_RADIUS , EARTH_POLAR_RADIUS * sqrt_x2y2);
	double Lat_tmp = sqrt_x2y2 - e2 * EARTH_EQUATORIAL_RADIUS * POW(cos(belta), 3);
	LLA.Lng = ATAN2(Wgs84Fix.Pos(1), Wgs84Fix.Pos(0));
	LLA.Lat = ATAN2(Wgs84Fix.Pos(2) + e_2 * EARTH_EQUATORIAL_RADIUS * POW(sin(belta), 3), Lat_tmp);
	double cosB = cos(LLA.Lat);
	double sinB = sin(LLA.Lat);
	if (cosB != 0)
	{
		LLA.Lat = sqrt_x2y2 / cosB - (EARTH_EQUATORIAL_RADIUS / (SQRT(1.0 - e2 * sinB * sinB)));
	}
	else
	{
		LLA.Lat = Wgs84Fix.Pos(2) - EARTH_POLAR_RADIUS * SIGN(Wgs84Fix.Pos(2));
	}
}

void COrbit::FixPos2LLR()
{
	//@brief: �ع�ϵ�������LLR
	//@para : none
	//@return : none
	//@remark : δ����
	LLR.Lng = ATAN2(Wgs84Fix.Pos(1), Wgs84Fix.Pos(0));
	LLR.Lat = ATAN2(Wgs84Fix.Pos(2), SQRT(Wgs84Fix.Pos(0) * Wgs84Fix.Pos(0) + Wgs84Fix.Pos(1) * Wgs84Fix.Pos(1)));
	LLR.Rds = Wgs84Fix.Pos.norm();
}

Eigen::Matrix3d COrbit::NED2ECEF()
{
	//@brief: ���㱱����ϵ���ع�ϵ��ת�ƾ���
	//@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
	//@return : none
	//@remark : static
	/*��λ*/
	Eigen::Matrix3d res;
	Eigen::Matrix3d temres;
	double sin_lng = sin(LLR.Lng);
	double cos_lng = cos(LLR.Lng);
	double sin_lat = sin(LLR.Lat);
	double cos_lat = cos(LLR.Lat);
	res << -sin_lat * cos_lng, -sin_lat * sin_lng,  cos_lat,
		   -sin_lng,                  cos_lng,         0,
		   -cos_lat * cos_lng, -cos_lat * sin_lng, -sin_lat;
	temres = res.inverse();
	return res;
}


std::ostream& operator<<(std::ostream& _cout, const RV& j2000)
{
	_cout << "J2000 Pos(km) " << j2000.Pos(0) / 1000 << " " << j2000.Pos(1) / 1000 << " " << j2000.Pos(2) / 1000 << std::endl;
	_cout << "J2000 Vel(km/s) " << j2000.Vel(0) / 1000 << " " << j2000.Vel(1) / 1000 << " " << j2000.Vel(2) / 1000 << std::endl;
	return _cout;
}
