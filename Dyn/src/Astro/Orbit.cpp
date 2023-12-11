#include "Astro/Orbit.h"
#include "Astro/Environment.h"
#include"Astro/Attitude.h"
#include"SatelliteMath/Quaternions.h"

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

void COrbit::Inl2Fix(const int64_t timestamp)
{
	//@brief: 惯性系位置速度转地固系位置速度
	//@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
	//@return : none
	//@remark : 已测试
	Eigen::Matrix3d Aif;
	Aif = Environment::ECI2ECEF(timestamp);
	ECEFFix.Pos = Aif * J2000Inertial.Pos;
	Eigen::Vector3d EarthAngularVelocityFixed;
	EarthAngularVelocityFixed << 0, 0, EARTH_RATE;
	ECEFFix.Vel = Aif * J2000Inertial.Vel - EarthAngularVelocityFixed.cross(ECEFFix.Pos) ;
}

void COrbit::FixPos2LLA()
{
	//@brief: 地固系轨道计算LLA
	//@para : none
	//@return : none
	//@remark : 已测试
	double sqrt_x2y2 = SQRT(ECEFFix.Pos(0) * ECEFFix.Pos(0) + ECEFFix.Pos(1) * ECEFFix.Pos(1));
	double e2 = 1.0 - (EARTH_POLAR_RADIUS * EARTH_POLAR_RADIUS) / (EARTH_EQUATORIAL_RADIUS * EARTH_EQUATORIAL_RADIUS);
	double e_2 = (EARTH_EQUATORIAL_RADIUS * EARTH_EQUATORIAL_RADIUS) / (EARTH_POLAR_RADIUS * EARTH_POLAR_RADIUS) - 1.0;
	double belta = ATAN2(ECEFFix.Pos(2) * EARTH_EQUATORIAL_RADIUS , EARTH_POLAR_RADIUS * sqrt_x2y2);
	double Lat_tmp = sqrt_x2y2 - e2 * EARTH_EQUATORIAL_RADIUS * POW(cos(belta), 3);
	LLA.Lng = ATAN2(ECEFFix.Pos(1), ECEFFix.Pos(0));
	LLA.Lat = ATAN2(ECEFFix.Pos(2) + e_2 * EARTH_EQUATORIAL_RADIUS * POW(sin(belta), 3), Lat_tmp);
	double cosB = cos(LLA.Lat);
	double sinB = sin(LLA.Lat);
	if (cosB != 0)
	{
		LLA.Alt = sqrt_x2y2 / cosB - (EARTH_EQUATORIAL_RADIUS / (SQRT(1.0 - e2 * sinB * sinB)));
	}
	else
	{
		LLA.Alt = ECEFFix.Pos(2) - EARTH_POLAR_RADIUS * SIGN(ECEFFix.Pos(2));
	}
}

void COrbit::FixPos2LLR()
{
	//@brief: 地固系轨道计算LLR
	//@para : none
	//@return : none
	//@remark : 已测试
	LLR.Lng = ATAN2(ECEFFix.Pos(1), ECEFFix.Pos(0));
	LLR.Lat = ATAN2(ECEFFix.Pos(2), SQRT(ECEFFix.Pos(0) * ECEFFix.Pos(0) + ECEFFix.Pos(1) * ECEFFix.Pos(1)));
	LLR.Rds = ECEFFix.Pos.norm();
}

Eigen::Matrix3d COrbit::NED2ECEF()
{
	//@brief: 计算北东地系到地固系的转移矩阵
	//@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
	//@return : none
	/*单位*/
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
	return temres;
}

void COrbit::StateRenew(double Ts, const int64_t timestamp)
{
	TwoBod(Ts);
	Inl2Fix(timestamp);
	FixPos2LLR();
	FixPos2LLA();
}


std::ostream& operator<<(std::ostream& _cout, const RV& j2000)
{
	_cout << "J2000 Pos(km) " << j2000.Pos(0) / 1000 << " " << j2000.Pos(1) / 1000 << " " << j2000.Pos(2) / 1000 << std::endl;
	_cout << "J2000 Vel(km/s) " << j2000.Vel(0) / 1000 << " " << j2000.Vel(1) / 1000 << " " << j2000.Vel(2) / 1000 << std::endl;
	return _cout;
}
