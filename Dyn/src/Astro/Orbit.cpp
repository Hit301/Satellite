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

void COrbit::Inl2Fix(const int64_t timestamp, const double deltaUT1, const double xp, const double yp)
{
	//@brief: 惯性系位置速度转地固系位置速度
	//@para : 惯性系RV
	//@return : none
	//@remark : 未测试
	Eigen::Matrix3d Aif;
	Aif = Env.ECI2ECEF(timestamp, deltaUT1, xp, yp);
	Wgs84Fix.Pos = Aif * J2000Inertial.Pos;
	Eigen::Vector3d EarthAngularVelocityFixed;
	EarthAngularVelocityFixed << 0, 0, EARTH_RATE;
	Wgs84Fix.Vel = Aif * J2000Inertial.Vel - EarthAngularVelocityFixed.cross(Wgs84Fix.Pos) ;
}

void COrbit::FixPosToLLA()
{
	//@brief: 地固系轨道计算LLA
	//@para : none
	//@return : none
	LLA.Lng = ATAN2(Wgs84Fix.Pos(1), Wgs84Fix.Pos(0));
}

Eigen::Matrix3d COrbit::NED2ECEF()
{
	//@brief: 计算北东地系到地固系的转移矩阵
	//@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
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
