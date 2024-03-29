#include "Orbit.h"
#include "Environment.h"
#include"Attitude.h"
#include"Quaternions.h"
#include"CConfig.h"
#include"InfluxDB.h"
int COrbit::TwoBodRK4(double Ts)
{
	if (IsRV(J2000Inertial) == false)
	{
		printf("���RV���Ϸ���R:%f(m) V:%f(m/s)\n", J2000Inertial.Pos.norm(), J2000Inertial.Vel.norm());
		return -1;
	}
	Eigen::VectorXd RVState(6, 1);
	RVState.block<3, 1>(0, 0) = J2000Inertial.Pos;
	RVState.block<3, 1>(3, 0) = J2000Inertial.Vel;

	Eigen::VectorXd k1(6, 1), k2(6, 1), k3(6, 1), k4(6, 1);
	k1 = TwoBodAcc(RVState);
	k2 = TwoBodAcc(RVState + k1 * (0.5 * Ts));
	k3 = TwoBodAcc(RVState + k2 * (0.5 * Ts));
	k4 = TwoBodAcc(RVState + k3 * Ts);

	RVState = RVState + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
	J2000Inertial.Pos = RVState.block<3, 1>(0, 0);
	J2000Inertial.Vel = RVState.block<3, 1>(3, 0);
	return 0;

}

void COrbit::Inl2Fix(const int64_t timestamp)
{
	//@brief: ����ϵλ���ٶ�ת�ع�ϵλ���ٶ�
	//@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
	//@return : none
	//@remark : �Ѳ���
	Eigen::Matrix3d Aif;
	Aif = Environment::ECI2ECEF(timestamp);
	ECEFFix.Pos = Aif * J2000Inertial.Pos;
	Eigen::Vector3d EarthAngularVelocityFixed;
	EarthAngularVelocityFixed << 0, 0, EARTH_RATE;
	ECEFFix.Vel = Aif * J2000Inertial.Vel - EarthAngularVelocityFixed.cross(ECEFFix.Pos);
}

void COrbit::FixPos2LLA()
{
	//@brief: �ع�ϵ�������LLA
	//@para : none
	//@return : none
	//@remark : �Ѳ���
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
	//@brief: �ع�ϵ�������LLR
	//@para : none
	//@return : none
	//@remark : �Ѳ���
	LLR.Lng = ATAN2(ECEFFix.Pos(1), ECEFFix.Pos(0));
	LLR.Lat = ATAN2(ECEFFix.Pos(2), SQRT(ECEFFix.Pos(0) * ECEFFix.Pos(0) + ECEFFix.Pos(1) * ECEFFix.Pos(1)));
	LLR.Rds = ECEFFix.Pos.norm();
}

Eigen::Matrix3d COrbit::NED2ECEF()
{
	//@brief: ���㱱����ϵ���ع�ϵ��ת�ƾ���
	//@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
	//@return : none
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
	return temres;
}

void COrbit::StateRenew(double Ts, const int64_t timestamp)
{
	TwoBodRK4(Ts);
	Inl2Fix(timestamp);
	FixPos2LLR();
	FixPos2LLA();
}

void COrbit::Init(int64_t Timestamp)
{
	CConfig* pCfg = CConfig::GetInstance();
	J2000Inertial.Pos << pCfg->Rx, pCfg->Ry, pCfg->Rz;
	J2000Inertial.Vel << pCfg->Vx, pCfg->Vy, pCfg->Vz;
	Inl2Fix(Timestamp);
	FixPos2LLR();
	FixPos2LLA();
}


std::ostream& operator<<(std::ostream& _cout, const RV& j2000)
{
	_cout << "J2000 Pos(km) " << j2000.Pos(0) / 1000 << " " << j2000.Pos(1) / 1000 << " " << j2000.Pos(2) / 1000 << std::endl;
	_cout << "J2000 Vel(km/s) " << j2000.Vel(0) / 1000 << " " << j2000.Vel(1) / 1000 << " " << j2000.Vel(2) / 1000 << std::endl;
	return _cout;
}

void COrbit::record(CInfluxDB& DB) {
	// ����ϵRV
	DB.addKeyValue("SIM009", J2000Inertial.Pos.x());
	DB.addKeyValue("SIM010", J2000Inertial.Pos.y());
	DB.addKeyValue("SIM011", J2000Inertial.Pos.z());
	DB.addKeyValue("SIM012", J2000Inertial.Vel.x());
	DB.addKeyValue("SIM013", J2000Inertial.Vel.y());
	DB.addKeyValue("SIM014", J2000Inertial.Vel.z());
	// �ع�ϵRV
	DB.addKeyValue("SIM015", ECEFFix.Pos.x());
	DB.addKeyValue("SIM016", ECEFFix.Pos.y());
	DB.addKeyValue("SIM017", ECEFFix.Pos.z());
	DB.addKeyValue("SIM018", ECEFFix.Vel.x());
	DB.addKeyValue("SIM019", ECEFFix.Vel.y());
	DB.addKeyValue("SIM020", ECEFFix.Vel.z());
	// ��������
	DB.addKeyValue("SIM021", RAD_PI(LLA.Lng) * RAD2DEG);
	DB.addKeyValue("SIM022", RAD_PI(LLA.Lat) * RAD2DEG);
	DB.addKeyValue("SIM023", LLA.Alt);
}

Eigen::VectorXd COrbit::TwoBodAcc(const Eigen::VectorXd& RVState)
{
	Eigen::VectorXd RVStateAcc(6,1);
	RVStateAcc.setZero(6, 1);

	if (RVState.size() != 6)
	{
		printf("invalid size in RVState\r\n");
		return RVStateAcc;
	}

	Eigen::MatrixXd Accmatrix(6, 6);
	Accmatrix.setZero();
	double tmp = 1 / J2000Inertial.Pos.norm();
	double tmp2 = -EARTH_GRAVITATIONAL * tmp * tmp * tmp;
	Accmatrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
	Accmatrix.block<3, 3>(3, 0) = tmp2*Eigen::Matrix3d::Identity();

	RVStateAcc = Accmatrix * RVState;
	return RVStateAcc;
}

