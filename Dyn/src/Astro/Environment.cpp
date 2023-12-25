#include "Astro/Environment.h"
#include"MySofaDll.h"
#include"Astro/Orbit.h"
#include"Astro/Attitude.h"
#include"General/CConfig.h"
#include"General/InfluxDB.h"
Environment::Environment()
{
	BodyMag << 0, 0, 0;
	NEDMag << 0, 0, 0;
	SunVecInl << 0, 0, 0;
	SunVecBody << 0, 0, 0;
}

Eigen::Matrix3d Environment::ECI2ECEF(const int64_t timestamp, const double deltaUT1, const double xp, const double yp)
{
	double tmpres[3][3];
	GetECI2ECEF(timestamp, tmpres, deltaUT1, xp, yp);
	Eigen::Matrix3d res;
	res << tmpres[0][0], tmpres[0][1], tmpres[0][2],
		tmpres[1][0], tmpres[1][1], tmpres[1][2],
		tmpres[2][0], tmpres[2][1], tmpres[2][2];
	return res;
}

void Environment::SunPos(const int64_t timestamp)
{
	double TJD; double M; long double lamM; long double rs; double bs;
	Eigen::Matrix3d Rx;
	Eigen::Vector3d sunpos;

	//儒略世纪数
	TJD = TS2CEN(timestamp/1000);
	//TJD = 0.2258; //为了与MATLAB结果对比时使用
	//太阳平近点角，这里忽略了儒略世纪数的平方及以上项
	M = 357.5256 + 35999.049 * TJD;
	//太阳相对于当日平春分点的真近点角（黄道经度）
	lamM = 282.94 + M + SEC2DEG * 6892 * SIND(M) + SEC2DEG * 72 * SIND(M);
	//太阳地心距
	rs = (149.619 - 2.499 * COSD(M) - 0.021 * COSD(2.0 * M)) * 1e9;  //单位为m
	//23.4393是黄赤交角
	bs = 23.4393 - 46.815 / 3600 * TJD - 0.00059 / 3600 * TJD * TJD;
	sunpos << rs * COSD(lamM), rs* SIND(lamM), 0;  //单位为m
	Rx << 1, 0, 0,
		0, COSD(-bs), SIND(-bs),
		0, -SIND(-bs), COSD(-bs);

	SunVecInl = Rx * sunpos / rs;//归一化的结果
}

void Environment::GetNEDMag(const COrbit& Orbit, const int64_t timestamp)
{
	//@brief: 北东地系地磁场
	//@para : 轨道类轨道根数-半长轴 轨道类LLR
	//@return : none

	CConfig* pCfg = CConfig::GetInstance();
	size_t& Order = pCfg->MagOrder;

	//时间戳转年月日
	YMD m_ymd = UTCTimeStamp2YMD(timestamp);
	double epoch = DecYear(2020,1,1);
	double dt_change = DecYear(m_ymd.year, m_ymd.month, m_ymd.day) - epoch;

	Eigen::ArrayXXd g(Order + 1, Order + 1);
	Eigen::ArrayXXd h(Order + 1, Order + 1);
	g = pCfg->gauss_g + pCfg->gauss_gdot * dt_change;
	h = pCfg->gauss_h + pCfg->gauss_hdot * dt_change;

	Eigen::ArrayXXd P(Order + 2, Order + 2);
	P.setZero();
	double x = sin(Orbit.LLR.Lat);

	//计算帝合勒让德函数
	//计算对角线上元素
	P(0, 0) = 1;
	for (int n = 1; n < P.rows(); n++)
	{
		P(n, n) = (-1)*(2 * n - 1) * POW(1 - x * x, 0.5) * P(n - 1, n - 1);
	}
	
	//计算主对角线下一层元素
	for (int n = 0; n < P.rows()-1; n++)
	{
		P(n + 1, n) = x * (2 * n + 1) * P(n, n);
	}

	//计算非对角线元素
	for (int n = 2; n < P.rows(); n++)
	{
		for (int m = 0; m < n - 1; m++)
		{
			P(n, m) = 1.0 / (n - m) * (x * (2 * n - 1) * P((n - 1), m) - (n + m - 1) * P((n - 2), m));
		}
	}

	//对P中的每一个元素都进行施密特半归一化 
	for (int n = 0; n < P.rows(); n++)
	{
		for (int m = 0; m < n + 1; m++)
		{
			if (m > 0)
			{
				P(n, m) = POW(-1.0, m) * SQRT(2.0 * Factorial(n - m) / Factorial(n + m)) * P(n, m);
			}
		}
	}
	
	//计算dP
	Eigen::ArrayXXd dP(Order+1, Order+1);
	dP.setZero();
	for (int n = 0; n < dP.rows(); n++)
	{
		for (int m = 0; m < n + 1; m++)
		{
			dP(n, m) = (n + 1) * tan(Orbit.LLR.Lat) * P(n, m) - SQRT((n + 1) * (n + 1) - m * m) * (1 / cos(Orbit.LLR.Lat)) * P(n + 1, m);
		}
	}

	double X_prime = 0;
	double Y_prime = 0;
	double Z_prime = 0;
	double sumtempX = 0;
	double sumtempY = 0;
	double sumtempZ = 0;
	Eigen::ArrayXXd tempX(1, Order + 1);
	Eigen::ArrayXXd tempY(1, Order + 1);
	Eigen::ArrayXXd tempZ(1, Order + 1);

	for (int n = 1; n < Order + 1; n++)
	{
		tempX.setZero();
		tempY.setZero();
		tempZ.setZero();
		sumtempX = 0;
		sumtempY = 0;
		sumtempZ = 0;

		for (int m = 0; m < n + 1; m++)
		{
			tempX(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * dP(n, m);
			tempY(m) = m * (g(n, m) * sin(m * (Orbit.LLR.Lng)) - h(n, m) * cos(m * (Orbit.LLR.Lng))) * P(n, m);
			tempZ(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * P(n, m);
		}
	
		//for (int i = 0; i < Order + 1; i++) 我感觉应该写成n+1
		for (int i = 0; i < n + 1; i++)
		{
			sumtempX += tempX(i);
			sumtempY += tempY(i);
			sumtempZ += tempZ(i);
		}

		X_prime += POW((EARTH_RADIUS_M / Orbit.LLR.Rds), (n + 2)) * sumtempX;
		Y_prime += POW((EARTH_RADIUS_M / Orbit.LLR.Rds), (n + 2)) * sumtempY;
		Z_prime += (n + 1) * POW((EARTH_RADIUS_M / Orbit.LLR.Rds), (n + 2)) * sumtempZ;
	}
	
	X_prime = - X_prime;
	Y_prime = 1 / cos(Orbit.LLR.Lat) * Y_prime;
	Z_prime = - Z_prime;

	double BN = X_prime * cos(Orbit.LLR.Lat - Orbit.LLA.Lat) - Z_prime * sin(Orbit.LLR.Lat - Orbit.LLA.Lat);
	double BE = Y_prime;
	double BD = X_prime * sin(Orbit.LLR.Lat - Orbit.LLA.Lat) + Z_prime * cos(Orbit.LLR.Lat - Orbit.LLA.Lat);

	NEDMag << BN, BE, BD;
	NEDMag = NT2T(NEDMag);
}

void Environment::StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp)
{
	GetNEDMag(Orbit, timestamp);
	Eigen::Matrix3d Ane = Orbit.NED2ECEF();
	Eigen::Vector3d ECEFMag = Ane * NEDMag;
	//地固系到惯性系
	Eigen::Matrix3d  Aif = Environment::ECI2ECEF(timestamp);
	Eigen::Vector3d ECIMag = Aif.inverse() * ECEFMag;\
	//惯性系到本体系
	BodyMag = Attitude.Qib.ToDcm() * ECIMag;
	SunPos(timestamp);
	SunVecBody = Attitude.Qib.ToDcm() * SunVecInl;
}

void Environment::record(CInfluxDB& DB) {
	DB.addKeyValue("SIM002", 5.5);
}