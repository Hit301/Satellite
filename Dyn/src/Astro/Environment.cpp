#include "Astro/Environment.h"
#include"MySofaDll.h"
#include"Astro/Orbit.h"
#include"Astro/Attitude.h"
#include"fstream"
#include "ctime"
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

	//初始化高斯系数
	Eigen::ArrayXXd g(13, 13);
	Eigen::ArrayXXd h(13, 13);
	Eigen::ArrayXXd gdot(13, 13);
	Eigen::ArrayXXd hdot(13, 13);

	g = Eigen::ArrayXXd::Zero(13, 13);
	h = Eigen::ArrayXXd::Zero(13, 13);
	gdot = Eigen::ArrayXXd::Zero(13, 13);
	hdot = Eigen::ArrayXXd::Zero(13, 13);

	Eigen::ArrayXXd wmm_2020_data(90, 6);
	wmm_2020_data = Eigen::ArrayXXd::Zero(90, 6);

	std::ifstream file("src/Config/wmm_2020_data.txt");
	if (file.is_open()) 
	{
		for (int i = 0; i < 90; i++) 
		{
			for (int j = 0; j < 6; j++) 
			{
				if (!(file >> wmm_2020_data(i,j)))
				{
					std::cerr << "Unable to successfully read data from file" << std::endl;
				}
			}
		}
		file.close();
	}
	else 
	{
		std::cerr << "Unable to open file" << std::endl;
	}

	for (int i = 0; i < 90; i++) 
	{
		int row = (int)wmm_2020_data(i, 0);
		int col = (int)wmm_2020_data(i, 1) ;
		g(row, col) = wmm_2020_data(i, 2);
		h(row, col) = wmm_2020_data(i, 3);
		gdot(row, col) = wmm_2020_data(i, 4);
		hdot(row, col) = wmm_2020_data(i, 5);
	}
	//时间戳
	time_t utcTime = timestamp;
	tm timeInfo;
	gmtime_s(&timeInfo, &utcTime);

	int year = timeInfo.tm_year + 1900;
	int month = timeInfo.tm_mon + 1;
	int day = timeInfo.tm_mday;

	double epoch = DecYear(2020,1,1);
	double dt_change = DecYear(year, month, day) - epoch;

	g = g + gdot * dt_change;
	h = h + hdot * dt_change;

	Eigen::ArrayXXd P(14, 14);
	P = Eigen::MatrixXd::Zero(14, 14);
	double x = sin(Orbit.LLR.Lat);

	//计算帝合勒让德函数
	//计算对角线上元素
	for (int n = 0; n < 14; n++) 
	{
		P(n, n) = POW(-1, n) * DoubleFactorial(2 * n - 1) * POW(1 - x * x, n / 2.0);
	}
	
	//计算主对角线下一层元素
	for (int n = 0; n < 13; n++)
	{
		P(n + 1, n) = x * (2 * n + 1) * P(n, n);
	}

	//计算非对角线元素
	for (int n = 2; n < 14; n++)
	{
		for (int m = 0; m < n - 1; m++)
		{
			P(n, m) = 1.0 / (n - m) * (x * (2 * n - 1) * P((n - 1), m) - (n + m - 1) * P((n - 2), m));
		}
	}

	//对P中的每一个元素都进行施密特半归一化 
	for (int n = 0; n < 14; n++)
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
	Eigen::ArrayXXd dP(13, 13);
	dP = Eigen::MatrixXd::Zero(13, 13);
	for (int n = 0; n < 13; n++)
	{
		for (int m = 0; m < n + 1; m++)
		{
			dP(n, m) = (n + 1) * tan(Orbit.LLR.Lat) * P(n, m) - SQRT((n + 1) * (n + 1) - m * m) * (1 / cos(Orbit.LLR.Lat)) * P(n + 1, m);
		}
	}
	double X_prime = 0;
	double Y_prime = 0;
	double Z_prime = 0;

	int MagOrder = 12;

	for (int n = 1; n < MagOrder + 1; n++)
	{
		Eigen::ArrayXXd tempX(1, MagOrder + 1);
		Eigen::ArrayXXd tempY(1, MagOrder + 1);
		Eigen::ArrayXXd tempZ(1, MagOrder + 1);

		tempX = Eigen::ArrayXXd::Zero(1, MagOrder + 1);
		tempY = Eigen::ArrayXXd::Zero(1, MagOrder + 1);
		tempZ = Eigen::ArrayXXd::Zero(1, MagOrder + 1);

		for (int m = 0; m < n + 1; m++)
		{
			tempX(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * dP(n, m);
			tempY(m) = m * (g(n, m) * sin(m * (Orbit.LLR.Lng)) - h(n, m) * cos(m * (Orbit.LLR.Lng))) * P(n, m);
			tempZ(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * P(n, m);
		}
		
		double sumtempX = 0;
		double sumtempY = 0;
		double sumtempZ = 0;

		for (int i = 0; i < MagOrder + 1; i++)
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

	double BN = 0;
	double BE = 0;
	double BG = 0;
	BN = X_prime * cos(Orbit.LLR.Lat - Orbit.LLA.Lat) - Z_prime * sin(Orbit.LLR.Lat - Orbit.LLA.Lat);
	BE = Y_prime;
	BG = X_prime * sin(Orbit.LLR.Lat - Orbit.LLA.Lat) + Z_prime * cos(Orbit.LLR.Lat - Orbit.LLA.Lat);

	NEDMag << BN, BE, BG;
}

void Environment::StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp)
{
	GetNEDMag(Orbit, timestamp);
	Eigen::Matrix3d Ane = Orbit.NED2ECEF();
	BodyMag = Ane * NEDMag;
	SunPos(timestamp);
	SunVecBody = Attitude.Qib.ToDcm() * SunVecInl;
}