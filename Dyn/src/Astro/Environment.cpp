#include "Astro/Environment.h"
#include"MySofaDll.h"
#include"Astro/Orbit.h"
#include"Astro/Attitude.h"
#include<fstream>
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

void Environment::GetNEDMag(const COrbit& Orbit)
{
	//@brief: 北东地系地磁场
	//@para : 轨道类轨道根数-半长轴 轨道类LLR
	//@return : none

	//初始化高斯系数
	Eigen::ArrayXXf g(13, 13);
	Eigen::ArrayXXf h(13, 13);
	Eigen::ArrayXXf gdot(13, 13);
	Eigen::ArrayXXf hdot(13, 13);

	g = Eigen::ArrayXXf::Zero(13, 13);
	h = Eigen::ArrayXXf::Zero(13, 13);
	gdot = Eigen::ArrayXXf::Zero(13, 13);
	hdot = Eigen::ArrayXXf::Zero(13, 13);

	//double g[13][13] = { };
	//double h[13][13] = { };
	//double gdot[13][13] = { };
	//double hdot[13][13] = { };

	Eigen::ArrayXXf wmm_2020_data(90, 6);
	wmm_2020_data = Eigen::ArrayXXf::Zero(90, 6);

	//const int rows = 90;
	//const int cols = 6;
	//double wmm_2020_data[rows][cols] = { };

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
		//输出
		//for (int i = 0; i < 90; i++) 
		//{
		//	for (int j = 0; j < 6; j++) 
		//	{
		//		std::cout << wmm_2020_data(i, j) << " ";
		//	}
		//	std::cout << std::endl;
		//}
		file.close();
	}
	else 
	{
		std::cerr << "Unable to open file" << std::endl;
	}

	for (int i = 0; i < 90; i++) 
	{
		int row = wmm_2020_data(i, 0);
		int col = wmm_2020_data(i, 1) ;
		g(row, col) = wmm_2020_data(i, 2);
		h(row, col) = wmm_2020_data(i, 3);
		gdot(row, col) = wmm_2020_data(i, 4);
		hdot(row, col) = wmm_2020_data(i, 5);
	}
	/*std::cout << hdot << std::endl;*/
	//时间戳
	int year = 2020;
	int epoch = 2020;
	int dt_change = year - epoch;

	g = g + gdot * dt_change;
	h = h + hdot * dt_change;

	Eigen::ArrayXXf P(14, 14);
	P = Eigen::MatrixXf::Zero(14, 14);
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

	//对P中的每一个元素都进行施密特半归一化 无法计算高阶阶乘(估计22-28)
	for (int n = 0; n < 14; n++)
	{
		for (int m = 0; m < n + 1; m++)
		{
			if (m > 0)
			{
				P(n, m) = POW(-1, m) * SQRT(2.0 * Factorial(n - m) / Factorial(n + m)) * P(n, m);
			}
		}
	}

	//计算dP
	Eigen::ArrayXXf dP(13, 13);
	dP = Eigen::MatrixXf::Zero(13, 13);
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

	for (int n = 1; n < 13; n++)
	{
		Eigen::ArrayXXf tempX(1, 13);
		Eigen::ArrayXXf tempY(1, 13);
		Eigen::ArrayXXf tempZ(1, 13);

		tempX = Eigen::ArrayXXf::Zero(1, 13);
		tempY = Eigen::ArrayXXf::Zero(1, 13);
		tempZ = Eigen::ArrayXXf::Zero(1, 13);

		for (int m = 0; m < n + 1; m++)
		{
			tempX(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * dP(n, m);
			tempY(m) = m * (g(n, m) * sin(m * (Orbit.LLR.Lng)) - h(n, m) * cos(m * (Orbit.LLR.Lng))) * P(n, m);
			tempZ(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * P(n, m);
		}
		
		double sumtempX = 0;
		double sumtempY = 0;
		double sumtempZ = 0;

		for (int i = 0; i < 13; i++) 
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
	BN = X_prime * cos(Orbit.LLR.Lat - 1.3963) - Z_prime * sin(Orbit.LLR.Lat - 1.3963);
	BE = Y_prime;
	BG = X_prime * sin(Orbit.LLR.Lat - 1.3963) + Z_prime * cos(Orbit.LLR.Lat - 1.3963);

	NEDMag << BN, BE, BG;
}

void Environment::StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp)
{
	GetNEDMag(Orbit);
	Eigen::Matrix3d Ane = Orbit.NED2ECEF();
	BodyMag = Ane * NEDMag;
	SunPos(timestamp);
	SunVecBody = Attitude.Qib.ToDcm() * SunVecInl;
}