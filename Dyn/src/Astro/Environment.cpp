#include "Astro/Environment.h"
#include"MySofaDll.h"
#include"Astro/Orbit.h"

Environment::Environment()
{
	BodyMag << 0, 0, 0;
	NEDMag << 0, 0, 0;
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

void Environment::GetNEDMag(const COrbit& Orbit)
{
	//@brief: 北东地系地磁场
	//@para : 轨道类轨道根数-半长轴 轨道类LLR
	//@return : none

	const double theta = HALFPI - Orbit.LLR.Lat;
	const double _re_r = EARTH_RADIUS_M / Orbit.OrbitElements.a;
	const double re_r3 = _re_r * _re_r * _re_r;
	const double re_r4 = re_r3 * _re_r;
	const double re_r5 = re_r4 * _re_r;

	const double sin_theta = sin(theta);

	const double cos_theta = cos(theta);
	const double sin_2theta = sin(2.0 * theta);
	const double cos_2theta = cos(2.0 * theta);
	const double sin_lambdal = sin(Orbit.LLR.Lng);
	const double cos_lambdal = cos(Orbit.LLR.Lng);
	const double sin_2lambdal = sin(2.0 * Orbit.LLR.Lng);
	const double sin_3lambdal = sin(3.0 * Orbit.LLR.Lng);
	const double cos_2lambdal = cos(2.0 * Orbit.LLR.Lng);
	const double cos_3lambdal = cos(3.0 * Orbit.LLR.Lng);
	const double sin2_theta = sin_theta * sin_theta;
	const double sin3_theta = sin2_theta * sin_theta;
	const double cos2_theta = cos_theta * cos_theta;
	const double cos3_theta = cos2_theta * cos_theta;

	//BN,BE,BG为三轴磁强，单位nT 

//wmm2020_data
/*高斯系数理论上会随着时间变化而变化，但是变动较为微小，
且这样的微小变化对于磁强最终大小的影响是较小的，不会改变计算的磁强与官方公布磁强的误差的数量级*/
#define G10  (-29404.5)
#define G11  (-1450.7)
#define H11  (4652.9)
#define G20  (-2500.0)
#define G21  (2982.0 )
#define H21  (-2991.6)
#define G22  (1676.8)
#define H22  (-734.8)
#define G30  (1363.9)
#define G31  (-2381.0)
#define H31  (-82.2)
#define G32  (1236.2)
#define H32  (241.8)
#define G33  (525.7)
#define H33  (-542.9)

//wmm2010_data
//#define G10  (-29496.6)
//#define G11  (-1586.3)
//#define H11  (4944.4)
//#define G20  (-2396.6)
//#define G21  (3026.1)
//#define H21  (-2707.7)
//#define G22  (1668.6)
//#define H22  (-576.1)
//#define G30  (1340.1)
//#define G31  (-2326.2)
//#define H31  (-160.2)
//#define G32  (1231.9)
//#define H32  (251.9)
//#define G33  (634.0)
//#define H33  (-536.6)

//银河
//#define G10  (-29442.0)
//#define G11  (-1501.00)
//#define H11  (4797.100)
//#define G20  (-2445.10)
//#define G21  (3012.900)
//#define H21  (-2845.60)
//#define G22  (1676.700)
//#define H22  (-641.900)
//#define G30  (1350.700)
//#define G31  (-2352.30)
//#define H31  (-115.300)
//#define G32  (1225.600)
//#define H32  (244.9000)
//#define G33  (582.0000)
//#define H33  (-538.400)

	//分别计算BN,BE,BG
	double re3_f64 = (G10 * (-sin_theta) + (G11 * cos_lambdal + H11 * sin_lambdal) * cos_theta);
	double re4_f64 = (G20 * (-3.0 * sin_theta * cos_theta) +
		(G21 * cos_lambdal + H21 * sin_lambdal) * (1.732050807568877 * cos_2theta) +
		(G22 * cos_2lambdal + H22 * sin_2lambdal) * (0.866025403784438 * sin_2theta));
	double re5_f64 = (G30 * (1.5 * sin_theta * (1.0 - 5.0 * cos2_theta)) +
		(G31 * cos_lambdal + H31 * sin_lambdal) * (0.612372435695794 * cos_theta * (4.0 - 15.0 * sin2_theta)) +
		(G32 * cos_2lambdal + H32 * sin_2lambdal) * (1.936491673103708 * sin_theta * (3.0 * cos2_theta - 1.0)) +
		(G33 * cos_3lambdal + H33 * sin_3lambdal) * (2.371708245126284 * sin2_theta * cos_theta));

	double bn = re_r3 * re3_f64 + re_r4 * re4_f64 + re_r5 * re5_f64;

	re3_f64 = ((G11 * sin_lambdal - H11 * cos_lambdal) * sin_theta);
	re4_f64 = ((G21 * sin_lambdal - H21 * cos_lambdal) * (0.866025403784438 * sin_2theta) +
		2.0 * (G22 * sin_2lambdal - H22 * cos_2lambdal) * (0.866025403784438 * sin2_theta));
	re5_f64 = ((G31 * sin_lambdal - H31 * cos_lambdal) * (0.612372435695794 * sin_theta * (5.0 * cos2_theta - 1.0)) +
		2.0 * (G32 * sin_2lambdal - H32 * cos_2lambdal) * (1.936491673103708 * sin2_theta * cos_theta) +
		3.0 * (G33 * sin_3lambdal - H33 * cos_3lambdal) * (0.790569415042094 * sin3_theta));

	double be = 1.0 / sin_theta * (re_r3 * re3_f64 + re_r4 * re4_f64 + re_r5 * re5_f64);

	re3_f64 = (G10 * cos_theta + (G11 * cos_lambdal + H11 * sin_lambdal) * sin_theta);
	re4_f64 = (G20 * (1.5 * cos2_theta - 0.5) +
		(G21 * cos_lambdal + H21 * sin_lambdal) * (0.866025403784438 * sin_2theta) +
		(G22 * cos_2lambdal + H22 * sin_2lambdal) * (0.866025403784438 * sin2_theta));
	re5_f64 = (G30 * (2.5 * cos3_theta - 1.5 * cos_theta) +
		(G31 * cos_lambdal + H31 * sin_lambdal) * (0.612372435695794 * sin_theta * (5.0 * cos2_theta - 1.0)) +
		(G32 * cos_2lambdal + H32 * sin_2lambdal) * (1.936491673103708 * sin2_theta * cos_theta) +
		(G33 * cos_3lambdal + H33 * sin_3lambdal) * (0.790569415042094 * sin3_theta));

	double bg = -2.0 * re_r3 * re3_f64 - 3.0 * re_r4 * re4_f64 - 4.0 * re_r5 * re5_f64;

	NEDMag << bn, be, bg;
}

void Environment::StateRenew(COrbit& Orbit, const int64_t timestamp)
{
	GetNEDMag(Orbit);
	Eigen::Matrix3d Ane = Orbit.NED2ECEF();
	BodyMag = Ane * NEDMag;
}
CSunCal::CSunCal()
{
	//示例时间
	//SunTime =1659312000;
	SunTime = GetTimeStampMs();//单位ms
}

Eigen::Vector3d CSunCal::SunPos(int64_t SunTime)
{
	double TJD; double M;long double lamM;long double rs; double bs;
	Eigen::Matrix3d Rx;
	Eigen::Vector3d sunpos;

	//儒略世纪数
	TJD = TS2CEN(SunTime);
	//TJD = 0.2258; //为了与MATLAB结果对比时使用
	//太阳平近点角，这里忽略了儒略世纪数的平方及以上项
	M = 357.5256 + 35999.049 * TJD;
	//太阳相对于当日平春分点的真近点角（黄道经度）
	lamM = 282.94 + M + SEC2DEG*6892 * SIND(M) + SEC2DEG*72 * SIND(M);
	//太阳地心距
	rs = (149.619 - 2.499 * COSD(M) - 0.021 * COSD(2.0 *M)) * 1e9;  //单位为m
	//23.4393是黄赤交角
	bs = 23.4393 - 46.815 / 3600 * TJD - 0.00059 / 3600 * TJD * TJD;
	sunpos << rs * COSD(lamM), rs* SIND(lamM), 0;  //单位为m

	//两种计算（第二个精度更高一些）	
	//Rx << 1, 0, 0,												
	//	0, COSD(-23.4393), SIND(-23.4393),
	//	0, -SIND(-23.4393), COSD(-23.4393);
	Rx << 1, 0, 0,
		0, COSD(-bs), SIND(-bs),
		0, -SIND(-bs), COSD(-bs);
	Eigen::Vector3d SunPos = Rx * sunpos / rs;  //归一化的结果
	return SunPos;
}

