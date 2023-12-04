#include "Astro/Environment.h"
#include"MySofaDll.h"
Environment::Environment()
{
	EarthMag << 0, 0, 0;
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
CSunCal::CSunCal()
{
	//示例时间
	//SunTime =1659312000;
	SunTime = GetTimeStampMs();//单位ms
}

Eigen::Vector3d CSunCal::SunPos(int64_t SunTime)
{
	double TJD; double M; double lamM; double rs; double bs;
	Eigen::Matrix3d Rx;
	Eigen::Vector3d sunpos;

	//儒略世纪数
	TJD = TS2CEN(SunTime);
	//TJD = 0.2258; //为了与MATLAB结果对比时使用
	//太阳平近点角，这里忽略了儒略世纪数的平方及以上项
	M = 357.5256 + 35999.049 * TJD;
	//太阳相对于当日平春分点的真近点角（黄道经度）
	lamM = 282.94 + M + 1.9144 * SIND(DEG2RAD * M) + 0.02 * SIND(2.0 * DEG2RAD * M);
	//太阳地心距
	rs = (149.619 - 2.499 * COSD(DEG2RAD * M) - 0.021 * COSD(2.0 * DEG2RAD * M)) * 1e9;  //单位为m
	//23.4393是黄赤交角
	bs = 23.4393 - 46.815 / 3600 * TJD - 0.00059 / 3600 * TJD * TJD;
	sunpos << rs * COSD(DEG2RAD * lamM), rs* SIND(DEG2RAD * lamM), 0;  //单位为m

	//两种计算（第二个精度更高一些）	
	//Rx << 1, 0, 0,												
	//	0, cos(DEG2RAD*23.4393), -sin(DEG2RAD*23.4393),
	//	0, sin(DEG2RAD*23.4393), cos(DEG2RAD*23.4393);
	Rx << 1, 0, 0,
		0, cos(-DEG2RAD*bs), sin(-DEG2RAD*bs),
		0, -sin(-DEG2RAD*bs), cos(-DEG2RAD*bs);
	Eigen::Vector3d SunPos = Rx * sunpos / rs;  //归一化的结果
	return SunPos;

}

