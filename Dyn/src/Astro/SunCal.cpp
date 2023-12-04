#include "Astro/SunCal.h"


CSunCal::CSunCal()
{
	//示例时间
	//CurrentTime << 2023,12, 4;
}
//SunTime = int64_t GetTimeStampMs(); //单位ms
Eigen::Vector3d SunPos(int SunTime)
{
	double T; double M; double lamM; double rs; double bs;
	Eigen::Vector3d sunpos;
	//儒略世纪数
	T = TS2CEN(SunTime);
	//太阳平近点角，这里忽略了儒略世纪数的平方及以上项
	M = 357.5256 + 35999.049 * T;
	//太阳相对于当日平春分点的真近点角
	lamM = 282.94 + M + 1.9144 * SIND(DEG2RAD * M) + 0.02 * SIND(2.0 * DEG2RAD*M);
	//太阳地心距
	rs = (149.619 - 2.499 * COSD(DEG2RAD * M) - 0.021 * COSD(2.0 * DEG2RAD * M)) * 1e9; //单位为m
	//黄赤交角
	bs = 23.4393 - 46.815 / 3600 * T - 0.00059 / 3600 * T *T;
	sunpos <<rs * COSD(DEG2RAD*lamM), rs* SIND(DEG2RAD*lamM), 0;
	return sunpos;
}
