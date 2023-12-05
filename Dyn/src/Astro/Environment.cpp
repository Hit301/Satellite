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

void Environment::NEDMag()
{
	//@brief: 北东地系地磁场
	//@para : none
	//@return : none

}
