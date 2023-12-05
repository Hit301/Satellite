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
	//ʾ��ʱ��
	//SunTime =1659312000;
	SunTime = GetTimeStampMs();//��λms
}

Eigen::Vector3d CSunCal::SunPos(int64_t SunTime)
{
	double TJD; double M;long double lamM;long double rs; double bs;
	Eigen::Matrix3d Rx;
	Eigen::Vector3d sunpos;

	//����������
	TJD = TS2CEN(SunTime);
	//TJD = 0.2258; //Ϊ����MATLAB����Ա�ʱʹ��
	//̫��ƽ����ǣ����������������������ƽ����������
	M = 357.5256 + 35999.049 * TJD;
	//̫������ڵ���ƽ���ֵ�������ǣ��Ƶ����ȣ�
	lamM = 282.94 + M + SEC2DEG*6892 * SIND(M) + SEC2DEG*72 * SIND(M);
	//̫�����ľ�
	rs = (149.619 - 2.499 * COSD(M) - 0.021 * COSD(2.0 *M)) * 1e9;  //��λΪm
	//23.4393�ǻƳཻ��
	bs = 23.4393 - 46.815 / 3600 * TJD - 0.00059 / 3600 * TJD * TJD;
	sunpos << rs * COSD(lamM), rs* SIND(lamM), 0;  //��λΪm

	//���ּ��㣨�ڶ������ȸ���һЩ��	
	//Rx << 1, 0, 0,												
	//	0, COSD(-23.4393), SIND(-23.4393),
	//	0, -SIND(-23.4393), COSD(-23.4393);
	Rx << 1, 0, 0,
		0, COSD(-bs), SIND(-bs),
		0, -SIND(-bs), COSD(-bs);
	Eigen::Vector3d SunPos = Rx * sunpos / rs;  //��һ���Ľ��
	return SunPos;
}

