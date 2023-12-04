#include "Astro/SunCal.h"


CSunCal::CSunCal()
{
	//ʾ��ʱ��
	//CurrentTime << 2023,12, 4;
}
//SunTime = int64_t GetTimeStampMs(); //��λms
Eigen::Vector3d SunPos(int SunTime)
{
	double T; double M; double lamM; double rs; double bs;
	Eigen::Vector3d sunpos;
	//����������
	T = TS2CEN(SunTime);
	//̫��ƽ����ǣ����������������������ƽ����������
	M = 357.5256 + 35999.049 * T;
	//̫������ڵ���ƽ���ֵ��������
	lamM = 282.94 + M + 1.9144 * SIND(DEG2RAD * M) + 0.02 * SIND(2.0 * DEG2RAD*M);
	//̫�����ľ�
	rs = (149.619 - 2.499 * COSD(DEG2RAD * M) - 0.021 * COSD(2.0 * DEG2RAD * M)) * 1e9; //��λΪm
	//�Ƴཻ��
	bs = 23.4393 - 46.815 / 3600 * T - 0.00059 / 3600 * T *T;
	sunpos <<rs * COSD(DEG2RAD*lamM), rs* SIND(DEG2RAD*lamM), 0;
	return sunpos;
}
