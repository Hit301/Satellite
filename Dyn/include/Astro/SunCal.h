#pragma once
#include"SatelliteMath/BaseMath.h" 
class CSunCal
{

public:
	int64_t SunTime;//utcʱ�����λms

public:
	CSunCal();

public:


	//@brief: ����ϵ��̫��ʸ���ļ���
    //@para : ʱ���
    //@return : ����ϵ��̫��ʸ��
	Eigen::Vector3d SunPos(int64_t SunTime);

};

