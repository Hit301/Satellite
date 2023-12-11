#pragma once
#include"SatelliteMath/BaseMath.h"

class COrbit;

class Environment
{
public:
	Eigen::Vector3d BodyMag;//����ϵ�شų�ǿ��
	Eigen::Vector3d NEDMag;//������ϵ�شų�ǿ��
	Environment();

	//@brief: �������ϵת�ع�ϵ��ת�ƾ���
	//@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
	//@return : none
	static Eigen::Matrix3d ECI2ECEF(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);


	//@brief: ������ϵ�شų�
	//@para : �����������-�볤�� �����LLR
	//@return : none
	void GetNEDMag(const COrbit& Orbit);

	void StateRenew(COrbit& Orbit, const int64_t timestamp);
};
};

class CSunCal
{
public:
	int64_t SunTime;//utcʱ�����λms
	CDcm dcm;
public:
	CSunCal();

public:
	//@brief: ����ϵ��̫��ʸ���ļ��㣨��ʱ���ת�������������н������ļ��㣩
	//@para : ʱ���
	//@return : ����ϵ̫��ʸ��
static Eigen::Vector3d SunPos(int64_t SunTime);

};