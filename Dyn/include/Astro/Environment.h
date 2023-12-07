#pragma once
#include"SatelliteMath/BaseMath.h"

class COrbit;

class Environment
{
public:
	Eigen::Vector3d EarthMag;//����ϵ�شų�ǿ��
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
};