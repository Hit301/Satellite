#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Satellite/Gyro.h"
#include "Astro/Environment.h"
class Satellite
{
public:
	int64_t SatelliteTime;//������utcʱ�����λms
	COrbit Orbit;//���
	CAttitude Attitude;//��̬
	Environment Env;//����
	GyroScope _Gyro;//����
	CSunCal SunCal;//̫��ʸ�������ԣ�
	Eigen::Vector3d SolVec;//̫��ʸ�������ԣ�

public:
	Satellite();

	~Satellite()=default;
	void StateRenew(double SampleTime);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);