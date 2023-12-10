#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Satellite/Gyro.h"
#include "Astro/Environment.h"
#include "AStro/AttitudeControl.h"
class Satellite
{
public:
	int64_t SatelliteTime;//������utcʱ�����λms
	COrbit Orbit;//���
	CAttitude Attitude;//��̬
	Environment Env;//����
	GyroScope _Gyro;//����
	CAttitudeControl AttitudeControl;//��̬����
	CSunCal SunCal;//̫��ʸ�������ԣ�
	Eigen::Vector3d SolVec;//̫��ʸ�������ԣ�
	double Angle; // ̫��ʸ�������� - z��нǣ���λ:deg��
	Quat _Qbo;//���ϵ�µ���̬��Ԫ��

	// ����ģʽ��־����
	bool sunControlMode;
	bool earthControlMode;

public:
	Satellite();

	~Satellite()=default;
	void StateRenew(double SampleTime);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);