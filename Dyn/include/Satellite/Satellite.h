#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"
#include "AStro/AttitudeControl.h"
#include"Componet/Componet.h"


class Satellite
{
public:
	int64_t SatelliteTime;//������utcʱ�����λms
	COrbit Orbit;//���
	CAttitude Attitude;//��̬
	Environment Env;//����
	CComponet* pComponet;//����
	CAttitudeController AttController;//��̬����
public:
	double SampleTime;//����ʱ������λs
	int SpeedTimes;//���ٱ���
public:
	Satellite();
	Satellite(double Ts, int m_SpeedTimes);
	~Satellite()=default;
	void StateRenew();
	// 2023-12-22 11:28:55
	void dataToDB(CInfluxDB& DB, double Period);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);