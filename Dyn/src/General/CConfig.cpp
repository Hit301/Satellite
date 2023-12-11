#include"General/CConfig.h"

CConfig::DeleteHelper CConfig::helper;


CConfig::CConfig()
{
	SatelliteTime = 1640966400000;

	Rx = 6678136.9999998566;
	Ry = 0.0002095685;
	Rz = -1.3800009224;

	Vx = 0.0007615644;
	Vy = 6789.5304738682;
	Vz = 3686.4138485846;

	a = 6678137;
	e = 0;
	i = RAD(28.5);
	RAAN = 0;
	omega = 0;
	M = 0;

	Wx = 0.05;
	Wy = -0.04;
	Wz = 0.01;

	Q0 = 1;
	Q1 = Q2 = Q3 = 0;

	Jxx = 30;
	Jyy = 40;
	Jzz = 50;
	Jxy = Jxz = Jyz = 0;
}
CConfig* CConfig::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new CConfig;
	return m_instance;
}

void CConfig::ReleaseInstance()
{
	CConfig* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}
