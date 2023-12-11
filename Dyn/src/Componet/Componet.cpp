#include<windows.h>
#include "Componet/Componet.h"
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"


CComponet::DeleteHelper CComponet::helper;

CComponet* CComponet::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new CComponet;
	return m_instance;
}

void CComponet::Init(CAttitude& Att, COrbit& Obt, Environment& Env, int64_t timestamp)
{
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		pGyro[i].LastRenewTime = timestamp;
		pGyro[i].Data = RAD2DEG * pGyro[i].InstallMatrix * Att.Omega_b;
	}
}

void CComponet::StateRenew(CAttitude& Att, COrbit& Obt, Environment& Env, int64_t timestamp)
{
	//�������ݸ���
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		pGyro[i].StateRenew(timestamp, Att.Omega_b);
	}
}

CComponet::CComponet() :
	GyroNums(1)
{
	//����������ļ�Ӧ�ã�����Ĭ�����ö�����������
	GyroNums = 1;

	//֮����ݵ����Ĳ����������ã��ɶ�һ��ini
	if (GyroNums <= 0)
	{
		std::cout << "���������Ƿ� ֵ= " << GyroNums << "��ΪĬ��ֵ1" << std::endl;
		MessageBox(NULL, "���������Ƿ�,�������", "����", MB_OKCANCEL);
		exit(0);
	}

	pGyro = new GyroScope[GyroNums];
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		//����Ҫ�ĳ����ñ����͵�
		pGyro[i].InstallMatrix << Eigen::Matrix3d::Identity();
		pGyro[i].SamplePeriod = 0.25;
	}
}

CComponet::~CComponet()
{
	delete[] pGyro;
}

void CComponet::ReleaseInstance()
{
	CComponet* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}
