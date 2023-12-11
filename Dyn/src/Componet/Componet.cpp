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
	//陀螺数据更新
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		pGyro[i].StateRenew(timestamp, Att.Omega_b);
	}
}

CComponet::CComponet() :
	GyroNums(1)
{
	//这里读配置文件应该，先走默认配置读各单机数量
	GyroNums = 1;

	//之后根据单机的参数进行配置，可读一个ini
	if (GyroNums <= 0)
	{
		std::cout << "陀螺数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "陀螺数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}

	pGyro = new GyroScope[GyroNums];
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		//这里要改成配置表类型的
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
