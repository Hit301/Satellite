#include<windows.h>
#include "Componet/Componet.h"
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"


CComponet::DeleteHelper CComponet::helper;

CComponet* pComponet = CComponet::GetInstance();


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
		pGyro[i].Init(Att.Omega_b, timestamp);
	}
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		pWheel[i].Init(0, timestamp);
	}
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		pSun[i].Init(Env.BodyMag, timestamp);
	}
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		pStar[i].Init(Att.Qib, timestamp);
	}
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		pMag[i].Init(Env.BodyMag, timestamp);
	}


}

void CComponet::StateRenew(CAttitude& Att, COrbit& Obt, Environment& Env, int64_t timestamp)
{
	//陀螺数据更新
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		pGyro[i].StateRenew(timestamp, Att.Omega_b);
	}
	for (size_t i{ 0 }; i < MagSensorNums; i++)
	{
		pMag[i].StateRenew(timestamp,Env.BodyMag);
	}
	for (size_t i{ 0 }; i < StarSensorNums; i++)
	{
		pStar[i].StateRenew(timestamp,Att.Qib);
	}
	for (size_t i{ 0 }; i < SunSensorNums; i++)
	{
		pSun[i].StateRenew(timestamp, Env.SunVecBody);
	}
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		pWheel[i].StateRenew(timestamp, 0.0);
	}

}

CComponet::CComponet() :
	GyroNums(1)
{
	//这里读配置文件应该，先走默认配置读各单机数量
	GyroNums = 1;
	FlywheelNums = 1;
	MagSensorNums = 1;
	SunSensorNums = 1;
	StarSensorNums = 1;
	//之后根据单机的参数进行配置，可读一个ini
	if (GyroNums <= 0)
	{
		std::cout << "陀螺数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "陀螺数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	if (FlywheelNums<= 0)
	{
		std::cout << "飞轮数量非法 值= " << FlywheelNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "飞轮数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	if (MagSensorNums <= 0)
	{
		std::cout << "磁强计数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "磁强计数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}

	if (StarSensorNums <= 0)
	{
		std::cout << "星敏数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "星敏数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	if (SunSensorNums <= 0)
	{
		std::cout << "太敏数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "太敏数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}





	pGyro = new GyroScope[GyroNums];
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		//这里要改成配置表类型的
		pGyro[i].InstallMatrix << Eigen::Matrix3d::Identity();
		pGyro[i].SamplePeriod = 0.25;
	}
	pWheel = new flywheel[FlywheelNums];
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		//这里要改成配置表类型的
		pWheel[i].InstallVet << Eigen::Vector3d::Identity();
		pWheel[i].SamplePeriod = 0.25;
	}

	pSun = new SunSensor[SunSensorNums];
	for (size_t i{ 0 }; i < SunSensorNums; i++)
	{
		//这里要改成配置表类型的
		pSun[i].InstallMatrix << Eigen::Matrix3d::Identity();
		pSun[i].SamplePeriod = 0.25;
	}
	pStar = new StarSensor[StarSensorNums];
	for (size_t i{ 0 }; i < StarSensorNums; i++)
	{
		//这里要改成配置表类型的
		pStar[i].InstallMatrix;
		pStar[i].SamplePeriod = 0.25;
	}
	
	pMag = new MagSensor[MagSensorNums];
	for (size_t i{ 0 }; i < MagSensorNums; i++)
	{
		//这里要改成配置表类型的
		pMag[i].InstallMatrix<< Eigen::Matrix3d::Identity();
		pMag[i].SamplePeriod = 0.25;
	}




}

CComponet::~CComponet()
{
	delete[] pGyro;
	delete[] pWheel;
	delete[] pMag;
	delete[] pSun;
	delete[] pStar;
}

void CComponet::ReleaseInstance()
{
	CComponet* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}
