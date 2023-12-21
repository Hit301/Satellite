#pragma once
#include "Componet/Gyro.h"
#include"Componet/Flywheel.h"
#include"Componet/MagSensor.h"
#include"Componet/StarSensor.h"
#include"Componet/SunSensor.h"
#include"General/AllHead.h"


class CComponet
{
public:
	static CComponet* GetInstance();
	void Init(CAttitude& Att, COrbit& Obt, Environment& Env, int64_t timestamp);
	void StateRenew(CAttitude& Att, COrbit& Obt, Environment& Env, int64_t timestamp);
public:
	size_t GyroNums;
	size_t FlywheelNums;
	size_t MagSensorNums;
	size_t StarSensorNums;
	size_t SunSensorNums;
	GyroScope* pGyro;
	flywheel* pWheel;
	SunSensor* pSun;
	StarSensor* pStar;
	MagSensor* pMag;


	
	
private:
	static inline CComponet* m_instance{ NULL };

	CComponet();
	~CComponet();
	CComponet(const CComponet& _CComponet) = delete;
	CComponet& operator=(const CComponet& _CComponet) = delete;

	static void ReleaseInstance();
	class DeleteHelper
	{
	public:
		DeleteHelper() = default;
		~DeleteHelper()
		{
			ReleaseInstance();
		}
	};
	static DeleteHelper helper;
};

//在Componet类的构造函数中中完成各单机数量和参数的初始化
//在Componet类的Init函数中完成各单机物理量的初始化
//在Componet类的StateRenew函数中完成各单机的数据更新
