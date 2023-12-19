#pragma once
#include "Componet/Gyro.h"
#include"Componet/Flywheel.h"
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
	GyroScope* pGyro;
	flywheel* pWheel;
	
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

//��Componet��Ĺ��캯��������ɸ����������Ͳ����ĳ�ʼ��
//��Componet���Init��������ɸ������������ĳ�ʼ��
//��Componet���StateRenew��������ɸ����������ݸ���
