#pragma once
#include "Componet/Gyro.h"
#include"General/AllHead.h"
// 2023-12-22 14:47:10

class CComponet
{
public:
	static CComponet* GetInstance();
	void Init(CAttitude& Att, COrbit& Obt, Environment& Env, int64_t timestamp);
	void StateRenew(CAttitude& Att, COrbit& Obt, Environment& Env, int64_t timestamp);
	// д�����ݿ�
	void record(CInfluxDB& DB);
public:
	size_t GyroNums;
	GyroScope* pGyro;
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


