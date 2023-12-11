#pragma once
#include "Componet/Gyro.h"

class CComponet
{
public:
	static CComponet* GetInstance();

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
