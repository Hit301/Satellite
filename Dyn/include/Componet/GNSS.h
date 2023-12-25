#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"
#include "Astro/Orbit.h"

class GNSS
{
public:
	RV Data;//GNSS����
	int64_t LastRenewTime;//�ϴθ���ʱ��,utcʱ���ms
	double SamplePeriod;//GNSS�������ڣ���λΪs
public:
	GNSS();
public:
	void StateRenew(int64_t NowTime, RV& InlRV);
	void Init(RV& InlRV, int64_t timestamp);
};

