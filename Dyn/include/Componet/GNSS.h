#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"
#include "Astro/Orbit.h"

class GNSS
{
public:
	RV Data;//GNSS数据
	int64_t LastRenewTime;//上次更新时间,utc时间戳ms
	double SamplePeriod;//GNSS采样周期，单位为s
public:
	GNSS();
public:
	void StateRenew(int64_t NowTime, RV& InlRV);
	void Init(RV& InlRV, int64_t timestamp);
};

