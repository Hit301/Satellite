#pragma once
#include"SatelliteMath/BaseMath.h" 
class CSunCal
{

public:
	int64_t SunTime;//utc时间戳单位ms

public:
	CSunCal();

public:


	//@brief: 惯性系下太阳矢量的计算
    //@para : 时间戳
    //@return : 惯性系下太阳矢量
	Eigen::Vector3d SunPos(int64_t SunTime);

};

