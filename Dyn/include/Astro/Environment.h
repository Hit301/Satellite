#pragma once
#include"SatelliteMath/BaseMath.h"

class COrbit;

class Environment
{
public:
	Eigen::Vector3d BodyMag;//本体系地磁场强度
	Eigen::Vector3d NEDMag;//北东地系地磁场强度
	Environment();

	//@brief: 计算惯性系转地固系的转移矩阵
	//@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
	//@return : none
	static Eigen::Matrix3d ECI2ECEF(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);


	//@brief: 北东地系地磁场
	//@para : 轨道类轨道根数-半长轴 轨道类LLR
	//@return : none
	void GetNEDMag(const COrbit& Orbit);

	void StateRenew(COrbit& Orbit, const int64_t timestamp);
};
};

class CSunCal
{
public:
	int64_t SunTime;//utc时间戳单位ms
	CDcm dcm;
public:
	CSunCal();

public:
	//@brief: 惯性系下太阳矢量的计算（用时间戳转儒略世纪数进行接下来的计算）
	//@para : 时间戳
	//@return : 惯性系太阳矢量
static Eigen::Vector3d SunPos(int64_t SunTime);

};