#pragma once
#include"SatelliteMath/BaseMath.h"

class COrbit;

class Environment
{
public:
	Eigen::Vector3d EarthMag;//本体系地磁场强度
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
};