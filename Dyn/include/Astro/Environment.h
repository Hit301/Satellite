#pragma once
#include"SatelliteMath/BaseMath.h"
// 2023-12-22 14:44:18
# include "General/AllHead.h"

class COrbit;
class CAttitude;
class Environment
{
public:
	Eigen::Vector3d BodyMag;//本体系地磁场强度T
	Eigen::Vector3d NEDMag;//北东地系地磁场强度T
	Eigen::Vector3d SunVecInl;//惯性系太阳矢量
	Eigen::Vector3d SunVecBody;//本体系太阳矢量

	Environment();

	//@brief: 计算惯性系转地固系的转移矩阵
	//@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
	//@return : none
	static Eigen::Matrix3d ECI2ECEF(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);

	//@brief: 惯性系下太阳矢量的计算（用时间戳转儒略世纪数进行接下来的计算）
	//@para :  timestamp: utc时间戳(ms)
	//@return : 惯性系太阳矢量
	void SunPos(const int64_t timestamp);

	//@brief: 北东地系地磁场
	//@para : 轨道类轨道根数-半长轴 轨道类LLR
	//@return : none
	void GetNEDMag(const COrbit& Orbit, const int64_t timestamp);

	void StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp);

	// 写入数据库
	void record(CInfluxDB& DB);
};