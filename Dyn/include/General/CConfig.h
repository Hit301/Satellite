#pragma once
#include"SatelliteMath/BaseMath.h"
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"
#include"Componet/Componet.h"
#include "AStro/AttitudeControl.h"


class CConfig
{
public:
	static CConfig* GetInstance();

public:
	int64_t SatelliteTime;//星历，utc时间戳单位ms

	//这里不用Eigen是为了客户端好配置

	//初始轨道，先暂定是惯性系的
	double Rx, Ry, Rz;
	double Vx, Vy, Vz;

	//六根数
	double a;               //轨道半长轴(Semi-major Axis)
	double e;               //轨道偏心率(Eccentricity)
	double i;               //轨道倾角（rad） (Inclination)
	double RAAN;            //升交点赤经（rad）(RAAN)
	double omega;           //近地点幅角（rad）(Arg of Perigee)
	double M;               //轨道平近点角（rad）(Mean Anomaly)

	//本体系角速度
	double Wx, Wy, Wz;

	//四元数，暂定Qib
	double Q0, Q1, Q2, Q3;

	//惯量矩阵
	double Jxx, Jxy, Jxz, Jyy, Jyz, Jzz;

	//单机配置， 因为单机数量不定所以应该走配置表

	//环境配置
	//地磁阶次，1~12
	size_t MagOrder;
	//地磁场高斯系数，一次性读进来方便后续使用
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_g;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_h;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_gdot;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_hdot;
private:
	static inline CConfig* m_instance{ NULL };

	CConfig();
	~CConfig()=default;
	CConfig(const CConfig& _Config) = delete;
	CConfig& operator=(const CConfig& _Config) = delete;

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

