#pragma once
#include"SatelliteMath/BaseMath.h"
#include"SatelliteMath/Quaternions.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"

//前向声明
class GyroScope;

class CAttitudeControl
{
public:
	int workmode;
	COrbit Orbit;//轨道
	Environment Env;//环境
public:
	CAttitudeControl();
public:
	//初始速率阻尼控制率设计

 //@brief: 速率阻尼控制器
 //@para : _Gyro:陀螺仪数据 Kp:控制器参数
 //@return : 控制力矩
static Eigen::Vector3d RateDamping(const GyroScope& _Gyro, double Kp);

//对日捕获与定向控制率设计

//@brief: 对日姿态控制器
//@para : _Qib:惯性系下的四元数 _SunPos:惯性系下的太阳矢量 _Gyro:陀螺仪数据 Kp:控制器参数 
//@return : 控制力矩

static Eigen::Vector3d SunControl(const GyroScope& _Gyro, double Kp, Quat _Qib, Eigen::Vector3d _SunPos);


//对地捕获与定向控制率设计

//@brief: 对地姿态控制器
//@para : 
//@return : 控制力矩

//Eigen::Vector3d ToEarthControl();



};

