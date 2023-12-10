#pragma once
#include"SatelliteMath/BaseMath.h"
#include"SatelliteMath/Quaternions.h"
#include"Astro/Orbit.h"
#include"SatelliteMath/Dcm.h"

//前向声明
class GyroScope;

class CAttitudeControl
{
public:
	int workmode;//姿态控制模式

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
//@para : _Gyro:陀螺仪数据 Kp:控制器参数 Kd:控制器参数 _Qib:惯性系下的四元数 _SunPos:惯性系下的太阳矢量
//@return : 控制力矩

static Eigen::Vector3d ToSunControl(const GyroScope& _Gyro, double Kp, double Kd, Quat _Qib, Eigen::Vector3d _SunPos);

//@brief: 太阳矢量与卫星-z轴夹角
static double GetAngle(Quat _Qib, Eigen::Vector3d _SunPos);

//对地捕获与定向控制率设计

//@brief: 对地姿态控制器
//@para : _Gyro:陀螺仪数据 Kp:控制器参数 Kd:控制器参数 _Qib:惯性系下的四元数
//@return : 控制力矩

static Eigen::Vector3d ToEarthControl(const GyroScope& _Gyro, double Kp, double Kd, Quat _Qib);

//@brief: 轨道系下姿态四元数
static Quat GetQbo(Quat _Qib);

};

