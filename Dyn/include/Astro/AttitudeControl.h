#pragma once
#include"SatelliteMath/BaseMath.h"

//前向声明
class GyroScope;

class CAttitudeControl
{
public:
	int workmode;
public:
	CAttitudeControl();
public:
	//初始速率阻尼控制率设计

 //@brief: 速率阻尼控制器
 //@para : _Gyro:陀螺仪数据 Kp:控制器参数
 //@return : 控制力矩
static Eigen::Vector3d RateDamping(const GyroScope& _Gyro, double Kp);


};

