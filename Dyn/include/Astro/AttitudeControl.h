#pragma once
#include"SatelliteMath/Quaternions.h"
#include"Astro/Attitude.h"
#include "Astro/Environment.h"

class AttitudeControl
{
public:
	int workmode = 1;

	//时间戳转换为年月日
    

	//计算太阳在J2000坐标系下的位置

	
	//力矩、角速度、四元数初值，主轴惯量


	//调用轨道动力学、姿态动力学、姿态运动学


	//根据卫星在惯性系中的四元数和惯性系中太阳矢量计算计算本体系下的太阳矢量


	//控制指令计算


	//
	



	//模式转换
	int changeworkmode()





    //计算轨道坐标系下的四元数


};
