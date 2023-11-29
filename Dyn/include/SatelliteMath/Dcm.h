#pragma once
#include"SatelliteMath/BaseMath.h"


//前向声明
class CEulerAgl;
class Quat;

class CDcm
{
public:
	//用Eigen的话，自带矩阵的计算功能
	Eigen::Matrix3d DcmData;

public:
	CDcm();//以单位阵初始化DcmData
	CDcm(double A00, double A01, double A02,//以给定值初始化DcmData
		 double A10, double A11, double A12,
		 double A20, double A21, double A22);
	CDcm(unsigned Axis, double Theta);//以单轴旋转初始化DcmData
	CDcm(CDcm& _Dcm);//以其他方向余弦矩阵初始化DcmData

	////@brief: 以欧拉角初始化DcmData
	////		  Agl(rad):欧拉角结构体
	////@return : none
	//CDcm(CEulerAgl Agl);
	//
	////@brief: 以四元数初始化DcmData
	////@para : quat:四元数
	////@return : none
	//CDcm(Quat quat);

	CDcm& operator=(CDcm _Dcm);//重载赋值运算符

public:
	CEulerAgl ToEulerAgl(unsigned Sequence);//方向余弦矩阵转欧拉角
	Quat ToQuat();//方向余弦矩阵转四元数
};