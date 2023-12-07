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
	//@brief: 以单位阵初始化DcmData
	//@para : none
	//@return : none
	CDcm();

	//@brief: 以给定值初始化DcmData
	//@para : none
	//@return : none
	CDcm(double A00, double A01, double A02,
		 double A10, double A11, double A12,
		 double A20, double A21, double A22);
	
	//@brief: 以单轴旋转初始化DcmData
	//@para : Axis(Dcm_X_AXIS,Dcm_Y_AXIS,Dcm_Z_AXIS)：转轴(查BaseMath)
	//		  Theta(rad)：转角
	//@return : none
	CDcm(unsigned Axis, double Theta);

	//@brief: 以其他方向余弦矩阵初始化DcmData
	//@para : _Dcm：另一个方向余弦矩阵
	//@return : none
	CDcm(CDcm& _Dcm);

	//@brief: 重载赋值运算符
	//@para : _Dcm：另一个方向余弦矩阵
	//@return : none
	CDcm& operator=(CDcm _Dcm);

public:
	//@brief:  方向余弦矩阵转欧拉角
	//@para : Sequence:转序(查BaseMath)
	//@return : 指定转序的欧拉角
	CEulerAgl ToEulerAgl(unsigned Sequence);

	//@brief:  方向余弦矩阵转四元数
	//@para : none
	//@return : 转换得到的四元数
	Quat ToQuat();

public:
	//运算符重载
	Eigen::Vector3d operator*(const Eigen::Vector3d& _Vector) const;//方向余弦矩阵和三维矢量的乘法
	Eigen::Matrix3d operator*(const CDcm& _CDcm) const;//方向余弦矩阵和方向余弦矩阵的乘法

};