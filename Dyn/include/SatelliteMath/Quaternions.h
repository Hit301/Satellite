#pragma once
#include "SatelliteMath/SatelliteEigen.h"

class Quat
{
private:
	double QuatData[4];//JPL四元数，实部在前
	void QuatRectify();//标量修正，保持为正
	void SelfNormalize();//自我归一化
public:
	Quat();//默认构造函数，返回单位四元数
	Quat(double q0, double q1, double q2, double q3);//以值的方式设置四元数
	Quat(const Quat& _Quat);//以其他四元数进行初始化
	Quat(double Theta, const Eigen::Vector3d& Axis);//用转角(rad)和转轴创建四元数
	Quat(const Eigen::Matrix3d& _Dcm);//以方向余弦矩阵初始化
	//此处可以补充其他形式
	~Quat()=default;

	Quat operator+(const Quat& _Quat) const;//四元数加法
	Quat operator-(const Quat& _Quat) const;//四元数减法
	Quat operator*(const Quat& _Quat) const;//四元数乘法
	Quat operator*(const double val) const;//四元数乘标量
	void operator=(const Quat& _Quat);

	void SetByIdx(const int idx, const double val);//按索引设置四元数
	double GetByIdx(const int idx) const;//按索引获取四元数
	Quat QuatNormalize() const;//四元数标准化
	Quat QuatInv() const;//四元数求逆
	Eigen::Matrix3d ToDcm() const;//四元数转方向余弦矩阵
	//此处可以补充其他转换
};

std::ostream& operator<<(std::ostream& _cout, const Quat& _Quat);