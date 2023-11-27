#pragma once
#include"SatelliteMath/BaseMath.h"

//前向声明
class CDcm;
class CEulerAgl;

class Quat
{
private:
	void QuatRectify();//标量修正，保持为正
	void SelfNormalize();//自我归一化

public:
	double QuatData[4];//JPL四元数，实部在前

public:
	Quat();//默认构造函数，返回单位四元数
	Quat(double q0, double q1, double q2, double q3);//以值的方式设置四元数
	Quat(double Theta, const Eigen::Vector3d& Axis);//用转角(rad)和转轴创建四元数
	Quat(const Quat& _Quat);//以其他四元数进行初始化
	~Quat()=default;

public:
	//运算符重载
	Quat operator+(const Quat& _Quat) const;//四元数加法
	Quat operator-(const Quat& _Quat) const;//四元数减法
	Quat operator*(const Quat& _Quat) const;//四元数乘法
	Quat operator*(const double val) const;//四元数乘标量
	Quat& operator=(const Quat& _Quat);

public:
	Quat QuatNormalize() const;//四元数标准化
	Quat QuatInv() const;//四元数求逆
	CDcm ToDcm() const;//四元数转方向余弦矩阵
	CEulerAgl ToEulerAgl(unsigned Seq) const;//四元数转指定转序欧拉角
};

std::ostream& operator<<(std::ostream& _cout, const Quat& _Quat);