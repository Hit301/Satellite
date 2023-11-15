#pragma once
#include "SatelliteMath/SatelliteEigen.h"

class Quat
{
private:
	double QuatData[4];//JPL��Ԫ����ʵ����ǰ
	void QuatRectify();//��������������Ϊ��
	void SelfNormalize();//���ҹ�һ��
public:
	Quat();//Ĭ�Ϲ��캯�������ص�λ��Ԫ��
	Quat(double q0, double q1, double q2, double q3);//��ֵ�ķ�ʽ������Ԫ��
	Quat(const Quat& _Quat);//��������Ԫ�����г�ʼ��
	Quat(double Theta, const Eigen::Vector3d& Axis);//��ת��(rad)��ת�ᴴ����Ԫ��
	Quat(const Eigen::Matrix3d& _Dcm);//�Է������Ҿ����ʼ��
	//�˴����Բ���������ʽ
	~Quat()=default;

	Quat operator+(const Quat& _Quat) const;//��Ԫ���ӷ�
	Quat operator-(const Quat& _Quat) const;//��Ԫ������
	Quat operator*(const Quat& _Quat) const;//��Ԫ���˷�
	Quat operator*(const double val) const;//��Ԫ���˱���
	void operator=(const Quat& _Quat);

	void SetByIdx(const int idx, const double val);//������������Ԫ��
	double GetByIdx(const int idx) const;//��������ȡ��Ԫ��
	Quat QuatNormalize() const;//��Ԫ����׼��
	Quat QuatInv() const;//��Ԫ������
	Eigen::Matrix3d ToDcm() const;//��Ԫ��ת�������Ҿ���
	//�˴����Բ�������ת��
};

std::ostream& operator<<(std::ostream& _cout, const Quat& _Quat);