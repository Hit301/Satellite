#pragma once
#include"SatelliteMath/BaseMath.h"

//ǰ������
class GyroScope;

class CAttitudeControl
{
public:
	int workmode;
public:
	CAttitudeControl();
public:
	//��ʼ����������������

 //@brief: �������������
 //@para : _Gyro:���������� Kp:����������
 //@return : ��������
static Eigen::Vector3d RateDamping(const GyroScope& _Gyro, double Kp);


};

