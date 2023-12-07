#pragma once
#include"SatelliteMath/BaseMath.h"
#include"SatelliteMath/Quaternions.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"

//ǰ������
class GyroScope;

class CAttitudeControl
{
public:
	int workmode;
	COrbit Orbit;//���
	Environment Env;//����
public:
	CAttitudeControl();
public:
	//��ʼ����������������

 //@brief: �������������
 //@para : _Gyro:���������� Kp:����������
 //@return : ��������
static Eigen::Vector3d RateDamping(const GyroScope& _Gyro, double Kp);

//���ղ����붨����������

//@brief: ������̬������
//@para : _Qib:����ϵ�µ���Ԫ�� _SunPos:����ϵ�µ�̫��ʸ�� _Gyro:���������� Kp:���������� 
//@return : ��������

static Eigen::Vector3d SunControl(const GyroScope& _Gyro, double Kp, Quat _Qib, Eigen::Vector3d _SunPos);


//�Եز����붨����������

//@brief: �Ե���̬������
//@para : 
//@return : ��������

//Eigen::Vector3d ToEarthControl();



};

