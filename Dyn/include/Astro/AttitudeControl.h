#pragma once
#include"SatelliteMath/BaseMath.h"
#include"SatelliteMath/Quaternions.h"
#include"Astro/Orbit.h"
#include"SatelliteMath/Dcm.h"

//ǰ������
class GyroScope;

class CAttitudeControl
{
public:
	int workmode;//��̬����ģʽ

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
//@para : _Gyro:���������� Kp:���������� Kd:���������� _Qib:����ϵ�µ���Ԫ�� _SunPos:����ϵ�µ�̫��ʸ��
//@return : ��������

static Eigen::Vector3d ToSunControl(const GyroScope& _Gyro, double Kp, double Kd, Quat _Qib, Eigen::Vector3d _SunPos);

//@brief: ̫��ʸ��������-z��н�
static double GetAngle(Quat _Qib, Eigen::Vector3d _SunPos);

//�Եز����붨����������

//@brief: �Ե���̬������
//@para : _Gyro:���������� Kp:���������� Kd:���������� _Qib:����ϵ�µ���Ԫ��
//@return : ��������

static Eigen::Vector3d ToEarthControl(const GyroScope& _Gyro, double Kp, double Kd, Quat _Qib);

//@brief: ���ϵ����̬��Ԫ��
static Quat GetQbo(Quat _Qib);

};

