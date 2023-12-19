#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"

//����ѡ��
#define RATEDAMP 1U
#define SUNPOINT 2U
#define EARTHPOINT 3U

class CAttitudeController
{
public:
	int workmode;//��̬����ģʽ
	Eigen::Vector3d TorqueRef;//�ο�����Nm
	Eigen::Matrix3d Kp;//������ϵ��
	Eigen::Matrix3d Kd;//������ϵ��
	double MaxTorque;//�������
public:
	CAttitudeController();

	//��������ģʽ����ο����أ�ʵ����Ӧ��ֻ��Ҫ������Ϣ
	Eigen::Vector3d TorqueRefRenew(CAttitude& Att, COrbit& Obt, Environment& Env, CComponet* pCom);
private:

	//@brief: �������������
	void RateDamping(const GyroScope& _Gyro);

	//@brief: ������̬������
	void ToSunControl(const GyroScope& _Gyro, Quat& _Qib, Eigen::Vector3d& _SunPos);

	//@brief: �Եز����붨������ʿ�����
	void ToEarthControl(const GyroScope& _Gyro, Quat& _Qob);

};
