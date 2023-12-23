#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"


class CAttitudeController
{
public:
	int workmode;//��̬����ģʽ
	Eigen::Vector3d TorqueRef;//�ο�����Nm
	Eigen::Matrix3d Kp;//������ϵ��
	Eigen::Matrix3d Kd;//������ϵ��
	double MaxTorque;//�������

	enum Mode
	{
		RATEDAMP,
		SUNPOINT,
		EARTHPOINT
	};
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

