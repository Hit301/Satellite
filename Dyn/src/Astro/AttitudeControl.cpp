#include "AStro/AttitudeControl.h"
#include "Satellite/Gyro.h"

//��ʼ����������������
CAttitudeControl::CAttitudeControl():workmode(1)
{
}

Eigen::Vector3d CAttitudeControl::RateDamping(const GyroScope& _Gyro, double Kp)
{

	Eigen::Vector3d Tcontrol  = -Kp* _Gyro.InstallMatrix.inverse() *DEG2RAD* _Gyro.Data;
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}


//���ղ����붨����������
Eigen::Vector3d CAttitudeControl::SunControl(const GyroScope& _Gyro, double Kp, Quat _Qib, Eigen::Vector3d _SunPos)
{
	//����ӹ���ϵ������ϵ����̬ת�ƾ���
	CDcm Aib = _Qib.ToDcm();
	//���㱾��ϵ�µ�̫��ʸ��
	Eigen::Vector3d Sunb = Aib * _SunPos;
	//̫��ʸ������
	Sunb = -Sunb;
	double sunb = sqrt(Sunb[0] * Sunb[0] + Sunb[1] * Sunb[1]);
	//���ǿ�ʵ�ֵ����ת��
	double Ws = HALFPI / 180;
	//���ղ���Ŀ����̬���ٶ�
	Eigen::Vector3d Wref(Sunb[1], Sunb[0], 0);
	Wref = Wref / sunb * Ws;
	Eigen::Vector3d Tcontrol = -Kp * _Gyro.InstallMatrix.inverse() * DEG2RAD * (Wref -_Gyro.Data);
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}

//�Եز����붨����������
//Eigen::Vector3d CAttitudeControl::ToEarthControl()
//{
//
//}