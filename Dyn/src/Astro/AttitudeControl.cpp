#include"Astro/AttitudeControl.h"


//��ʼ����������������
AttitudeControl::AttitudeControl()
{
	Kp = 3;
}
double ControlCommand(Eigen::Vector3d _Data, Eigen::Matrix3d _InstallMatrix)
{
	Eigen::Matrix3d M_inv = _InstallMatrix.inverse();
	Eigen::Vector3d W = -_Data;
	Tcontrol = Kp * M_inv * W;
}
//���ղ����붨����������


//�Եز����붨����������




