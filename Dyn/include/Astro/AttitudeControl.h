#pragma once
#include"SatelliteMath/Quaternions.h"
#include <Eigen/Dense>
#include"Satellite/Gyro.h"

class AttitudeControl
{
public:
	int workmode;
	double Kp;
	Eigen::Vector3d Tcontrol;

public:
	//��ʼ����������������
	AttitudeControl();
	double ControlCommand(Eigen::Vector3d _data, Eigen::Matrix3d _InstallMatrix);

	//ʱ���ת��Ϊ������
	//����̫����J2000����ϵ�µ�λ��	
	//���ء����ٶȡ���Ԫ����ֵ���������
	//���ù������ѧ����̬����ѧ����̬�˶�ѧ
	//���������ڹ���ϵ�е���Ԫ���͹���ϵ��̫��ʸ��������㱾��ϵ�µ�̫��ʸ��
	//����ָ�����
	//ģʽת��
	//int changeworkmode()
    //����������ϵ�µ���Ԫ��
};
