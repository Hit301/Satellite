#pragma once
#include"SatelliteMath/EulerAgl.h"
#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/Quaternions.h"

class CAttitude
{
public:

	Eigen::Vector3d Omega_b;//����ϵ���ٶȣ���λrad/s
	Quat Qib;//����ϵ������ϵ��Ԫ��
	Eigen::Matrix3d SatInaMat;//����ϵ�������󣬵�λkgm2
	Eigen::Vector3d WheelMomentum_b;//�������ڱ���ϵ�µĽǶ�������λNms
	Eigen::Vector3d TotalTorque;//Tf���������أ�TB �����أ�Tw�����ֱ���ϵ���� TotalTorque=TB+Tf-Tw

	//
	// brief  : Ĭ����̬�๹�캯��
	//
	CAttitude();

	//
	// brief  : ��̬����ѧ���ƽ��ٶ�
	//
	int AttitudeDynamicsRk4(double Ts);

	//
	// brief  : ��̬�˶�������Ԫ��
	//
	int AttitudeKinematics(double Ts);

};


