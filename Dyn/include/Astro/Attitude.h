#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"
#include"SatelliteMath/Quaternions.h"
#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/EulerAgl.h"
#include "Astro/Orbit.h"
class COrbit;

class CAttitude
{
public:

	Eigen::Vector3d Omega_b;//����ϵ���ٶȣ���λrad/s
	CDcm Aio;//����ϵת���ϵת�ƾ���
	Quat Qib;//����ϵ������ϵ��Ԫ��
	Quat Qob;//���ϵ������ϵ��Ԫ��
private:
	Eigen::Matrix3d SatInaMat;//����ϵ�������󣬵�λkgm2
	Eigen::Vector3d WheelMomentum_b;//�������ڱ���ϵ�µĽǶ�������λNms
	Eigen::Vector3d TotalTorque;//Tf���������أ�TB �����أ�Tw�����ֱ���ϵ���� TotalTorque=TB+Tf-Tw
public:

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

	void RenewAio(COrbit& Orbit);

	void StateRenew(double Ts, COrbit& Orbit, CComponet* pComponet);

	void Init(COrbit& Obt);

	// д�����ݿ�
	void record(CInfluxDB& DB);

	static CDcm GetAio(const RV& InlRv)
	{
		Eigen::Vector3d Pos = InlRv.Pos;//���ǵ�λ��ʸ��
		Eigen::Vector3d Vel = InlRv.Vel;//���ǵ��ٶ�ʸ��
		Eigen::Vector3d zo = Eigen::Vector3d::Zero() - Pos / Pos.norm();//ƫ���ᵥλʸ��
		Eigen::Vector3d y_tmp = Vel.cross(Pos);
		Eigen::Vector3d yo = y_tmp / y_tmp.norm(); // �����ᵥλʸ��
		Eigen::Vector3d xo = yo.cross(zo);//�����ᵥλʸ��

		
		CDcm Aio;
		Aio.DcmData << xo.transpose(), yo.transpose(), zo.transpose();
		return Aio;
	}
private:
	Eigen::Vector3d LastOmega_b;//��һ�ĵı���ϵ���ٶȣ���λrad/s
};


