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


//���ղ����붨���������ƣ������е�ע�ʹ���֤��ȷ�Ժ��ɾ����
Eigen::Vector3d CAttitudeControl::ToSunControl(const GyroScope& _Gyro, double Kp, double Kd, Quat _Qib, Eigen::Vector3d _SunPos)
{
	//����ӹ���ϵ������ϵ����̬ת�ƾ���
	CDcm Aib = _Qib.ToDcm();
	//���㱾��ϵ�µ�̫��ʸ��
	Eigen::Vector3d Sunb = Aib * _SunPos;
	//�ο���
	Eigen::Vector3d Wref(0, 0, 0.1 * DEG2RAD);
	Eigen::Vector3d Rb(0, 0, -1);
	//�������ؼ���
	Eigen::Vector3d Tcontrol = -Kp* Sunb.cross(Rb) +Kd * (Wref - _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data);
	//�޷�����
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}

//����̫��ʸ��������-z��нǣ���λ:deg��
double CAttitudeControl::GetAngle(Quat _Qib, Eigen::Vector3d _SunPos)
{
	CDcm Aib = _Qib.ToDcm();
	Eigen::Vector3d Sunb = Aib * _SunPos;
	Sunb = -Sunb;
	double angle = acos(Sunb(2)) * RAD2DEG;
	return angle;
}

//�Եز����붨���������ƣ������е�ע�ʹ���֤��ȷ�Ժ��ɾ����
Eigen::Vector3d CAttitudeControl::ToEarthControl(const GyroScope& _Gyro, double Kp, double Kd, Quat _Qib)
{
	//���ϵ�൱�ڹ���ϵ����̬����
	COrbit Orbit;//���
	Eigen::Vector3d Pos = Orbit.J2000Inertial.Pos;//���ǵ�λ��ʸ��
	Eigen::Vector3d Vel = Orbit.J2000Inertial.Vel;//���ǵ��ٶ�ʸ��
	Eigen::Vector3d zo = Pos / Pos.norm();//ƫ���ᵥλʸ��
	Eigen::Vector3d yo = Vel.cross(Pos) / (Vel.cross(Pos)).norm(); // �����ᵥλʸ��
	Eigen::Vector3d xo = yo.cross(zo);//�����ᵥλʸ��
	Eigen::Matrix3d Aoi;//��̬����
	Aoi << xo, yo, zo;
	CDcm Aoi_=CDcm(Aoi(0,0), Aoi(0,1), Aoi(0,2),
		           Aoi(1,0), Aoi(1,1), Aoi(1,2),
		           Aoi(2,0), Aoi(2,1), Aoi(2,2));//����ά��̬����ת��ΪCDcm
	//����̬����ת��Ϊ��ת��Ԫ��
	Quat Qoi= Aoi_.ToQuat();
	//���������ڹ��ϵ�µ���̬��Ԫ��
	Quat Qbo = Qoi.QuatInv() * _Qib;
	//���������ڹ��ϵ�µĽ��ٶȣ���������£�
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	//�������ؼ���
	Eigen::Vector3d Tcontrol(-Kp * Qbo.QuatData[1] - Kd * Wbi[0], -Kp * Qbo.QuatData[2] - Kd * Wbi[1], -Kp * Qbo.QuatData[3] - Kd * Wbi[2]);
	//�޷�����
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}

//���������ڹ��ϵ�µ���̬��Ԫ��
Quat CAttitudeControl::GetQbo(Quat _Qib) 
{
	//���ϵ�൱�ڹ���ϵ����̬����
	COrbit Orbit;//���
	Eigen::Vector3d Pos = Orbit.J2000Inertial.Pos;//���ǵ�λ��ʸ��
	Eigen::Vector3d Vel = Orbit.J2000Inertial.Vel;//���ǵ��ٶ�ʸ��
	Eigen::Vector3d zo = Pos / Pos.norm();//ƫ���ᵥλʸ��
	Eigen::Vector3d yo = Vel.cross(Pos) / (Vel.cross(Pos)).norm(); // �����ᵥλʸ��
	Eigen::Vector3d xo = yo.cross(zo);//�����ᵥλʸ��
	Eigen::Matrix3d Aoi;//��̬����
	Aoi << xo, yo, zo;
	CDcm Aoi_ = CDcm(Aoi(0, 0), Aoi(0, 1), Aoi(0, 2),
		             Aoi(1, 0), Aoi(1, 1), Aoi(1, 2),
		             Aoi(2, 0), Aoi(2, 1), Aoi(2, 2));//����ά��̬����ת��ΪCDcm
    //����̬����ת��Ϊ��ת��Ԫ��
	Quat Qoi = Aoi_.ToQuat();
	//���������ڹ��ϵ�µ���̬��Ԫ��
	Quat Qbo = Qoi.QuatInv() * _Qib;
	return Qbo;
}

//��������ظ������⣨��֤���պͶԵ���̬���Ƶ���ȷ�Ժ��ٸģ�

//����ģʽת��������(���������)��
//��ʼ�������ᡪ>������̬����  �ж����ǵ���̬���ٶ�С��0.00174(һ���ӽ�0��ֵ)  workmode(1) to workmode(2)
//������̬���ơ�>�Ե���̬����  �ж�̫��ʸ��������-z��нǵ���0(MATLAB�����õ���2)  workmode(2) to workmode(3)
//�Ե���̬����  �жϹ��ϵ�µ���̬��Ԫ��qob�Ƿ�Ϊ[1 0 0 0]