#include "Satellite/Satellite.h"

Satellite::Satellite() :Orbit(), Attitude(), _Gyro()
{
	SatelliteTime = 1640966400000;
	//��ʼ������������ʱ��
	_Gyro.LastRenewTime = SatelliteTime;
	_Gyro.Data = _Gyro.InstallMatrix * Attitude.Omega_b;
	Angle = 10;
	// ����ģʽ��־����
	sunControlMode = false;
	earthControlMode = false;
}


void Satellite::StateRenew(double SampleTime)
{
	//������Ϣ����
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//̫��ʸ������
    //SolVec=SunCal.SunPos(SatelliteTime);
	SolVec = SunCal.SunPos(1659312000);//��ȷ���Ѿ���֤����MATLAB����Աȣ���С�������λ���ֺ�MATLAB�����̫��ʸ���Ľ��һ��

	//�����ʲ��ԣ�����ģʽ�л������޸ģ�
	//Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	//Quat _Qib = Attitude.Qib;
	//_Qbo = CAttitudeControl::GetQbo(_Qib);
	//if (!sunControlMode && !earthControlMode && (std::fabs(Wbi[0]) > 0.00174 || std::fabs(Wbi[1]) > 0.00174 || std::fabs(Wbi[2]) > 0.00174)) {
	//	// �л�����ʼ�����������ģʽ
	//	sunControlMode = false;
	//	earthControlMode = false;

	//	Attitude.TotalTorque = CAttitudeControl::RateDamping(_Gyro, 10);
	//}
	//else if (!sunControlMode && Angle >= 0.5) {
	//	// �л���������̬���Ʋ���ģʽ
	//	sunControlMode = true;
	//	earthControlMode = false;
	//	Angle = CAttitudeControl::GetAngle(_Qib, SolVec);
	//	Attitude.TotalTorque = CAttitudeControl::ToSunControl(_Gyro, 0.125, 1, _Qib, SolVec);
	//}
	//else if (!earthControlMode) {
	//	// �л����Ե���̬���Ʋ���ģʽ
	//	sunControlMode = false;
	//	earthControlMode = true;

	//	Attitude.TotalTorque = CAttitudeControl::ToEarthControl(_Gyro, 8, 8, _Qib);
	//}

	//�����ʲ��ԣ�����֤��
    Quat _Qib = Attitude.Qib;
	Angle = CAttitudeControl::GetAngle(_Qib, SolVec);
	_Qbo = CAttitudeControl::GetQbo(_Qib);
	//Attitude.TotalTorque = CAttitudeControl::RateDamping(_Gyro, 3);//��ʼ�����������ͨ��
	//Attitude.TotalTorque = CAttitudeControl::ToSunControl(_Gyro, 0.125, 1, _Qib, SolVec);//������̬���Ʋ���ͨ��
	Attitude.TotalTorque = CAttitudeControl::ToEarthControl(_Gyro, 0.5, 1, _Qib);//�Ե���̬���Ʋ���ͨ��
	
	//����ѧ����
	Orbit.TwoBod(SampleTime);
	Attitude.AttitudeKinematics(SampleTime);
	Attitude.AttitudeDynamicsRk4(SampleTime);

	//�������ݸ���
	_Gyro.StateRenew(SatelliteTime, Attitude.Omega_b);

}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;//��ʼ�����������
	_cout << "Qib " << Sat.Attitude.Qib;
	_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	_cout << "SolVec(m) " << Sat.SolVec(0) << " " << Sat.SolVec(1) << " " << Sat.SolVec(2) << std::endl;//����ϵ��̫��ʸ������
	_cout << "SolAngle(m) " << Sat.Angle << std::endl;//������̬���Ʋ���
	_cout << "_Qob " << Sat._Qbo;//������̬���Ʋ���
	return _cout;
}
