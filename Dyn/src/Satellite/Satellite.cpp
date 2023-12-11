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
	//ʱ�������
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//����������
	Attitude.TotalTorque = CAttitudeControl::RateDamping(_Gyro, 3);

	//��������Ϣ����
	Orbit.StateRenew(SampleTime, SatelliteTime);

	//��̬�����Ϣ����
	Attitude.StateRenew(SampleTime);

	//������Ϣ����
	Env.StateRenew(Orbit, SatelliteTime);

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
