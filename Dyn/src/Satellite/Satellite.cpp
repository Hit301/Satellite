#include "Satellite/Satellite.h"
#include"General/CConfig.h"

Satellite::Satellite() :Orbit(), Attitude(), AttController()
{
	CConfig* pCfg = CConfig::GetInstance();

	//ʱ�����ʼ��
	SatelliteTime = pCfg->SatelliteTime;

	//�����س�ʼ��
	Orbit.J2000Inertial.Pos << pCfg->Rx, pCfg->Ry, pCfg->Rz;
	Orbit.J2000Inertial.Vel << pCfg->Vx, pCfg->Vy, pCfg->Vz;
	Orbit.Inl2Fix(SatelliteTime);
	Orbit.FixPos2LLR();
	Orbit.FixPos2LLA();

	//��̬��س�ʼ��
	Attitude.Omega_b << pCfg->Wx, pCfg->Wy, pCfg->Wz;
	Attitude.Qib.QuatData[0] = pCfg->Q0;
	Attitude.Qib.QuatData[1] = pCfg->Q1;
	Attitude.Qib.QuatData[2] = pCfg->Q2;
	Attitude.Qib.QuatData[3] = pCfg->Q3;
	Attitude.SatInaMat << pCfg->Jxx, pCfg->Jxy, pCfg->Jxz,
						  pCfg->Jxy, pCfg->Jyy, pCfg->Jyz,
		                  pCfg->Jxz, pCfg->Jyz, pCfg->Jzz;
	Attitude.GetAio(Orbit);
	Attitude.Qob = Attitude.Aio.ToQuat().QuatInv() * Attitude.Qib;

	//������س�ʼ��
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//������ʼ��
	pComponet = CComponet::GetInstance();

	//���ݳ�ʼ��
	GyroScope* pGyro = pComponet->pGyro;
	for (size_t i{ 0 }; i < pComponet->GyroNums; i++)
	{
		pGyro[i].LastRenewTime = SatelliteTime;
		pGyro[i].Data = RAD2DEG * pGyro[i].InstallMatrix * Attitude.Omega_b;
	}

	AttController.workmode = EARTHPOINT;
}


void Satellite::StateRenew(double SampleTime)
{
	//ʱ�������
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//����������
	Attitude.TotalTorque = AttController.TorqueRefRenew(Attitude, Orbit, Env, pComponet);

	//��������Ϣ����
	Orbit.StateRenew(SampleTime, SatelliteTime);

	//��̬�����Ϣ����
	Attitude.StateRenew(SampleTime, Orbit);

	//������Ϣ����
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//�������ݸ���
	GyroScope* pGyro = pComponet->pGyro;
	for (size_t i{ 0 }; i < pComponet->GyroNums; i++)
	{
		pGyro[i].StateRenew(SatelliteTime, Attitude.Omega_b);
	}

}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	//_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;//��ʼ�����������
	//_cout << "Qib " << Sat.Attitude.Qib;
	_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	//_cout<<"VSunBody"<< Sat.Env.SunVecBody(0)<<" "<< Sat.Env.SunVecBody(1) <<" "<< Sat.Env.SunVecBody(2) << std::endl;
	//_cout << "VSunInl" << Sat.Env.SunVecInl(0) << " " << Sat.Env.SunVecInl(1) << " " << Sat.Env.SunVecInl(2) << std::endl;
	_cout << "Qob " << Sat.Attitude.Qob;
	return _cout;
}
