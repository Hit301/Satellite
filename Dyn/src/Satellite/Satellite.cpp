#include "Satellite/Satellite.h"
#include"General/CConfig.h"
#include"General/InfluxDB.h"
Satellite::Satellite() :Orbit(), Attitude(), AttController()
{
	CConfig* pCfg = CConfig::GetInstance();

	//ʱ�����ʼ��
	SatelliteTime = pCfg->SatelliteTime;

	//�����س�ʼ��
	Orbit.Init(SatelliteTime);

	//��̬��س�ʼ��
	Attitude.Init(Orbit);

	//������س�ʼ��
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//����������
	AttController.workmode = CAttitudeController::EARTHPOINT;

	//������ʼ��
	pComponet = CComponet::GetInstance();
	pComponet->Init(Attitude, Orbit, Env, AttController,SatelliteTime);
}


void Satellite::StateRenew(double SampleTime)
{
	//ʱ�������
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//����������
	AttController.TorqueRefRenew(Attitude, Orbit, Env, pComponet);

	//��������Ϣ����
	Orbit.StateRenew(SampleTime, SatelliteTime);

	//��̬�����Ϣ����
	Attitude.StateRenew(SampleTime, Orbit, pComponet);

	//������Ϣ����
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//�������ݸ���
	pComponet->StateRenew(Attitude, Orbit, Env, AttController, SatelliteTime, SampleTime);
}

void Satellite::dataToDB(CInfluxDB& DB, double Period)
{
	// -->Period�ɼ���������
	if (DB.IsSend(Period)) {
		DB.setMeasurement("nb");
		// ��¼�ַ���
		// ��¼һ��˳��
		Orbit.record(DB);
		Attitude.record(DB);
		Env.record(DB);
		pComponet->record(DB);
		DB.printStr2();
		// AttController.record(DB);
		DB.sendUdp();
		DB.clearStr2();
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
