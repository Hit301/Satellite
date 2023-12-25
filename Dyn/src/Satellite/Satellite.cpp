#include "Satellite/Satellite.h"
#include"General/CConfig.h"
#include"General/InfluxDB.h"
Satellite::Satellite() :Orbit(), Attitude(), AttController()
{
	SampleTime = 0.1;
	SpeedTimes = 1;
	CConfig* pCfg = CConfig::GetInstance();

	//时间戳初始化
	SatelliteTime = pCfg->SatelliteTime;

	//轨道相关初始化
	Orbit.Init(SatelliteTime);

	//姿态相关初始化
	Attitude.Init(Orbit);

	//环境相关初始化
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//控制器设置
	AttController.workmode = CAttitudeController::EARTHPOINT;

	//单机初始化
	pComponet = CComponet::GetInstance();
	pComponet->Init(Attitude, Orbit, Env, AttController,SatelliteTime);
}

Satellite::Satellite(double Ts, int m_SpeedTimes):Satellite()
{
	SampleTime = Ts;
	SpeedTimes = m_SpeedTimes;
}


void Satellite::StateRenew()
{
	//时间戳更新
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//控制器计算
	AttController.TorqueRefRenew(pComponet);

	//轨道相关信息更新
	Orbit.StateRenew(SampleTime, SatelliteTime);

	//姿态相关信息更新
	Attitude.StateRenew(SampleTime, Orbit, pComponet);

	//环境信息更新
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//单机数据更新
	pComponet->StateRenew(Attitude, Orbit, Env, AttController, SatelliteTime, SampleTime);

}

void Satellite::data2DB(CInfluxDB& DB, double Period)
{
	// -->Period采集发送数据
	if (DB.IsSend(Period)) {
		DB.setMeasurement("zzz");
		record(DB);
		Orbit.record(DB);
		Attitude.record(DB);
		Env.record(DB);
		pComponet->record(DB);
		AttController.record(DB);
		DB.printStr2();
		DB.sendUdp();
		DB.clearStr2();
	}
}

void Satellite::record(CInfluxDB& DB)
{
	DB.addKeyValue("SIM001", SampleTime);
	DB.addKeyValue("SIM002", SpeedTimes);
	DB.addKeyValue("SIM003", SatelliteTime);
}


std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	//_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;//初始速率阻尼测试
	//_cout << "Qib " << Sat.Attitude.Qib;
	//_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	//_cout<<"VSunBody"<< Sat.Env.SunVecBody(0)<<" "<< Sat.Env.SunVecBody(1) <<" "<< Sat.Env.SunVecBody(2) << std::endl;
	//_cout << "VSunInl" << Sat.Env.SunVecInl(0) << " " << Sat.Env.SunVecInl(1) << " " << Sat.Env.SunVecInl(2) << std::endl;
	_cout << "Qob " << Sat.Attitude.Qob;
	return _cout;
}