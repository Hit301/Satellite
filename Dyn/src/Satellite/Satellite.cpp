#include "Satellite/Satellite.h"
#include"General/CConfig.h"

Satellite::Satellite() :Orbit(), Attitude(), AttController()
{
	CConfig* pCfg = CConfig::GetInstance();

	//时间戳初始化
	SatelliteTime = pCfg->SatelliteTime;

	//轨道相关初始化
	Orbit.J2000Inertial.Pos << pCfg->Rx, pCfg->Ry, pCfg->Rz;
	Orbit.J2000Inertial.Vel << pCfg->Vx, pCfg->Vy, pCfg->Vz;
	Orbit.Inl2Fix(SatelliteTime);
	Orbit.FixPos2LLR();
	Orbit.FixPos2LLA();

	//姿态相关初始化
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

	//环境相关初始化
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//单机初始化
	pComponet = CComponet::GetInstance();

	//陀螺初始化
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
	//时间戳更新
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//控制器计算
	Attitude.TotalTorque = AttController.TorqueRefRenew(Attitude, Orbit, Env, pComponet);

	//轨道相关信息更新
	Orbit.StateRenew(SampleTime, SatelliteTime);

	//姿态相关信息更新
	Attitude.StateRenew(SampleTime, Orbit);

	//环境信息更新
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//单机数据更新
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
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;//初始速率阻尼测试
	//_cout << "Qib " << Sat.Attitude.Qib;
	_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	//_cout<<"VSunBody"<< Sat.Env.SunVecBody(0)<<" "<< Sat.Env.SunVecBody(1) <<" "<< Sat.Env.SunVecBody(2) << std::endl;
	//_cout << "VSunInl" << Sat.Env.SunVecInl(0) << " " << Sat.Env.SunVecInl(1) << " " << Sat.Env.SunVecInl(2) << std::endl;
	_cout << "Qob " << Sat.Attitude.Qob;
	return _cout;
}
