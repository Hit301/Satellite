#include "Satellite/Satellite.h"

Satellite::Satellite() :Orbit(), Attitude(), _Gyro()
{
	SatelliteTime = 1640966400000;
	//初始化敏感器开机时间
	_Gyro.LastRenewTime = SatelliteTime;
	_Gyro.Data = _Gyro.InstallMatrix * Attitude.Omega_b;
	Angle = 10;
	// 定义模式标志变量
	sunControlMode = false;
	earthControlMode = false;
}


void Satellite::StateRenew(double SampleTime)
{
	//环境信息更新
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//太阳矢量测试
    //SolVec=SunCal.SunPos(SatelliteTime);
	SolVec = SunCal.SunPos(1659312000);//正确性已经验证，与MATLAB结果对比，在小数点后三位保持和MATLAB计算的太阳矢量的结果一致

	//控制率测试（包含模式切换，待修改）
	//Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	//Quat _Qib = Attitude.Qib;
	//_Qbo = CAttitudeControl::GetQbo(_Qib);
	//if (!sunControlMode && !earthControlMode && (std::fabs(Wbi[0]) > 0.00174 || std::fabs(Wbi[1]) > 0.00174 || std::fabs(Wbi[2]) > 0.00174)) {
	//	// 切换到初始速率阻尼测试模式
	//	sunControlMode = false;
	//	earthControlMode = false;

	//	Attitude.TotalTorque = CAttitudeControl::RateDamping(_Gyro, 10);
	//}
	//else if (!sunControlMode && Angle >= 0.5) {
	//	// 切换到对日姿态控制测试模式
	//	sunControlMode = true;
	//	earthControlMode = false;
	//	Angle = CAttitudeControl::GetAngle(_Qib, SolVec);
	//	Attitude.TotalTorque = CAttitudeControl::ToSunControl(_Gyro, 0.125, 1, _Qib, SolVec);
	//}
	//else if (!earthControlMode) {
	//	// 切换到对地姿态控制测试模式
	//	sunControlMode = false;
	//	earthControlMode = true;

	//	Attitude.TotalTorque = CAttitudeControl::ToEarthControl(_Gyro, 8, 8, _Qib);
	//}

	//控制率测试（均验证）
    Quat _Qib = Attitude.Qib;
	Angle = CAttitudeControl::GetAngle(_Qib, SolVec);
	_Qbo = CAttitudeControl::GetQbo(_Qib);
	//Attitude.TotalTorque = CAttitudeControl::RateDamping(_Gyro, 3);//初始速率阻尼测试通过
	//Attitude.TotalTorque = CAttitudeControl::ToSunControl(_Gyro, 0.125, 1, _Qib, SolVec);//对日姿态控制测试通过
	Attitude.TotalTorque = CAttitudeControl::ToEarthControl(_Gyro, 0.5, 1, _Qib);//对地姿态控制测试通过
	
	//动力学更新
	Orbit.TwoBod(SampleTime);
	Attitude.AttitudeKinematics(SampleTime);
	Attitude.AttitudeDynamicsRk4(SampleTime);

	//单机数据更新
	_Gyro.StateRenew(SatelliteTime, Attitude.Omega_b);

}

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	std::cout << std::fixed;
	_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << Sat.Orbit.J2000Inertial;
	_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;//初始速率阻尼测试
	_cout << "Qib " << Sat.Attitude.Qib;
	_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	_cout << "SolVec(m) " << Sat.SolVec(0) << " " << Sat.SolVec(1) << " " << Sat.SolVec(2) << std::endl;//惯性系下太阳矢量测试
	_cout << "SolAngle(m) " << Sat.Angle << std::endl;//对日姿态控制测试
	_cout << "_Qob " << Sat._Qbo;//对日姿态控制测试
	return _cout;
}
