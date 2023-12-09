#include "AStro/AttitudeControl.h"
#include "Satellite/Gyro.h"

//初始速率阻尼控制率设计
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


//对日捕获与定向控制率设计（代码中的注释待验证正确性后删除）
Eigen::Vector3d CAttitudeControl::ToSunControl(const GyroScope& _Gyro, double Kp, Quat _Qib, Eigen::Vector3d _SunPos)
{
	//计算从惯性系到本体系的姿态转移矩阵
	CDcm Aib = _Qib.ToDcm();
	//计算本体系下的太阳矢量
	Eigen::Vector3d Sunb = Aib * _SunPos;
	//太阳矢量反向
	Sunb = -Sunb;
	//计算卫星可实现的最大转速
	double Ws = 0.5 * DEG2RAD;
	//计算对日捕获目标姿态角速度
	Eigen::Vector3d Wref(Sunb[1], Sunb[0], 0);
	double sunb = sqrt(Sunb[0] * Sunb[0] + Sunb[1] * Sunb[1]);
	Wref = Wref / sunb * Ws;
	//控制力矩计算
	Eigen::Vector3d Tcontrol = -Kp * (Wref - _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data);
	//限幅处理
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
	
	//太阳矢量与卫星-z轴夹角 用于后续做模式转换(这里考虑一下两个输出的问题，待修改)
	double angle = acos(Sunb(2)) * RAD2DEG;
}

//对地捕获与定向控制率设计（代码中的注释待验证正确性后删除）
Eigen::Vector3d CAttitudeControl::ToEarthControl(const GyroScope& _Gyro, double Kp, double Kd, Quat _Qib)
{
	//轨道系相当于惯性系的姿态矩阵
	COrbit Orbit;//轨道
	Eigen::Vector3d Pos = Orbit.J2000Inertial.Pos;//卫星的位置矢量
	Eigen::Vector3d Vel = Orbit.J2000Inertial.Vel;//卫星的速度矢量
	Eigen::Vector3d zo = Pos / Pos.norm();//偏航轴单位矢量
	Eigen::Vector3d yo = Vel.cross(Pos) / (Vel.cross(Pos)).norm(); // 俯仰轴单位矢量
	Eigen::Vector3d xo = yo.cross(zo);//滚动轴单位矢量
	Eigen::Matrix3d Aoi;//姿态矩阵
	Aoi << xo, yo, zo;
	CDcm Aoi_=CDcm(Aoi(0,0), Aoi(0,1), Aoi(0,2),
		           Aoi(1,0), Aoi(1,1), Aoi(1,2),
		           Aoi(2,0), Aoi(2,1), Aoi(2,2));//将三维姿态矩阵转化为CDcm
	//将姿态矩阵转换为旋转四元数
	Quat Qoi= Aoi_.ToQuat();
	//计算卫星在轨道系下的姿态四元数
	Quat Qbo = Qoi.QuatInv() * _Qib;
	//计算卫星在轨道系下的角速度（理想情况下）
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	//控制力矩计算
	Eigen::Vector3d Tcontrol(-Kp * Qbo.QuatData[1] - Kd * Wbi[0], -Kp * Qbo.QuatData[2] - Kd * Wbi[1], -Kp * Qbo.QuatData[3] - Kd * Wbi[2]);
	//限幅处理
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}

//关于模式转换的问题(代码待补充)：
//初始速率阻尼—>对日姿态控制  判断卫星的姿态角速度小于0.00174(一个接近0的值)  workmode(1) to workmode(2)
//对日姿态控制—>对地姿态控制  判断太阳矢量与卫星-z轴夹角等于0(MATLAB中设置的是2)  workmode(2) to workmode(3)