#include "Componet/Componet.h"
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Astro/Environment.h"
#include "AStro/AttitudeControl.h"
#include"General/InfluxDB.h"

CComponet::DeleteHelper CComponet::helper;

CComponet* pComponet = CComponet::GetInstance();


CComponet* CComponet::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new CComponet;
	return m_instance;
}

void CComponet::Init(CAttitude& Att, COrbit& Obt, Environment& Env, CAttitudeController& ACtrl, int64_t timestamp)
{
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		Gyros[i].Init(Att.Omega_b, timestamp);
	}

	Eigen::VectorXd WheelsTref = WheelsTrefCal(ACtrl.TorqueRef);
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		Wheels[i].Init(WheelsTref[i], timestamp);
	}

	for (size_t i{ 0 }; i < SunSensorNums; i++)
	{
		SunSensors[i].Init(Env.SunVecBody, timestamp);
	}

	for (size_t i{ 0 }; i < StarSensorNums; i++)
	{
		StarSensors[i].Init(Att.Qib, timestamp);
	}

	for (size_t i{ 0 }; i < MagSensorNums; i++)
	{
		MagSensors[i].Init(Env.BodyMag, timestamp);
	}

	for (size_t i{ 0 }; i < GnssNums; i++)
	{
		GNSSs[i].Init(Obt.J2000Inertial, timestamp);
	}
}

void CComponet::StateRenew(CAttitude& Att, COrbit& Obt, Environment& Env, CAttitudeController& ACtrl, int64_t timestamp, double Ts)
{
	//陀螺数据更新
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		Gyros[i].StateRenew(timestamp, Att.Omega_b);
	}
	for (size_t i{ 0 }; i < MagSensorNums; i++)
	{
		MagSensors[i].StateRenew(timestamp,Env.BodyMag);
	}
	for (size_t i{ 0 }; i < StarSensorNums; i++)
	{
		StarSensors[i].StateRenew(timestamp,Att.Qib);
	}
	for (size_t i{ 0 }; i < SunSensorNums; i++)
	{
		SunSensors[i].StateRenew(timestamp, Env.SunVecBody);
	}

	Eigen::VectorXd WheelsTref = WheelsTrefCal(ACtrl.TorqueRef);
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		Wheels[i].StateRenew(timestamp, Ts, WheelsTref[i]);
	}

	for (size_t i{ 0 }; i < GnssNums; i++)
	{
		GNSSs[i].StateRenew(timestamp, Obt.J2000Inertial);
	}
}

CComponet::CComponet() :
	GyroNums(1)
{
	//这里读配置文件应该，先走默认配置读各单机数量
	GyroNums = 1;
	FlywheelNums = 3;
	MagSensorNums = 1;
	SunSensorNums = 1;
	StarSensorNums = 1;
	GnssNums = 1;
	//之后根据单机的参数进行配置，可读一个ini
	if (GyroNums <= 0)
	{
		std::cout << "陀螺数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "陀螺数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	if (FlywheelNums<= 0)
	{
		std::cout << "飞轮数量非法 值= " << FlywheelNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "飞轮数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	if (MagSensorNums <= 0)
	{
		std::cout << "磁强计数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "磁强计数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}

	if (StarSensorNums <= 0)
	{
		std::cout << "星敏数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "星敏数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	if (SunSensorNums <= 0)
	{
		std::cout << "太敏数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "太敏数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	if (GnssNums <= 0)
	{
		std::cout << "Gnss数量非法 值= " << GyroNums << "改为默认值1" << std::endl;
		MessageBox(NULL, "Gnss数量非法,程序结束", "警告", MB_OKCANCEL);
		exit(0);
	}
	Gyros.resize(GyroNums);
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		//这里要改成配置表类型的
		Gyros[i].InstallMatrix << Eigen::Matrix3d::Identity();
		Gyros[i].SamplePeriod = 0.25;
	}
	Wheels.resize(FlywheelNums);
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		//这里要改成配置表类型的,暂且定为三正交
		Wheels[i].InstallVet << Eigen::Vector3d::Identity();
		//Wheels[i].SamplePeriod = 0.25;
	}
	//这部分是要删除的
	Wheels[0].InstallVet << 1, 0, 0;
	Wheels[1].InstallVet << 0, 1, 0;
	Wheels[2].InstallVet << 0, 0, 1;

	SunSensors.resize(SunSensorNums);
	for (size_t i{ 0 }; i < SunSensorNums; i++)
	{
		//这里要改成配置表类型的
		SunSensors[i].InstallMatrix << Eigen::Matrix3d::Identity();
		SunSensors[i].SamplePeriod = 0.25;
	}

	StarSensors.resize(StarSensorNums);
	for (size_t i{ 0 }; i < StarSensorNums; i++)
	{
		//这里要改成配置表类型的
		StarSensors[i].InstallMatrix.DcmData << Eigen::Matrix3d::Identity();
		StarSensors[i].SamplePeriod = 0.25;
	}
	
	MagSensors.resize(MagSensorNums);
	for (size_t i{ 0 }; i < MagSensorNums; i++)
	{
		//这里要改成配置表类型的
		MagSensors[i].InstallMatrix<< Eigen::Matrix3d::Identity();
		MagSensors[i].SamplePeriod = 0.25;
	}

	GNSSs.resize(GnssNums);
	for (size_t i{ 0 }; i < GnssNums; i++)
	{
		//这里要改成配置表类型的
		GNSSs[i].SamplePeriod = 0.25;
	}
}

CComponet::~CComponet()
{
}

Eigen::VectorXd CComponet::WheelsTrefCal(Eigen::Vector3d& TrefBody)
{
	Eigen::MatrixXd InstallMatrix(3, FlywheelNums);
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		InstallMatrix.col(i) = Wheels[i].InstallVet;
	}
	Eigen::MatrixXd tmp1(FlywheelNums, FlywheelNums);
	tmp1 = InstallMatrix * InstallMatrix.transpose();

	Eigen::MatrixXd tmp2(FlywheelNums, 3);
	tmp2 = InstallMatrix.transpose() * tmp1.inverse();

	Eigen::VectorXd WheelTrefs(FlywheelNums);

	//这里要记得加负号
	WheelTrefs = -tmp2 * TrefBody;
	return WheelTrefs;

}

void CComponet::ReleaseInstance()
{
	CComponet* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}

void CComponet::record(CInfluxDB& DB) {
	// 如果后续按照师兄讲的，有很多陀螺，用循环去addKeyValue
	// 为了保证陀螺... ...序号连续，可以定义一个序号变量，每次addKeyValue加一，以保证按大类存储
	// 陀螺表头
	DB.addKeyValue("SIM062", Gyros[0].Data.x());
	DB.addKeyValue("SIM063", Gyros[0].Data.y());
	DB.addKeyValue("SIM064", Gyros[0].Data.z());
	// 星敏表头
	DB.addKeyValue("SIM065", StarSensors[0].Data.QuatData[0]);
	DB.addKeyValue("SIM066", StarSensors[0].Data.QuatData[1]);
	DB.addKeyValue("SIM067", StarSensors[0].Data.QuatData[2]);
	DB.addKeyValue("SIM068", StarSensors[0].Data.QuatData[3]);
	// 太敏太阳矢量
	DB.addKeyValue("SIM069", SunSensors[0].Data.x());
	DB.addKeyValue("SIM070", SunSensors[0].Data.y());
	DB.addKeyValue("SIM071", SunSensors[0].Data.z());
	// 磁强计
	DB.addKeyValue("SIM072", MagSensors[0].Data.x());
	DB.addKeyValue("SIM073", MagSensors[0].Data.y());
	DB.addKeyValue("SIM074", MagSensors[0].Data.z());
	// GNSS
	DB.addKeyValue("SIM075", GNSSs[0].Data.Pos.x());
	DB.addKeyValue("SIM076", GNSSs[0].Data.Pos.y());
	DB.addKeyValue("SIM077", GNSSs[0].Data.Pos.z());
	DB.addKeyValue("SIM078", GNSSs[0].Data.Vel.x());
	DB.addKeyValue("SIM079", GNSSs[0].Data.Vel.y());
	DB.addKeyValue("SIM080", GNSSs[0].Data.Vel.z());
	// 飞轮1转速、力矩
	DB.addKeyValue("SIM081", VEL2RPM(Wheels[0].Speed));
	DB.addKeyValue("SIM082", Wheels[0].Torque);
	// 飞轮2转速、力矩
	DB.addKeyValue("SIM083", VEL2RPM(Wheels[1].Speed));
	DB.addKeyValue("SIM084", Wheels[1].Torque);
	// 飞轮3转速、力矩
	DB.addKeyValue("SIM085", VEL2RPM(Wheels[2].Speed));
	DB.addKeyValue("SIM086", Wheels[2].Torque);
	// 陀螺数量
	DB.addKeyValue("SIM087", GyroNums);
	// 星敏数量
	DB.addKeyValue("SIM088", StarSensorNums);
	// 太敏数量
	DB.addKeyValue("SIM089", SunSensorNums);
	// 磁强计数量
	DB.addKeyValue("SIM090", MagSensorNums);
	// GNSS数量
	DB.addKeyValue("SIM091", GnssNums);
	// 飞轮数量
	DB.addKeyValue("SIM092", FlywheelNums);

}
