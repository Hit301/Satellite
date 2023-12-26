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
	//�������ݸ���
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
	//����������ļ�Ӧ�ã�����Ĭ�����ö�����������
	GyroNums = 1;
	FlywheelNums = 3;
	MagSensorNums = 1;
	SunSensorNums = 1;
	StarSensorNums = 1;
	GnssNums = 1;
	//֮����ݵ����Ĳ����������ã��ɶ�һ��ini
	if (GyroNums <= 0)
	{
		std::cout << "���������Ƿ� ֵ= " << GyroNums << "��ΪĬ��ֵ1" << std::endl;
		MessageBox(NULL, "���������Ƿ�,�������", "����", MB_OKCANCEL);
		exit(0);
	}
	if (FlywheelNums<= 0)
	{
		std::cout << "���������Ƿ� ֵ= " << FlywheelNums << "��ΪĬ��ֵ1" << std::endl;
		MessageBox(NULL, "���������Ƿ�,�������", "����", MB_OKCANCEL);
		exit(0);
	}
	if (MagSensorNums <= 0)
	{
		std::cout << "��ǿ�������Ƿ� ֵ= " << GyroNums << "��ΪĬ��ֵ1" << std::endl;
		MessageBox(NULL, "��ǿ�������Ƿ�,�������", "����", MB_OKCANCEL);
		exit(0);
	}

	if (StarSensorNums <= 0)
	{
		std::cout << "���������Ƿ� ֵ= " << GyroNums << "��ΪĬ��ֵ1" << std::endl;
		MessageBox(NULL, "���������Ƿ�,�������", "����", MB_OKCANCEL);
		exit(0);
	}
	if (SunSensorNums <= 0)
	{
		std::cout << "̫�������Ƿ� ֵ= " << GyroNums << "��ΪĬ��ֵ1" << std::endl;
		MessageBox(NULL, "̫�������Ƿ�,�������", "����", MB_OKCANCEL);
		exit(0);
	}
	if (GnssNums <= 0)
	{
		std::cout << "Gnss�����Ƿ� ֵ= " << GyroNums << "��ΪĬ��ֵ1" << std::endl;
		MessageBox(NULL, "Gnss�����Ƿ�,�������", "����", MB_OKCANCEL);
		exit(0);
	}
	Gyros.resize(GyroNums);
	for (size_t i{ 0 }; i < GyroNums; i++)
	{
		//����Ҫ�ĳ����ñ����͵�
		Gyros[i].InstallMatrix << Eigen::Matrix3d::Identity();
		Gyros[i].SamplePeriod = 0.25;
	}
	Wheels.resize(FlywheelNums);
	for (size_t i{ 0 }; i < FlywheelNums; i++)
	{
		//����Ҫ�ĳ����ñ����͵�,���Ҷ�Ϊ������
		Wheels[i].InstallVet << Eigen::Vector3d::Identity();
		//Wheels[i].SamplePeriod = 0.25;
	}
	//�ⲿ����Ҫɾ����
	Wheels[0].InstallVet << 1, 0, 0;
	Wheels[1].InstallVet << 0, 1, 0;
	Wheels[2].InstallVet << 0, 0, 1;

	SunSensors.resize(SunSensorNums);
	for (size_t i{ 0 }; i < SunSensorNums; i++)
	{
		//����Ҫ�ĳ����ñ����͵�
		SunSensors[i].InstallMatrix << Eigen::Matrix3d::Identity();
		SunSensors[i].SamplePeriod = 0.25;
	}

	StarSensors.resize(StarSensorNums);
	for (size_t i{ 0 }; i < StarSensorNums; i++)
	{
		//����Ҫ�ĳ����ñ����͵�
		StarSensors[i].InstallMatrix.DcmData << Eigen::Matrix3d::Identity();
		StarSensors[i].SamplePeriod = 0.25;
	}
	
	MagSensors.resize(MagSensorNums);
	for (size_t i{ 0 }; i < MagSensorNums; i++)
	{
		//����Ҫ�ĳ����ñ����͵�
		MagSensors[i].InstallMatrix<< Eigen::Matrix3d::Identity();
		MagSensors[i].SamplePeriod = 0.25;
	}

	GNSSs.resize(GnssNums);
	for (size_t i{ 0 }; i < GnssNums; i++)
	{
		//����Ҫ�ĳ����ñ����͵�
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

	//����Ҫ�ǵüӸ���
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
	// �����������ʦ�ֽ��ģ��кܶ����ݣ���ѭ��ȥaddKeyValue
	// Ϊ�˱�֤����... ...������������Զ���һ����ű�����ÿ��addKeyValue��һ���Ա�֤������洢
	// ���ݱ�ͷ
	DB.addKeyValue("SIM062", pComponet->Gyros[0].Data.x() * RAD2DEG);
	DB.addKeyValue("SIM063", pComponet->Gyros[0].Data.y() * RAD2DEG);
	DB.addKeyValue("SIM064", pComponet->Gyros[0].Data.z() * RAD2DEG);
	// ������ͷ
	DB.addKeyValue("SIM065", pComponet->StarSensors[0].Data.QuatData[0]);
	DB.addKeyValue("SIM066", pComponet->StarSensors[0].Data.QuatData[1]);
	DB.addKeyValue("SIM067", pComponet->StarSensors[0].Data.QuatData[2]);
	DB.addKeyValue("SIM068", pComponet->StarSensors[0].Data.QuatData[3]);
	// ̫��̫��ʸ��
	DB.addKeyValue("SIM069", pComponet->SunSensors[0].Data.x());
	DB.addKeyValue("SIM070", pComponet->SunSensors[0].Data.y());
	DB.addKeyValue("SIM071", pComponet->SunSensors[0].Data.z());
	// ��ǿ��
	DB.addKeyValue("SIM072", pComponet->MagSensors[0].Data.x());
	DB.addKeyValue("SIM073", pComponet->MagSensors[0].Data.y());
	DB.addKeyValue("SIM074", pComponet->MagSensors[0].Data.z());
	// GNSS
	DB.addKeyValue("SIM075", pComponet->GNSSs[0].Data.Pos.x());
	DB.addKeyValue("SIM076", pComponet->GNSSs[0].Data.Pos.y());
	DB.addKeyValue("SIM077", pComponet->GNSSs[0].Data.Pos.z());
	DB.addKeyValue("SIM078", pComponet->GNSSs[0].Data.Vel.x());
	DB.addKeyValue("SIM079", pComponet->GNSSs[0].Data.Vel.y());
	DB.addKeyValue("SIM080", pComponet->GNSSs[0].Data.Vel.z());
	// ����1ת�١�����
	DB.addKeyValue("SIM081", VEL2RPM(pComponet->Wheels[0].Speed));
	DB.addKeyValue("SIM082", pComponet->Wheels[0].Torque);
	// ����2ת�١�����
	DB.addKeyValue("SIM083", VEL2RPM(pComponet->Wheels[1].Speed));
	DB.addKeyValue("SIM084", pComponet->Wheels[1].Torque);
	// ����3ת�١�����
	DB.addKeyValue("SIM085", VEL2RPM(pComponet->Wheels[2].Speed));
	DB.addKeyValue("SIM086", pComponet->Wheels[2].Torque);
	// ��������
	DB.addKeyValue("SIM087", static_cast<double>(pComponet->GyroNums));
	// ��������
	DB.addKeyValue("SIM088", static_cast<double>(pComponet->StarSensorNums));
	// ̫������
	DB.addKeyValue("SIM089", static_cast<double>(pComponet->SunSensorNums));
	// ��ǿ������
	DB.addKeyValue("SIM090", static_cast<double>(pComponet->MagSensorNums));
	// GNSS����
	DB.addKeyValue("SIM091", static_cast<double>(pComponet->GnssNums));
	// ��������
	DB.addKeyValue("SIM092", static_cast<double>(pComponet->FlywheelNums));

}
