#pragma once
#include"Astro/Attitude.h"
#include"Astro/Orbit.h"
#include "Satellite/Gyro.h"
#include "Astro/Environment.h"
class Satellite
{
public:
	int64_t SatelliteTime;//������utcʱ�����λms
	Inl_t J2000;//����ϵRV����λm��m/s
	Eigen::Vector3d Omega_b;//����ϵ���ٶȣ���λrad/s
	Quat Qib;//��һ������ϵ������ϵ��Ԫ��
	Eigen::Matrix3d SatInaMat;//����ϵ�������󣬵�λkgm2
	Eigen::Vector3d WheelMomentum;//�������ڱ���ϵ�µĽǶ�������λNms
	Eigen::Vector3d TotalTorque;//Tf���������أ�TB �����أ�Tw�����ֱ���ϵ���� TotalTorque=TB+Tf-Tw
	Environment Env;
	GyroScope _Gyro;
public:
	Satellite();

	~Satellite()=default;
	void StateRenew(double SampleTime);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);