#include"General/SimTime.h"
#include "Satellite/Satellite.h"
#include "Astro/Orbit.h"
#include"SatelliteMath/EulerAgl.h"

int main()
{
//	double SampleTime{ 0.1 };
//	Satellite Amadeus;
//	CSimTime* pSimTime = CSimTime::GetInstance();
//	pSimTime->InitSimSpeedManage(SampleTime, 1);
//	while (1)
//	{
//		pSimTime->WaitForSimCountMute();
//		if (pSimTime->SimCountJudge())
//		{
//			Amadeus.StateRenew(SampleTime);
////			std::cout << Amadeus << std::endl;
//			std::cout << "Gyro:\n " << Amadeus._Gyro.Data << std::endl;
//
//			//std::cout << Amadeus;
//		}
//		pSimTime->ReleaseSimCountMute();
//	}


	//CDcm DcmDemo2(1,2,3,4,5,6,7,8,9);
	//std::cout << DcmDemo2.DcmData << std::endl;
	//Eigen::Matrix3d a;
	//a = Eigen::Matrix3d::Identity();
	//std::cout << a << std::endl;

	//Eigen::Vector3d b;
	//b << 1, 2, 3;
	//std::cout << b << std::endl;	
	//
	Eigen::Vector3d c;
	c << 1, 2, 3;
	std::cout << c.norm() << std::endl;
	////std::cout << b.transpose() << std::endl;

	//Eigen::Vector3d a;
	//a = c.cross(b);
	//std::cout << a << std::endl;

	//COrbit COrbitDemo1;
	//std::cout << COrbitDemo1.LLA.Alt << std::endl;

	//CEulerAgl CEulerAglDemo1;
	//std::cout << CEulerAglDemo1.AglData.Angle << std::endl;

	//Eigen::MatrixXf matrix1(3, 1);

	//matrix1 << 1, 0, 0;
	//std::cout << "------ matrix1 ------" << std::endl << matrix1 << std::endl;
	//// ×ªÖÃ
	//std::cout << "------ matrix1 transpose------" << std::endl << matrix1.transpose() << std::endl;
}