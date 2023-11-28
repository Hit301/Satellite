#include"General/SimTime.h"
#include "Satellite/Satellite.h"
#include "SatelliteMath/Dcm.h"

int main()
{
	//double SampleTime{ 0.1 };
	//Satellite Amadeus;
	//CSimTime* pSimTime = CSimTime::GetInstance();
	//pSimTime->InitSimSpeedManage(SampleTime, 1);
	//while (1)
	//{
	//	pSimTime->WaitForSimCountMute();
	//	if (pSimTime->SimCountJudge())
	//	{
	//		Amadeus.StateRenew(SampleTime);
	//		std::cout << Amadeus << std::endl;
	//		std::cout << "Gyro: " << Amadeus._Gyro.Data << std::endl;

	//		//std::cout << Amadeus;
	//	}
	//	pSimTime->ReleaseSimCountMute();
	//}

	CDcm DcmDemo1;
	std::cout << DcmDemo1.DcmData << std::endl;//打印时要带上变量名字
	//CDcm DcmDemo2(1,2,3,4,5,6,7,8,9);
	//std::cout << DcmDemo2.DcmData << std::endl;
	//Eigen::Matrix3d a;
	//a = Eigen::Matrix3d::Identity();
	//std::cout << a << std::endl;

	//Eigen::Vector3d b;
	//b = Eigen::Vector3d::Identity();
	//std::cout << b << std::endl;
	//std::cout << b.transpose() << std::endl;

	//Eigen::MatrixXf matrix1(3, 1);

	//matrix1 << 1, 0, 0;
	//std::cout << "------ matrix1 ------" << std::endl << matrix1 << std::endl;
	//// 转置
	//std::cout << "------ matrix1 transpose------" << std::endl << matrix1.transpose() << std::endl;


}