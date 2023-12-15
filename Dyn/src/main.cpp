#include"General/SimTime.h"
#include "Satellite/Satellite.h"

int main()
{
	//double SampleTime{ 0.1 };
	//Satellite Amadeus;
	//CSimTime* pSimTime = CSimTime::GetInstance();
	//pSimTime->InitSimSpeedManage(SampleTime, 10);
	//while (1)
	//{
	//	pSimTime->WaitForSimCountMute();
	//	if (pSimTime->SimCountJudge())
	//	{
	//		Amadeus.StateRenew(SampleTime);
	//		std::cout << Amadeus << std::endl;
	//	}
	//	pSimTime->ReleaseSimCountMute();
	//}
	//Eigen::MatrixXf g(2, 2);
	//g = Eigen::MatrixXf::Zero(13, 13);

	//std::cout << "------ g ------" << std::endl << g << std::endl;

	//Eigen::ArrayXXf matrix1(2, 3);
	//Eigen::ArrayXXf matrix2(2, 3);

	//matrix1 << 1, 2, 3,
	//	5, 6, 7;

	//matrix2 << 1, 1, 2,
	//	2, 1, 1;

	//std::cout << "------ matrix1 ------" << std::endl << matrix1(1, 2) << std::endl;

	//Eigen::ArrayXXf g(13, 13);
	//Eigen::ArrayXXf h(13, 13);
	//Eigen::ArrayXXf gdot(13, 13);
	//Eigen::ArrayXXf hdot(13, 13);

	//g = Eigen::MatrixXf::Zero(13, 13);
	//h = Eigen::MatrixXf::Zero(13, 13);
	//gdot = Eigen::MatrixXf::Zero(13, 13);
	//hdot = Eigen::MatrixXf::Zero(13, 13);

	//std::cout << g << std::endl;
	//std::cout << "------ matrix1 ------" << std::endl << matrix1(1,2) << std::endl;

	//const int rows = 12;
	//const int cols = 12;
	//int array[rows][cols] = { };

	//// 输出数组
	//for (int i = 0; i < rows; i++) {
	//	for (int j = 0; j < cols; j++) {
	//		std::cout << array[i][j] << " ";
	//	}
	//	std::cout << std::endl;
	//}
	
	COrbit Orbit;
	//std::cout << "------ matrix1 ------" << std::endl << Orbit.LLR.Lat << std::endl;
	//std::cout << "------ matrix1 ------" << std::endl << Orbit.LLR.Lng << std::endl;
	//std::cout << "------ matrix1 ------" << std::endl << Orbit.LLR.Rds << std::endl;
	Environment Env;
	Env.GetNEDMag(Orbit);
	std::cout << "------ matrix1 ------" << std::endl << Env.NEDMag << std::endl;

	//Eigen::ArrayXXf matrix1(1, 5);

	//matrix1 << 1, 2, 3, 4, 5;

	//std::cout << "------ matrix1 ------" << std::endl << matrix1(3) << std::endl;

	//int array[2][2] = { {1,2},{3,4} };

	//// 打印第一个元素
	//std::cout << array << std::endl;

	//double g[13][13] = { };
	//double h[13][13] = { };
	//double gdot[13][13] = { };
	//double hdot[13][13] = { };

	//std::cout << g[0][0] << std::endl;

	//const int rows = 90;
	//const int cols = 6;
	//double wmm_2020_data[rows][cols] = { };

	//std::ifstream file("src/Config/wmm_2020_data.txt");
	//if (file.is_open())
	//{
	//	for (int i = 0; i < rows; i++)
	//	{
	//		for (int j = 0; j < cols; j++)
	//		{
	//			if (!(file >> wmm_2020_data[i][j]))
	//			{
	//				std::cerr << "无法成功读取文件中的数据" << std::endl;
	//			}
	//		}
	//	}
	//	file.close();
	//}
	//else {
	//	std::cerr << "无法打开文件" << std::endl;
	//}
	///*std::cout << wmm_2020_data[1][j] << std::endl;*/
	////for (int i = 0; i < rows; i++) 
	////{
	////	for (int j = 0; j < cols; j++) 
	////	{
	////		std::cout << wmm_2020_data[i][j] << " ";
	////	}
	////	std::cout << std::endl;
	////}
	//for (int i = 0; i < 2; i++)
	//{
	//	int row = wmm_2020_data[i][0] + 1;
	//	int col = wmm_2020_data[i][1] + 1;
	//	g[row][col] = wmm_2020_data[i][2];
	//	h[row][col] = wmm_2020_data[i][3];
	//	gdot[row][col] = wmm_2020_data[i][4];
	//	hdot[row][col] = wmm_2020_data[i][5];
	//}
	//std::cout << g[0][0] << std::endl;

	//double x = sin(1.3962634015954636);
	//std::cout << x << std::endl;

	//double b = POW(1 - x * x, 1 / 2.0);
	//std::cout << b << std::endl;

	//uint64_t a = DoubleFactorial(2 * 11 - 1);
	//std::cout << a << std::endl;

	//uint64_t a = Factorial(28);
	//std::cout << a << std::endl;

	//int a = POW(-1 , 13);
	//std::cout << a << std::endl;

	//double a = SQRT(2.0 * Factorial(1) / Factorial(3));
	//std::cout << a << std::endl;

	//double a = SQRT(2);	
	//std::cout << a << std::endl;

	//int a = POW(-1, 1);
	//std::cout << a << std::endl;

}