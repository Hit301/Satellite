#include"General/CConfig.h"
#include<fstream>
CConfig::DeleteHelper CConfig::helper;


CConfig::CConfig()
{
	SatelliteTime = 1640966400000;

	Rx = 6678136.9999998566;
	Ry = 0.0002095685;
	Rz = -1.3800009224;

	Vx = 0.0007615644;
	Vy = 6789.5304738682;
	Vz = 3686.4138485846;

	a = 6678137;
	e = 0;
	i = RAD(28.5);
	RAAN = 0;
	omega = 0;
	M = 0;

	Wx = 0.05;
	Wy = -0.04;
	Wz = 0.01;

	Q0 = 1;
	Q1 = Q2 = Q3 = 0;

	Jxx = 30;
	Jyy = 40;
	Jzz = 50;
	Jxy = Jxz = Jyz = 0;

	//地磁相关
	MagOrder = 12;
	if ((MagOrder < 1) || (MagOrder > 12))
	{
		MagOrder = 2;
	}
	Eigen::Index rows = (MagOrder + 1) * (MagOrder + 2)/2 - 1;
	gauss_g.resize(MagOrder+1, MagOrder+1);
	gauss_h.resize(MagOrder + 1, MagOrder + 1);
	gauss_gdot.resize(MagOrder + 1, MagOrder + 1);
	gauss_hdot.resize(MagOrder + 1, MagOrder + 1);
	gauss_g.setZero();
	gauss_h.setZero();
	gauss_gdot.setZero();
	gauss_hdot.setZero();

	//打开配置文件读取数据
	std::ifstream file("Config/wmm_2020_data.txt");
	if (file.is_open())
	{
		//索引行
		int row, col;
		for (int i = 0; i < rows; i++)
		{
			//每行6列数据分别是 r c g h gdot hdot
			file >> row;
			file >> col;
			file >> gauss_g(row, col);
			file >> gauss_h(row, col);
			file >> gauss_gdot(row, col);
			file >> gauss_hdot(row, col);
		}
		file.close();
	}
	else
	{
		std::cerr << "Unable to open file" << std::endl;
		exit(0);
	}
}
CConfig* CConfig::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new CConfig;
	return m_instance;
}

void CConfig::ReleaseInstance()
{
	CConfig* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}
