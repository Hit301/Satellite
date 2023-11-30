#include"Astro/AttitudeControl.h"


//初始速率阻尼控制率设计
AttitudeControl::AttitudeControl()
{
	Kp = 3;
}
double ControlCommand(Eigen::Vector3d _Data, Eigen::Matrix3d _InstallMatrix)
{
	Eigen::Matrix3d M_inv = _InstallMatrix.inverse();
	Eigen::Vector3d W = -_Data;
	Tcontrol = Kp * M_inv * W;
}
//对日捕获与定向控制率设计


//对地捕获与定向控制率设计




