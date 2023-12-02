#include "AStro/AttitudeControl.h"

//初始速率阻尼控制率设计
CAttitudeControl::CAttitudeControl()
{
	workmode = 1;
	Kp = 3;
}
Eigen::Vector3d CAttitudeControl::ControlCommand(Eigen::Vector3d _Data, Eigen::Matrix3d _InstallMatrix)
{
	Eigen::Matrix3d I_inverse = _InstallMatrix.inverse();
	Eigen::Vector3d W = -_Data;
	Tcontrol = Kp * I_inverse * W;
	for (int i = 0; i < 3; ++i)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -0.08, 0.08);
	}
	return Tcontrol;
}



//对日捕获与定向控制率设计


//对地捕获与定向控制率设计