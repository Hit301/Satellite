#include "AStro/AttitudeControl.h"

AttitudeControl::AttitudeControl()
{
	workmode = 1;
	Kp = 3;
}
Eigen::Vector3d AttitudeControl::ControlCommand(Eigen::Vector3d _Data, Eigen::Matrix3d _InstallMatrix)
{
	Eigen::Matrix3d I_inverse = _InstallMatrix.inverse();
	Eigen::Vector3d W = -_Data;
	Tcontrol = Kp * I_inverse * W;
	return Tcontrol;
}