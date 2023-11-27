#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/EulerAgl.h"
#include"SatelliteMath/Quaternions.h"

//CDcm::CDcm()
//{
//	DcmData << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//}

CDcm::CDcm() : DcmData(Eigen::Matrix3d::Identity())
{
	// 或者
	// DcmData << 1, 0, 0, 0, 1, 0, 0, 0, 1;
}


CDcm::CDcm(double A00, double A01, double A02, double A10, double A11, double A12, double A20, double A21, double A22)
{
	DcmData << A00, A01, A02, 
		       A10, A11, A12, 
		       A20, A21, A22;
}

CDcm::CDcm(unsigned Axis, double Theta)
{
	//@brief: 以单轴旋转初始化DcmData
	//@para : Axis(Dcm_X_AXIS,Dcm_Y_AXIS,Dcm_Z_AXIS)：转轴(查BaseMath)
	//		  Theta(rad)：转角
	//@return : none
}

//CDcm::CDcm(CEulerAgl Agl)
//{
//}
//
//CDcm::CDcm(Quat quat)
//{
//}

CDcm::CDcm(CDcm& _Dcm)
{
}

CDcm& CDcm::operator=(CDcm _Dcm)
{
	return *this;
}

CEulerAgl CDcm::ToEulerAgl(unsigned Sequence)
{
	return CEulerAgl();
}

Quat CDcm::ToQuat()
{
	return Quat();
}
