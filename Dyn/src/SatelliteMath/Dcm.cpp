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

CDcm::CDcm(const Eigen::Vector3d& Axis, double Theta)
{
	//@brief: 以单轴旋转初始化DcmData
	//@para : Axis(Dcm_X_AXIS,Dcm_Y_AXIS,Dcm_Z_AXIS)：转轴(查BaseMath)
	//		  Theta(rad)：转角
	//@return : none
	//@remark : 未测试
	Eigen::Vector3d axis_normalize = Axis.normalized();
	Eigen::Matrix3d Identity = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d anti_symmetric_Axis;
	anti_symmetric_Axis << 0,            -axis_normalize(0),  axis_normalize(1),
		             axis_normalize(2),          0,          -axis_normalize(0), 
	                -axis_normalize(1),   axis_normalize(0),          0 ;
	DcmData = cos(Theta) * Identity + (1 - cos(Theta)) * axis_normalize * axis_normalize.transpose() - sin(Theta) * anti_symmetric_Axis;
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
	//@brief: 以其他方向余弦矩阵初始化DcmData
	//@para : _Dcm：另一个方向余弦矩阵
	//@return : none
	//@remark : 未测试
	DcmData = _Dcm.DcmData;
}

CDcm& CDcm::operator=(CDcm _Dcm)
{
	//@brief: 重载赋值运算符
	//@para : _Dcm：另一个方向余弦矩阵
	//@return : none
	//@remark : 未测试
	DcmData = _Dcm.DcmData;
	return *this;
}

CEulerAgl CDcm::ToEulerAgl(unsigned Sequence)
{
	//@brief:  方向余弦矩阵转欧拉角
	//@para : Sequence:转序(查BaseMath)
	//@return : 指定转序的欧拉角
	return CEulerAgl();
}

Quat CDcm::ToQuat()
{
	//@brief:  方向余弦矩阵转四元数
	//@para : none
	//@return : 转换得到的四元数
	return Quat();
}
