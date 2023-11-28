#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/EulerAgl.h"
#include"SatelliteMath/Quaternions.h"

//CDcm::CDcm()
//{
//	DcmData << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//}

CDcm::CDcm() : DcmData(Eigen::Matrix3d::Identity())
{
	// ����
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
	//@brief: �Ե�����ת��ʼ��DcmData
	//@para : Axis(Dcm_X_AXIS,Dcm_Y_AXIS,Dcm_Z_AXIS)��ת��(��BaseMath)
	//		  Theta(rad)��ת��
	//@return : none
	//@remark : δ����
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
	//@brief: �������������Ҿ����ʼ��DcmData
	//@para : _Dcm����һ���������Ҿ���
	//@return : none
	//@remark : δ����
	DcmData = _Dcm.DcmData;
}

CDcm& CDcm::operator=(CDcm _Dcm)
{
	//@brief: ���ظ�ֵ�����
	//@para : _Dcm����һ���������Ҿ���
	//@return : none
	//@remark : δ����
	DcmData = _Dcm.DcmData;
	return *this;
}

CEulerAgl CDcm::ToEulerAgl(unsigned Sequence)
{
	//@brief:  �������Ҿ���תŷ����
	//@para : Sequence:ת��(��BaseMath)
	//@return : ָ��ת���ŷ����
	return CEulerAgl();
}

Quat CDcm::ToQuat()
{
	//@brief:  �������Ҿ���ת��Ԫ��
	//@para : none
	//@return : ת���õ�����Ԫ��
	return Quat();
}
