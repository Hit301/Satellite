#pragma once
#include"SatelliteMath/BaseMath.h"


//ǰ������
class CEulerAgl;
class Quat;

class CDcm
{
public:
	//��Eigen�Ļ����Դ�����ļ��㹦��
	Eigen::Matrix3d DcmData;

public:
	//@brief: �Ե�λ���ʼ��DcmData
	//@para : none
	//@return : none
	CDcm();

	//@brief: �Ը���ֵ��ʼ��DcmData
	//@para : none
	//@return : none
	CDcm(double A00, double A01, double A02,
		 double A10, double A11, double A12,
		 double A20, double A21, double A22);
	
	//@brief: �Ե�����ת��ʼ��DcmData
	//@para : Axis(Dcm_X_AXIS,Dcm_Y_AXIS,Dcm_Z_AXIS)��ת��(��BaseMath)
	//		  Theta(rad)��ת��
	//@return : none
	CDcm(unsigned Axis, double Theta);

	//@brief: �������������Ҿ����ʼ��DcmData
	//@para : _Dcm����һ���������Ҿ���
	//@return : none
	CDcm(CDcm& _Dcm);

	//@brief: ���ظ�ֵ�����
	//@para : _Dcm����һ���������Ҿ���
	//@return : none
	CDcm& operator=(CDcm _Dcm);

public:
	//@brief:  �������Ҿ���תŷ����
	//@para : Sequence:ת��(��BaseMath)
	//@return : ָ��ת���ŷ����
	CEulerAgl ToEulerAgl(unsigned Sequence);

	//@brief:  �������Ҿ���ת��Ԫ��
	//@para : none
	//@return : ת���õ�����Ԫ��
	Quat ToQuat();

public:
	//���������
	Eigen::Vector3d operator*(const Eigen::Vector3d& _Vector) const;//�������Ҿ������άʸ���ĳ˷�
	Eigen::Matrix3d operator*(const CDcm& _CDcm) const;//�������Ҿ���ͷ������Ҿ���ĳ˷�

};