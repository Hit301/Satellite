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
	CDcm();//�Ե�λ���ʼ��DcmData
	CDcm(double A00, double A01, double A02,//�Ը���ֵ��ʼ��DcmData
		 double A10, double A11, double A12,
		 double A20, double A21, double A22);
	CDcm(const Eigen::Vector3d& Axis, double Theta);//�Ե�����ת��ʼ��DcmData

	////@brief: ��ŷ���ǳ�ʼ��DcmData
	////		  Agl(rad):ŷ���ǽṹ��
	////@return : none
	//CDcm(CEulerAgl Agl);
	//
	////@brief: ����Ԫ����ʼ��DcmData
	////@para : quat:��Ԫ��
	////@return : none
	//CDcm(Quat quat);

	CDcm(CDcm& _Dcm);//�������������Ҿ����ʼ��DcmData

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
};