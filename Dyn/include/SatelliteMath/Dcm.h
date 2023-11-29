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
	CDcm(unsigned Axis, double Theta);//�Ե�����ת��ʼ��DcmData
	CDcm(CDcm& _Dcm);//�������������Ҿ����ʼ��DcmData

	////@brief: ��ŷ���ǳ�ʼ��DcmData
	////		  Agl(rad):ŷ���ǽṹ��
	////@return : none
	//CDcm(CEulerAgl Agl);
	//
	////@brief: ����Ԫ����ʼ��DcmData
	////@para : quat:��Ԫ��
	////@return : none
	//CDcm(Quat quat);

	CDcm& operator=(CDcm _Dcm);//���ظ�ֵ�����

public:
	CEulerAgl ToEulerAgl(unsigned Sequence);//�������Ҿ���תŷ����
	Quat ToQuat();//�������Ҿ���ת��Ԫ��
};