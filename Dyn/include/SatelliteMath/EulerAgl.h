#pragma once
#include"SatelliteMath/BaseMath.h"

//typedef struct EulerAgl
//{
//	Eigen::Vector3d Angle;//���������(rad)
//	unsigned Seq;//ת��
//	EulerAgl()
//	{
//		Angle << 0, 0, 0;
//		Seq = EUL_SQE_DEFAULT;
//	}
//};

//ǰ������
class CDcm;
class Quat;

class CEulerAgl
{
public:
	struct {
		Eigen::Vector3d Angle;//���������(rad)
		unsigned Seq;//ת��
	} AglData;//ŷ��������

public:
	//@brief: ��Ĭ��ת���0������ʼ��
	//@para : none
	//@return : none
	CEulerAgl();

	//@brief: �Է�����ָ��ת���ʼ��
	// 
	// 
	// 
	// 
	//@para : R1~R3(rad)������Ƕ�
	//		  Seq��ת��
	//@return : none
	CEulerAgl(double R1, double R2, double R3, unsigned Seq);

	//@brief: ������ŷ���ǳ�ʼ��
	//@para : Agl��ŷ����
	//@return : none
	CEulerAgl(CEulerAgl& Agl);

	//@brief: ���ظ�ֵ�����
	//@para : Agl��ŷ����
	//@return : none
	CEulerAgl& operator=(CEulerAgl Agl);

public:
	//@brief:  ŷ����ת�������Ҿ���
	//@para : Sequence:ת��(��BaseMath)
	//@return : �������Ҿ���
	CDcm ToDcm();

	//@brief:  ŷ����ת��Ԫ��
	//@para :  none
	//@return : ��Ԫ��
	Quat ToQuat();
};

