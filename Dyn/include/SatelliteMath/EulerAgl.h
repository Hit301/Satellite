#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"

class CEulerAgl
{
private:
	static bool CheckSeq(unsigned Seq);
public:
	struct {
		Eigen::Vector3d Angle;//三轴角数据(rad)
		unsigned Seq;//转序
	} AglData;//欧拉角数据

public:
	//@brief: 以默认转序和0向量初始化
	//@para : none
	//@return : none
	CEulerAgl();

	//@para : R1~R3(rad)：三轴角度
	//		  Seq：转序
	//@return : none
	CEulerAgl(double R1, double R2, double R3, unsigned Seq);

	//@brief: 以其他欧拉角初始化
	//@para : Agl：欧拉角
	//@return : none
	CEulerAgl(const CEulerAgl& Agl);

	//@brief: 重载赋值运算符
	//@para : Agl：欧拉角
	//@return : none
	CEulerAgl& operator=(const CEulerAgl Agl);

public:
	//@brief:  欧拉角转方向余弦矩阵
	//@para : Sequence:转序(查BaseMath)
	//@return : 方向余弦矩阵
	CDcm ToDcm() const;

	//@brief:  欧拉角转四元数
	//@para :  none
	//@return : 四元数
	Quat ToQuat() const;
};

