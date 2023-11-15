#pragma once

#include"SatelliteMath/Quaternions.h"

//��̬����ѧ���ƽ��ٶ�,��ʽ��Iw_dot+wX(Iw+hw)=T_sum  Tsum=Tf+TB-Tw
//I���������� w������ϵ���ٶ� hw�����ֱ���ϵ�Ƕ��� Tf���������أ�TB �����أ�Tw�����ֱ���ϵ����
Eigen::Vector3d Omega_bRK4(Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Omega_b, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s, double Ts);

#if 0
//��̬�˶�ѧ������Ԫ��
//Quat QibRK4(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts);
#endif
//��Ԫ���˷�������Ԫ��(�ɱ�֤��һ��)
Quat QibIntegrate(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts);