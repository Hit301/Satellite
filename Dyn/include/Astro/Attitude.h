#pragma once

#include"SatelliteMath/Quaternions.h"

//姿态动力学递推角速度,形式是Iw_dot+wX(Iw+hw)=T_sum  Tsum=Tf+TB-Tw
//I：惯量矩阵 w：本体系角速度 hw：飞轮本体系角动量 Tf：干扰力矩：TB 磁力矩：Tw：飞轮本体系力矩
Eigen::Vector3d Omega_bRK4(Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Omega_b, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s, double Ts);

#if 0
//姿态运动学递推四元数
//Quat QibRK4(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts);
#endif
//四元数乘法递推四元数(可保证归一性)
Quat QibIntegrate(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts);