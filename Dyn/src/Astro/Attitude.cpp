#include"Astro/Attitude.h"

//��̬����ѧ������ٶȱ仯��
Eigen::Vector3d AttDynamics(Eigen::Vector3d Omega_b, Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s)
{
    Eigen::Vector3d tmp = Omega_b.cross(SatInaMat * Omega_b + Hw);
    Eigen::Vector3d result = SatInaMat.inverse() * (Tau_s - tmp);
    return result;
}

Eigen::Vector3d Omega_bRK4(Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Omega_b, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s, double Ts)
{
    Eigen::Vector3d k1, k2, k3, k4;
    k1 = AttDynamics(Omega_b, SatInaMat, Hw, Tau_s);
    k2 = AttDynamics(Omega_b + k1 * (0.5 * Ts), SatInaMat, Hw, Tau_s);
    k3 = AttDynamics(Omega_b + k2 * (0.5 * Ts), SatInaMat, Hw, Tau_s);
    k4 = AttDynamics(Omega_b + k3 * Ts, SatInaMat, Hw, Tau_s);

    return Omega_b + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
}


//��̬�˶�ѧ��������Ԫ��
#if 0
Quat AttKinematics(Quat Quat_k, Eigen::Vector3d Omega_b)
{
    double w, x, y, z;
    w = (-Omega_b(0,0) *Quat_k.GetByIdx(1) - Omega_b(1,0) * Quat_k.GetByIdx(2) - Omega_b(2,0) * Quat_k.GetByIdx(3)) * 0.5;
    x = (Omega_b(0,0) *Quat_k.GetByIdx(0) - Omega_b(1,0) * Quat_k.GetByIdx(3) + Omega_b(2,0) * Quat_k.GetByIdx(2)) * 0.5;
    y = (Omega_b(1,0) *Quat_k.GetByIdx(0) - Omega_b(2,0) *Quat_k.GetByIdx(1) + Omega_b(0,0) * Quat_k.GetByIdx(3)) * 0.5;
    z = (Omega_b(2,0) *Quat_k.GetByIdx(0) + Omega_b(1,0) *Quat_k.GetByIdx(1) - Omega_b(0,0) * Quat_k.GetByIdx(2)) * 0.5;
    return Quat(w, x, y, z);
}


Quat QibRK4(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts)
{
    Quat k1, k2, k3, k4,Res;
    k1 = AttKinematics(Quat_k, Omega_b);
    k2 = AttKinematics(Quat_k + k1 * (0.5 * Ts), Omega_b);
    k3 = AttKinematics(Quat_k + k2 * (0.5 * Ts), Omega_b);
    k4 = AttKinematics(Quat_k + k3 * Ts, Omega_b);
    Res = Quat_k + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
    return Res.QuatNormalize();
}
#endif

//���ٶȺ�ʱ�����������Ԫ��
Quat PlstToDeltaQuat(const Eigen::Vector3d Omega_b, double OfstSec)
{
    double PlstVal = Omega_b.norm();
    return Quat(PlstVal * OfstSec, Omega_b);
}

Quat QibIntegrate(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts)
{
    Quat QuatTemp;
    QuatTemp = PlstToDeltaQuat(Omega_b, Ts);
    return Quat_k * QuatTemp;
}
