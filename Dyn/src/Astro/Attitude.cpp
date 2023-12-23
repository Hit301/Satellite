#include"Astro/Attitude.h"
#include "Astro/Orbit.h"
#include"General/CConfig.h"
#include"General/InfluxDB.h"

//姿态运动学计算差分四元数
#if 0
Eigen::Vector3d Omega_bRK4(Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Omega_b, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s, double Ts)
{
    Eigen::Vector3d k1, k2, k3, k4;
    k1 = AttDynamics(Omega_b, SatInaMat, Hw, Tau_s);
    k2 = AttDynamics(Omega_b + k1 * (0.5 * Ts), SatInaMat, Hw, Tau_s);
    k3 = AttDynamics(Omega_b + k2 * (0.5 * Ts), SatInaMat, Hw, Tau_s);
    k4 = AttDynamics(Omega_b + k3 * Ts, SatInaMat, Hw, Tau_s);

    return Omega_b + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
}

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

Quat QibIntegrate(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts)
{
    Quat QuatTemp;
    QuatTemp = PlstToDeltaQuat(Omega_b, Ts);
    return Quat_k * QuatTemp;
}
#endif

//姿态动力学计算角速度变化率
Eigen::Vector3d AttDynamics(Eigen::Vector3d Omega_b, Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s)
{
    Eigen::Vector3d tmp = Omega_b.cross(SatInaMat * Omega_b + Hw);
    Eigen::Vector3d result = SatInaMat.inverse() * (Tau_s - tmp);
    return result;
}


//角速度和时长计算误差四元数
Quat PlstToDeltaQuat(const Eigen::Vector3d Omega_b, double OfstSec)
{
    double PlstVal = Omega_b.norm();
    return Quat(PlstVal * OfstSec, Omega_b);
}



CAttitude::CAttitude() :Qib(),Qob(),Aio()
{
    Omega_b << 0.05, -0.04, 0.1;

    SatInaMat << 10, 0, 0,
        0, 20, 0,
        0, 0, 30;
    WheelMomentum_b << 0, 0, 0;
    TotalTorque << 0, 0, 0;
}

int CAttitude::AttitudeDynamicsRk4(double Ts)
{
    Eigen::Vector3d k1, k2, k3, k4;
    k1 = AttDynamics(Omega_b, SatInaMat, WheelMomentum_b, TotalTorque);
    k2 = AttDynamics(Omega_b + k1 * (0.5 * Ts), SatInaMat, WheelMomentum_b, TotalTorque);
    k3 = AttDynamics(Omega_b + k2 * (0.5 * Ts), SatInaMat, WheelMomentum_b, TotalTorque);
    k4 = AttDynamics(Omega_b + k3 * Ts, SatInaMat, WheelMomentum_b, TotalTorque);
    Omega_b += (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
    return 0;
}

int CAttitude::AttitudeKinematics(double Ts)
{
    Quat QuatTemp;
    QuatTemp = PlstToDeltaQuat(Omega_b, Ts);
    Qib = Qib * QuatTemp;
    return 0;
}

void CAttitude::GetAio(COrbit& Orbit)
{
    Eigen::Vector3d Pos = Orbit.J2000Inertial.Pos;//卫星的位置矢量
    Eigen::Vector3d Vel = Orbit.J2000Inertial.Vel;//卫星的速度矢量
    Eigen::Vector3d zo = Eigen::Vector3d::Zero() - Pos / Pos.norm();//偏航轴单位矢量
    Eigen::Vector3d y_tmp = Vel.cross(Pos);
    Eigen::Vector3d yo = y_tmp / y_tmp.norm(); // 俯仰轴单位矢量
    Eigen::Vector3d xo = yo.cross(zo);//滚动轴单位矢量

    Aio.DcmData << xo, yo, zo;
}

void CAttitude::StateRenew(double Ts, COrbit& Orbit)
{
    AttitudeKinematics(Ts);
    AttitudeDynamicsRk4(Ts);
    GetAio(Orbit);

    Quat Qio = Aio.ToQuat();
    Qob = Qio.QuatInv() * Qib;
}

void CAttitude::Init(COrbit& Obt)
{
    CConfig* pCfg = CConfig::GetInstance();
   Omega_b << pCfg->Wx, pCfg->Wy, pCfg->Wz;
   Qib.QuatData[0] = pCfg->Q0;
   Qib.QuatData[1] = pCfg->Q1;
   Qib.QuatData[2] = pCfg->Q2;
   Qib.QuatData[3] = pCfg->Q3;
   SatInaMat << pCfg->Jxx, pCfg->Jxy, pCfg->Jxz,
        pCfg->Jxy, pCfg->Jyy, pCfg->Jyz,
        pCfg->Jxz, pCfg->Jyz, pCfg->Jzz;
    GetAio(Obt);
    Qob = Aio.ToQuat().QuatInv() * Qib;
}

void CAttitude::record(CInfluxDB& DB) {
    DB.addKeyValue("SIM001", 6.6);
    //SIM001~SIM004 SIM005~SIM009预留 
    // 物理意义 编号 单位
    //SELETE SIM001 FROM Satellite_db
}
