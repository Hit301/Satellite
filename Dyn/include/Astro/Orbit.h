#pragma once
#include"SatelliteMath/BaseMath.h"

struct RV
{
    Eigen::Vector3d Pos;//轨道的位置矢量，单位m
    Eigen::Vector3d Vel;//轨道的速度矢量，单位m/s
    RV() : Pos(6678136.9999998566, 0.0002095685, -1.3800009224), 
            Vel(0.0007615644, 6789.5304738682, 3686.4138485846) {}
    RV(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel)
     : Pos(initialPos),Vel(initialVel){}
};

struct OrbitElement
{
    double a;               //轨道半长轴(Semi-major Axis)
    double e;               //轨道偏心率(Eccentricity)
    double i;               //轨道倾角（rad） (Inclination)
    double RAAN;            //升交点赤经（rad）(RAAN)
    double omega;           //近地点幅角（rad）(Arg of Perigee)
    double M;               //轨道平近点角（rad）(Mean Anomaly)
    double f;               //真近点角（rad）(True Anomaly)
    double u;               //轨道幅角（纬度幅角）（rad）(Arg of Latitude)
    double E;               //轨道偏近点角（rad）(Eccentric Anomaly)
    double w;               //轨道平均角速度（rad/s）(Palstance  2PI/Period)
    double T;               //轨道周期（s）(Period)
    OrbitElement():
        a(6678137), e(0), i(RAD(28.5)),
        RAAN(0), omega(0), M(0)
    {
        double Epre = M;
        double Enxt = 0.0;
        for (int i{0};i<5;i++)
        {
            Enxt = Epre - (Epre - e * sin(Epre) - M) / (1.0 - e * cos(Epre));
            if (fabs(Enxt - Epre) < 1e-7) { break; }
            Epre = Enxt;
        }
        E = RAD_2PI(Enxt);
        f = RAD_2PI(ATAN2(SQRT(1.0 - e * e) * sin(E), cos(E) - e));
        u = RAD_2PI(omega + f);
        w = sqrt(EARTH_GRAVITATIONAL / (pow(a, 3)));
        T = TWOPI / w;
    }
};

class COrbit
{
private:
    bool IsRV(RV& rv)
    {
        return (rv.Pos.norm() > EARTH_EQUATORIAL_RADIUS && rv.Vel.norm() > 0 && rv.Vel.norm() < 7900);
    }

    bool IsOrbitElement(OrbitElement& oe)
    {
        return (oe.a > EARTH_EQUATORIAL_RADIUS && oe.e >= 0 && oe.e < 1.0);
    }

public:
    RV J2000Inertial;//惯性系RV
    OrbitElement OrbitElements;//轨道根数

    COrbit(): J2000Inertial(), OrbitElements()
    {}

    //
    // brief  : 使用惯性系RV和二体递推轨道
    //
    int TwoBod(double Ts);

    //
    //brief:轨道系和惯性系的转移矩阵
    //


};


std::ostream& operator<<(std::ostream& _cout, const RV& j2000);