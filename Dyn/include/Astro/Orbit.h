#pragma once
#include "SatelliteMath/SatelliteEigen.h"

class Inl_t
{
public:
    Eigen::Vector3d Pos;
    Eigen::Vector3d Vel;
    Inl_t(): Pos(0.0, 0.0, 0.0), Vel(0.0, 0.0, 0.0)   
    {}
    Inl_t(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel)
        : Pos(initialPos),
        Vel(initialVel)
    {
    }
};

Inl_t TwoBod(Inl_t& J2000, double Ts);
std::ostream& operator<<(std::ostream& _cout, const Inl_t& j2000);