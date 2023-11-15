#pragma once
#include "SatelliteMath/SatelliteEigen.h"
class Environment
{
public:
	Eigen::Vector3d EarthMag;//本体系地磁场强度
	Environment();
};