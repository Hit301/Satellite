#include"SatelliteMath/EulerAgl.h"
#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/Quaternions.h"

CEulerAgl::CEulerAgl()
{

}

CEulerAgl::CEulerAgl(double R1, double R2, double R3, unsigned Seq)
{
}

CEulerAgl::CEulerAgl(CEulerAgl& Agl)
{
}

CEulerAgl& CEulerAgl::operator=(CEulerAgl Agl)
{
	return *this;
}

CDcm CEulerAgl::ToDcm()
{
	return CDcm();
}

Quat CEulerAgl::ToQuat()
{
	return Quat();
}
