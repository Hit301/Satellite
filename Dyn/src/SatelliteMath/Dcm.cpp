#include"SatelliteMath/Dcm.h"
#include"SatelliteMath/EulerAgl.h"
#include"SatelliteMath/Quaternions.h"

CDcm::CDcm()
{
}

CDcm::CDcm(double A00, double A01, double A02, double A10, double A11, double A12, double A20, double A21, double A22)
{
}

CDcm::CDcm(unsigned Axis, double Theta)
{
}

//CDcm::CDcm(CEulerAgl Agl)
//{
//}
//
//CDcm::CDcm(Quat quat)
//{
//}

CDcm::CDcm(CDcm& _Dcm)
{
}

CDcm& CDcm::operator=(CDcm _Dcm)
{
	return *this;
}

CEulerAgl CDcm::ToEulerAgl(unsigned Sequence)
{
	return CEulerAgl();
}

Quat CDcm::ToQuat()
{
	return Quat();
}
