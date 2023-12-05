#pragma once
#include"SatelliteMath/BaseMath.h"
#include "Astro/Environment.h"

struct RV
{
    Eigen::Vector3d Pos;//�����λ��ʸ������λm
    Eigen::Vector3d Vel;//������ٶ�ʸ������λm/s
    RV() : Pos(6678136.9999998566, 0.0002095685, -1.3800009224), 
            Vel(0.0007615644, 6789.5304738682, 3686.4138485846) {}
    RV(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel)
     : Pos(initialPos),Vel(initialVel){}
};

struct OrbitElement
{
    double a;               //����볤��(Semi-major Axis)
    double e;               //���ƫ����(Eccentricity)
    double i;               //�����ǣ�rad�� (Inclination)
    double RAAN;            //������ྭ��rad��(RAAN)
    double omega;           //���ص���ǣ�rad��(Arg of Perigee)
    double M;               //���ƽ����ǣ�rad��(Mean Anomaly)
    double f;               //�����ǣ�rad��(True Anomaly)
    double u;               //������ǣ�γ�ȷ��ǣ���rad��(Arg of Latitude)
    double E;               //���ƫ����ǣ�rad��(Eccentric Anomaly)
    double w;               //���ƽ�����ٶȣ�rad/s��(Palstance  2PI/Period)
    double T;               //������ڣ�s��(Period)
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
    RV J2000Inertial;//����ϵRV
    RV Wgs84Fix;//�ع�ϵRV
    OrbitElement OrbitElements;//�������
    Environment Env;//����

    struct 
    {
        //
        // brief  : ������
        //
        double Lng;
        //
        // brief  : ����γ��
        //
        double Lat;
        //
        // brief  : ���θ߶�
        //
        double Alt;
    } LLA;//����γ��

    Eigen::Vector3d LLR;//����γ�ȺͰ뾶


    COrbit(): J2000Inertial(), OrbitElements(), Wgs84Fix()
    {
        LLA.Lng = 0;
        LLA.Lat = 0;
        LLA.Alt = 0;//LLA��LLR������
    }

    //
    // brief  : ʹ�ù���ϵRV�Ͷ�����ƹ��
    //
    int TwoBod(double Ts);

    //@brief: ����ϵλ���ٶ�ת�ع�ϵλ���ٶ�
    //@para : ����ϵRV
    //@return : none
    //@remark : ����ϵ���ع�ϵת�ƾ���ĵ���
    void Inl2Fix(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);

    //@brief: �ع�ϵ�������LLA
    //@para : none
    //@return : none
    void FixPosToLLA();

    //@brief: ���㱱����ϵ���ع�ϵ��ת�ƾ���
    //@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
    //@return : none
    //@remark : static
    static Eigen::Matrix3d NED2ECEF();
};


std::ostream& operator<<(std::ostream& _cout, const RV& j2000);