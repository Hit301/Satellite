#pragma once
#include"SatelliteMath/BaseMath.h"
#include"General/AllHead.h"
// 2023-12-22 11:19:45
struct RV
{
    Eigen::Vector3d Pos;//轨道的位置矢量，单位m
    Eigen::Vector3d Vel;//轨道的速度矢量，单位m/s
    RV() : Pos(6678136.9999998566, 0.0002095685, -1.3800009224), 
            Vel(0.0007615644, 6789.5304738682, 3686.4138485846) {}
    RV(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel)
     : Pos(initialPos),Vel(initialVel){}
};
struct LLA_t
{
    double Lng;//地理经度，单位弧度
    double Lat;//地理纬度，单位弧度
    double Alt;//海拔高度，单位m
    LLA_t() : Lng(0), Lat(1.3963), Alt(0) {}
};
struct LLR_t
{
    double Lng;//球心经度，单位弧度
    double Lat;//球心纬度，单位弧度
    double Rds;//球心半径，单位m
    LLR_t() : Lng(0), Lat(1.3951), Rds(6357400) {}
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
    RV ECEFFix;//地固系RV
    OrbitElement OrbitElements;//轨道根数
    LLA_t LLA;//经纬高
    LLR_t LLR;//地球经纬度和半径


public:
    COrbit(): J2000Inertial(), OrbitElements(), ECEFFix(), LLA(), LLR()
    {
        //先写一个以惯性系初始化的方式吧

    }

    //
    // brief  : 使用惯性系RV和二体递推轨道
    //
    int TwoBod(double Ts);

    //@brief: 惯性系位置速度转地固系位置速度
    //@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
    //@return : none
    //@remark : 已测试
    void Inl2Fix(const int64_t timestamp);

    //@brief: 地固系轨道计算LLA
    //@para : none
    //@return : none
    //@remark : 已测试
    void FixPos2LLA();

    //@brief: 地固系轨道计算LLR
    //@para : none
    //@return : none
    //@remark : 已测试
    void FixPos2LLR();

    //@brief: 计算北东地系到地固系的转移矩阵
    //@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
    //@return : none
    //@remark : 静态成员函数只能访问静态成员变量
    Eigen::Matrix3d NED2ECEF();

    void StateRenew(double Ts, const int64_t timestamp);

    void Init(int64_t Timestamp);

    // 写入数据库
    void record(CInfluxDB& DB);
};


std::ostream& operator<<(std::ostream& _cout, const RV& j2000);