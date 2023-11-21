#pragma once
#include<iostream>
#include <errno.h>
#include <stdint.h>
#include<cmath>
#include <Eigen/Dense>



//定义数学常量
#define M_PI   3.14159265358979323846
#define ZERO        (1e-12)
#define MATH_E      (2.71828182845904523536)               //e
#define HALFPI      (1.57079632679489661923)               //pi/2  
#define TWOPI       (6.28318530717958647692)               //2*pi 
#define DEG2RAD     (0.01745329251994329576)               //pi/180 
#define RAD2DEG     (57.2957795130823208767)               //180/pi
#define SEC2DEG     (2.777777777777777778e-4)              //1/3600
#define SEC2RAD     (4.848136811095359936e-6)              //1/3600*DEG2RAD 
#define SQRT2       (1.41421356237309504880)               //sqrt(2)
#define SQRT3       (1.73205080756887729352)               //sqrt(3)
#define SQRT5       (2.23606797749978969640)               //sqrt(5)
#define LIGHT_VELOCITY                      (299792458.458)                   //(m/s)
#define EARTH_RADIUS_M						(6371230.000000000)               //(m)                             
#define EARTH_ECCENTRICITY                  (0.081819190842621)               //                                
#define EARTH_RATE                          (7.292115859146235E-05)           //15.04106717866910/3600* pi/180                     
#define EARTH_EQUATORIAL_RADIUS             (6378137.0)                       //(m)                            
#define EARTH_POLAR_RADIUS                  (6356752.31424)                   //(m)                                    
#define EARTH_GRAVITATIONAL                 (398600441500000.0)               //miu
#define SUN_RADIUS_M                        (695508000.0)                     //
#define MOON_RADIUS_M                       (1737400.0)                       //
#define J2                                  (0.00108263)                      //
#define J3                                  (-2.5356e-6)
#define J4                                  (-0.00000162336)                  //

//
// brief  : 磁感应强度高斯转特斯拉
//
#define GAUSS2T(Gauss) ((Gauss)*0.0001)

//
// brief  : 磁感应强度特斯拉转高斯
//
#define T2GAUSS(t) ((t)*10000.0)

//
// brief  : 弧度转角度
//
#define DEG(rad) ((rad)*RAD2DEG)

//
// brief  : 角度转弧度
//
#define RAD(deg) ((deg)*DEG2RAD) 

//
// brief  : 计算角度的sin值
//
#define SIND(deg) (sin(RAD(deg)))

//
// brief  : 计算角度的cos值
//
#define COSD(deg) (cos(RAD(deg)))

//
// brief  : 转速rmp转角速度
//
#define RPM2VEL(rpm) ((rpm)*(0.10471975511965977461))             // 2PI/60

//
// brief  : 角速度转rmp
//
#define VEL2RPM(vel) ((vel)*(9.54929658551372014613))             // 60/2PI


// brief  : MJD转JD
//
#define MJD2JD(mjd)                         ((mjd)-2400000.5)                //MJD -> JD

//
// brief  : JD转MJD
//
#define JD2MJD(jd)                          ((jd)+2400000.5)                 //JD -> MJD

//
// brief  : JD计算1970年起的总天数
//
#define JD2DAYS(jd)                         ((jd)-2440587.5)                 //Total days from 1970-1-1

//
// brief  : MJD计算1970年起的总天数
//
#define MJD2DAYS(mjd)                       ((mjd)-40587.0)                  //Total days from 1970-1-1

//
// brief  : 时间戳转JD
//
#define TS2JD(timestamp)                    (((timestamp)-1609459200.0)/86400.0+2459215.5)     //time stamp to JD

//
// brief  : 时间戳转儒略世纪数
//
#define TS2CEN(timestamp)                   ((TS2JD(timestamp)-2451545.0)/36525.0)            //time stamp to JD centry

//
// brief  : JD转时间戳
//
#define JD2TS(jd)                          (((jd)-2459215.5)*86400.0+1609459200.0)            //JD to time stamp



//
// brief  : 取符号
//
#define SIGN(v) (v<0?(-1):(1))

//
// brief  : 比较两个浮点数是否相等（差值极小）
//
#define EQUALS(a,b) (fabs((D64_t)((a)-(b)))<ZERO)

//
// brief  : 比较a，b的差小于v
//
#define EQUALSVAL(a,b,v) (fabs((D64_t)((a)-(b)))<fabs((D64_t)(v)))

//
// brief  : 判断数值是否为0
//
#define IS_ZERO(v) (fabs((D64_t)(v))<ZERO)

//
// brief  : 判断双精度数据是否合法
//
#define IS_DOUBLE(v) (FP_NORMAL == fpclassify(v) || FP_ZERO == fpclassify(v))

//
// brief  : 判断浮点数是否合法
//
#define IS_FLOAT(v) IS_DOUBLE(v)

//
// brief  : 取两者较大者
//
#define MAX(v1,v2) ((v1)>(v2)?(v1):(v2))

//
// brief  : 取两者的较小值
//
#define MIN(v1,v2) ((v1)<(v2)?(v1):(v2))

//
// brief  : 数值的峰值限幅
//
#define LIMIT(v,min,max) MAX(MIN(v,max),min)

//
// brief  : 判断是否为NULL
//
#define IS_NULL(v) (NULL==(v))

//
// brief  : 判断是否为非NULL
//
#define NOT_NULL(v) (NULL!=(v))


//
// brief  : 弧度调整到0 ~ 2pi
//
double RAD_2PI(double value);

//
// brief  : 弧度转成-pi ~ pi
//
double RAD_PI(double value);

//
// brief  : 整型除0保护
//
int PROTECT_INT(int value);

//
// brief  : 整型除0保护
//
unsigned PROTECT_UINT(unsigned value);

//
// brief  : 浮点型除0保护
//
double PROTECT(double value);

//
// brief  : acos运算，对输入值限幅[-1,1]
//
double ACOS(double value);

//
// brief  : asin运算，对输入值限幅[-1,1]
//
double ASIN(double value);

//
// brief  : sqrt运算,限制定义域
//
double SQRT(double value);

//
// brief  : 幂运算
//
double POW(double X, double Y);

//
// brief  : X对Y取模（浮点数）
//
double MOD(double X, double Y);

//
// brief  : X对Y取模（Int32）
//
int MOD_INT(int X, int Y);

//
// brief  : X对Y取模（Uint32）
//
unsigned MOD_UINT(unsigned X, unsigned Y);

//
// brief  : 求反正切值
//
double ATAN2(double Y, double X);

//
// brief  : 求自然对数
//
double LOG(double Value);

//
// brief  : 求以2为底的对数
//
double LOG2(double Value);

//
// brief  : 求以10为底的对数
//
double LOG10(double Value);