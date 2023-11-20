#include"SatelliteMath/BaseMath.h"

double RAD_2PI(double value)
{
    if (value > TWOPI) { return fmod(value, TWOPI); }
    if (value < 0) { return fmod(value, TWOPI) + TWOPI; }
    return value;
}

double RAD_PI(double value)
{
    double v = RAD_2PI(value);
    if (v > M_PI) return v - TWOPI;
    return v;
}

int PROTECT_INT(int value)
{
    if (value == 0)
    {
        printf("PROTECT_INT error Value=%d\n", value);
        return 1;
    }
    return value;
}

unsigned PROTECT_UINT(unsigned value)
{
    if (value == 0)
    {
        printf("PROTECT_UINT error Value=%u\n", value);
        return 1;
    }
    return value;
}

double PROTECT(double value)
{
    if (value >= 0 && value < ZERO)
    {
        printf("PROTECT error Value=%E\n", value);
        return ZERO;
    }

    if (value < 0 && value > -ZERO)
    {
        printf("PROTECT error Value=%E\n", value);
        return -ZERO;
    }

    return value;
}

double ACOS(double value)
{
    if (value >= -1.0 && value <= 1.0) return acos(value);
    if (value < -1.0) return M_PI;

    return 0;
}

double ASIN(double value)
{
    if (value >= -1.0 && value <= 1.0) return asin(value);

    if (value < -1.0) return -HALFPI;
    return HALFPI;
}

//
// brief  : 开方运算
//
double SQRT(double value)
{
    if (value >= 0) return sqrt(value);
    return 0;
}

//
// brief  : 幂运算
//
double POW(double X, double Y)
{
    errno = 0;
    double Ret = pow(X, Y);
    if (errno != 0)
    {
        printf("POW error X=%E , Y=%E\n", X, Y);

        Ret = 1;
    }
    return Ret;
}

double MOD(double X, double Y)
{
    if (fpclassify(Y) == FP_ZERO)
    {
        printf("MOD error X=%E , Y=%E\n", X, Y);
        return 0;
    }
    return fmod(X, Y);
}

int MOD_INT(int X, int Y)
{
    if (Y == 0)
    {
        printf("MOD_INT error X=%d , Y=%d\n", X, Y);
        return 0;
    }

    return X % Y;
}

unsigned MOD_UINT(unsigned X, unsigned Y)
{
    if (Y == 0)
    {
        printf("MOD_UINT error X=%u , Y=%u\n", X, Y);
        return 0;
    }

    return X % Y;
}


//
// brief  : 求反正切值,返回弧度
//
double ATAN2(double Y, double X)
{
    errno = 0;
    double Ret = atan2(Y, X);
    if (errno != 0)
    {
        printf("ATAN2 error Y=%E , X=%E\n", Y, X);

        Ret = 0;
    }
    return Ret;
}


//
// brief  : 求自然对数
//
double LOG(double Value)
{
    if (Value > 0)
    {
        return log(Value);
    }

    printf("LOG error Value=%E\n", Value);

    return 0;
}

//
// brief  : 求以2为底的对数
//
double LOG2(double Value)
{
    if (Value > 0)
    {
        return log2(Value);
    }

    printf("LOG2 error Value=%E\n", Value);

    return 0;
}


//
// brief  : 求以10为底的对数
//
double LOG10(double Value)
{
    if (Value > 0)
    {
        return log10(Value);
    }

    printf("LOG10 error Value=%E\n", Value);

    return 0;
}
