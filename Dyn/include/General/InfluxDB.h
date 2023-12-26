#pragma once
#include<string>
#include <windows.h>
#pragma comment(lib,"ws2_32.lib")
class CInfluxDB
{
public:
    CInfluxDB(const std::string& host, int port, const std::string& dbname, const std::string& meas);
    ~CInfluxDB();
    // 设置measurement
    void setMeasurement(const std::string& measurement);
    // 查看measurement
    void printMeasurement() const;
    // 增加键值对 ==> str2
    template<typename type1>
    void addKeyValue(const std::string& field, const type1& value);
    // 查看str2
    void printStr2() const;
    // UDP发送InfluxDB
    void sendUdp();
    // 返回上次发送数据库时间浮点数
    bool IsSend(double Period) const;
    // 发送完成清理str2
    void ResetStr2();
private:
    SOCKET s;
    std::string str1; // 存储measurements
    std::string str2; // 存储键值对key ==> value
    std::string sendBuffer; // 发送缓冲区
    // 配置数据库
    sockaddr_in serverAddr;
    std::string dbName;

    mutable int64_t LastRenewTime;
};

template<typename type1>
inline void CInfluxDB::addKeyValue(const std::string& field, const type1& value)
{
    str2 += "," + field + "=" + std::to_string(value);
}

