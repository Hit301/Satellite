#pragma once
#include <iostream>
#include <windows.h>
#pragma comment(lib,"ws2_32.lib")
class CInfluxDB
{
public:
    CInfluxDB(const std::string& host, int port, const std::string& db);
    ~CInfluxDB();
    // 设置measurement
    void setMeasurement(const std::string& measurement);
    // 查看measurement
    void printMeasurement() const;
    // 增加键值对 ==> str2
    void addKeyValue(const std::string& field, const double& value);
    // 查看str2
    void printStr2() const;
    // UDP发送InfluxDB
    void sendUdp();
    // 返回上次发送数据库时间浮点数
    bool IsSend(double Period) const;
    // 发送完成清理str2
    void clearStr2();
private:
    SOCKET s;
    std::string str1; // 存储measurements
    std::string str2; // 存储键值对key ==> value
    std::string sendBuffer; // 发送缓冲区
    // 函数double ==> string
    std::string doubleToString(const double& value) const;
    // 配置数据库
    sockaddr_in server;
    std::string serverAddress;
    int serverPort;
    std::string dbName;

    mutable int64_t LastRenewTime;
};

