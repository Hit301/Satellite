/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:53:10
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-02-26 09:24:22
 * @FilePath: /Satellite/include/General/InfluxDB.h
 * @Description: 
 */
#pragma once
#include <string>
#include <ctime> // 用于获取时间
#include <cstring> // 用于memset等函数

// 根据不同的操作系统包含不同的头文件
#ifdef _WIN32
    #include<Windows.h>
    #pragma comment(lib,"ws2_32.lib") // Windows需要的库
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h> // 提供close函数
    typedef int SOCKET; // 在非Windows系统中定义SOCKET为int
    #define closesocket close
    #define INVALID_SOCKET (-1)
    #define SOCKET_ERROR   (-1)
#endif
class CInfluxDB
{
public:
    CInfluxDB();
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
    // 配置数据库
    sockaddr_in serverAddr;
    mutable int64_t LastRenewTime;
};

template<typename type1>
inline void CInfluxDB::addKeyValue(const std::string& field, const type1& value)
{
    str2 += "," + field + "=" + std::to_string(value);
}

