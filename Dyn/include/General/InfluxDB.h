#pragma once
#include <iostream>
#include <windows.h>
#pragma comment(lib,"ws2_32.lib")
class CInfluxDB
{
public:
    CInfluxDB(const std::string& host, int port, const std::string& db);
    ~CInfluxDB();
    // ����measurement
    void setMeasurement(const std::string& measurement);
    // �鿴measurement
    void printMeasurement() const;
    // ���Ӽ�ֵ�� ==> str2
    void addKeyValue(const std::string& field, const double& value);
    // �鿴str2
    void printStr2() const;
    // UDP����InfluxDB
    void sendUdp();
    // �����ϴη������ݿ�ʱ�両����
    bool IsSend(double Period) const;
    // �����������str2
    void clearStr2();
private:
    SOCKET s;
    std::string str1; // �洢measurements
    std::string str2; // �洢��ֵ��key ==> value
    std::string sendBuffer; // ���ͻ�����
    // ����double ==> string
    std::string doubleToString(const double& value) const;
    // �������ݿ�
    sockaddr_in server;
    std::string serverAddress;
    int serverPort;
    std::string dbName;

    mutable int64_t LastRenewTime;
};

