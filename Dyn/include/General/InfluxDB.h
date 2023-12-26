#pragma once
#include<string>
#include <windows.h>
#pragma comment(lib,"ws2_32.lib")
class CInfluxDB
{
public:
    CInfluxDB(const std::string& host, int port, const std::string& dbname, const std::string& meas);
    ~CInfluxDB();
    // ����measurement
    void setMeasurement(const std::string& measurement);
    // �鿴measurement
    void printMeasurement() const;
    // ���Ӽ�ֵ�� ==> str2
    template<typename type1>
    void addKeyValue(const std::string& field, const type1& value);
    // �鿴str2
    void printStr2() const;
    // UDP����InfluxDB
    void sendUdp();
    // �����ϴη������ݿ�ʱ�両����
    bool IsSend(double Period) const;
    // �����������str2
    void ResetStr2();
private:
    SOCKET s;
    std::string str1; // �洢measurements
    std::string str2; // �洢��ֵ��key ==> value
    std::string sendBuffer; // ���ͻ�����
    // �������ݿ�
    sockaddr_in serverAddr;
    std::string dbName;

    mutable int64_t LastRenewTime;
};

template<typename type1>
inline void CInfluxDB::addKeyValue(const std::string& field, const type1& value)
{
    str2 += "," + field + "=" + std::to_string(value);
}

