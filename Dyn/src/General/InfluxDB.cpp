#include"General/InfluxDB.h"
#include"General/SimTime.h"
#include <sstream>
CInfluxDB::CInfluxDB(const std::string& host, int port, const std::string& db)
    : serverAddress(host), serverPort(port), dbName(db) {
    WSADATA wsaData;
    // ��ʼ��Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Failed. Error Code : " << WSAGetLastError();
    }
    // ���� socket
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
        std::cerr << "socket() failed with error code : " << WSAGetLastError();
    }
    // ���÷�������ַ
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr("127.0.0.1"); // ���� InfluxDB ��ַ
    server.sin_port = htons(port);
    LastRenewTime = GetTimeStampMs();
}

CInfluxDB::~CInfluxDB()
{   // ����������������
    closesocket(s);
    WSACleanup();
}

void CInfluxDB::setMeasurement(const std::string& measurement)
{
    str1 = measurement;
    str2 = str1 + ",tag=success SIM000=0.0";
    // InfluxDB-UDP-API�������ݸ�ʽ��ʱ����ɼӿɲ���
    // ��ǩֵ��������
    // "measurement,tag1=value1,tag2=value2 field1=value1,field2=value2 timestamp"
}

void CInfluxDB::printMeasurement() const {
    if (!str1.empty()) {
        std::cout << "measurement: " << str1 << std::endl;
    }
    else {
        std::cout << "measurement is empty." << std::endl;
    }
}


void CInfluxDB::addKeyValue(const std::string& field, const double& value)
{
    str2 += "," + field + "=" + std::to_string(value);
}

void CInfluxDB::printStr2() const
{
    std::cout << "str2: " << str2 << std::endl;
}

void CInfluxDB::sendUdp()
{
    if (sendto(s, str2.c_str(), (int)str2.length(), 0, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
        std::cerr << "sendto() failed with error code : " << WSAGetLastError();
    }
    else {
        std::cout << "Message sent successfully." << std::endl;
    }
    // ���·���ʱ��
}

void CInfluxDB::clearStr2() {
    str2 = str1 + ",";
}

std::string CInfluxDB::doubleToString(const double& value) const
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

bool CInfluxDB::IsSend(double Period) const
{
    if (GetTimeStampMs() - LastRenewTime > Period * 1e3)
    {
        LastRenewTime = GetTimeStampMs();
        return true;
    }
    return false;
}