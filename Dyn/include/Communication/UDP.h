#pragma once
#include <windows.h>
#include <winsock.h>
#pragma comment(lib,"ws2_32.lib")

typedef struct
{
	double timeStep;

	double Omega_b[3];

	double Qib[4];

	double R[3];

	double V[3];

	double Qbo[4];

	double T[3];

	double SunPointAngle;
}UDPSendDataStruct;

class CUDP
{
public:
	CUDP(const char* cpPara="127.0.0.1", u_short portPara=5010); // 初始化函数

	void InitSocket();

	// DWORD WINAPI UDPServer(LPVOID lpParameter); // UDP通信
	int SendUDPData(); // 向串口发送数据

	void SetServerSocket(SOCKET soc);

	UDPSendDataStruct* GetUDPData();
	SOCKET* GetSocket();
	SOCKADDR_IN* GetServerAddr();
	SOCKADDR_IN* GetRecvAddr();
	char* GetRecvBuff();
	int* GetRecvCount();


private:
	const char* cp;
	u_short port;

	UDPSendDataStruct UDPData;
	SOCKET ServerSocket;
	SOCKADDR_IN ServerAddr;

	SOCKADDR_IN RecvAddr;
	char RecvBuff[0X3FFF];

	int RecvCount;

};

DWORD WINAPI UDPServer(LPVOID lpParameter);

