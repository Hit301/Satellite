#pragma once
#include <string>
#include <windows.h>
#include <winsock.h>

#include "Utils/utils.h"

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

typedef struct {
	float faultTimeLow;
	float faultAttLow;
	float faultTimeUp;
	float faultAttUp;
	int faultType;
	
	int gyroGroup;
	int gyroID;

	int runMode;
} faultParaStruct;

typedef struct {
	bool gyroIsChecked;
	bool starIsChecked;
	bool sunIsChecked;
	bool rwIsChecked;

	char path[NAMA_LEN_MAX];
} saveDataStruct;

class CUDP
{
public:
	CUDP(const char* cpPara="127.0.0.1", u_short portPara=5010); // 初始化函数

	void InitSocket();

	int SendUDPData(); // 向串口发送数据

	void SetServerSocket(SOCKET soc);

	UDPSendDataStruct* GetUDPData();
	faultParaStruct* GetfaultPara();
	SOCKET* GetSocket();
	SOCKADDR_IN* GetServerAddr();
	SOCKADDR_IN* GetRecvAddr();
	// char* GetRecvBuff();
	int* GetRecvCount();
	bool* GetInjectFlag();
	float* GetRunTimeTotal();
	bool* GetRunFlag();
	bool* GetSaveDataFlag();
	saveDataStruct* GetSaveData();

private:
	const char* cp;
	u_short port;

	UDPSendDataStruct UDPData;
	SOCKET ServerSocket;
	SOCKADDR_IN ServerAddr;

	SOCKADDR_IN RecvAddr;
	// char RecvBuff[0X3FFF];

	faultParaStruct faultPara;
	saveDataStruct saveData;

	int RecvCount;
	float runTimeTotal;

	bool injectFlag;
	bool RunFlag;
	bool SaveDataFlag;

};

DWORD WINAPI UDPServer(LPVOID lpParameter);

