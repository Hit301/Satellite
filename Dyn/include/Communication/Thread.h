#pragma once
#include "Communication/USART.h"
#include "Communication/UDP.h"

#pragma comment(lib,"ws2_32.lib")


class CCommuThread
{
public:
	CCommuThread(const char* comPara = "COM1", const char* cpPara = "127.0.0.1", u_short portPara = 5010, bool runFlag=false); // 初始化函数
	~CCommuThread();

	void ThreadRun(); // 串口和UDP线程运行

	CUSART* GetUSART();
	HANDLE GetMutex();
	CUDP* GetUDP();

	void UpdateUDPData(double sampleTime, double* omega, double* qib);
	void UpdateUSARTData(double* omega, double* qib);

	void SendUsartData();
	void SendUdpData();

	void IterAdd(int num);
	int GetIter();

private:
	CUDP* udp;
	CUSART* usart;
	int iter;
	bool RunFlag;
};
