#include "Communication/Thread.h"
#include "Communication/USART.h"
#include "Communication/UDP.h"
#include "Communication/BaseCommuPara.h"

HANDLE Mutex = NULL;

CCommuThread::CCommuThread(const char* comPara, const char* cpPara, u_short portPara, bool runFlag)
{
	// Mutex为全局变量 (非类的成员变量)
	Mutex = CreateMutex(NULL, FALSE, NULL); // Mutex初始化
	this->usart = new CUSART(comPara);
	this->udp = new CUDP(cpPara, portPara);

	this->iter = 0;
	this->RunFlag = runFlag;
}

CCommuThread::~CCommuThread()
{
	delete this->udp;
	delete this->usart;
}

CUSART* CCommuThread::GetUSART()
{
	return this->usart;
}

HANDLE CCommuThread::GetMutex()
{
	return Mutex;
}

CUDP* CCommuThread::GetUDP()
{
	return this->udp;
}

void CCommuThread::UpdateUDPData(double sampleTime, double* omega, double* qib)
{
	if (this->RunFlag == false)
		return;
	UDPSendDataStruct* UDPData = this->GetUDP()->GetUDPData();
	UDPData->timeStep = this->GetIter() * sampleTime;
	memcpy(&UDPData->Omega_b, omega, sizeof(UDPData->Omega_b));
	memcpy(&UDPData->Qib, qib, sizeof(UDPData->Qib));
}

void CCommuThread::UpdateUSARTData(double* omega, double* qib)
{
	if (this->RunFlag == false)
		return;
	USARTDataStruct* USARTData = this->GetUSART()->GetUSARTData();
	memcpy(&USARTData->Omega_b, omega, sizeof(USARTData->Omega_b));
	memcpy(&USARTData->Qib, qib, sizeof(USARTData->Qib));
}

void CCommuThread::SendUsartData()
{
	if (this->RunFlag == false)
		return;
	this->GetUSART()->SendUsartData();
}

void CCommuThread::SendUdpData()
{
	if (this->RunFlag == false)
		return;
	this->GetUDP()->SendUDPData();
}

void CCommuThread::IterAdd(int num)
{
	this->iter += num;
}

int CCommuThread::GetIter()
{
	return this->iter;
}

void CCommuThread::ThreadRun()
{
	if (RunFlag == false)
		return;
	HANDLE thread1 = CreateThread(NULL, 0, UDPServer, this->udp, 0, NULL);
	HANDLE thread2 = CreateThread(NULL, 0, USARTServer, this->usart, 0, NULL);
	CloseHandle(thread1);
	CloseHandle(thread2);
}
