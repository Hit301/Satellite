#include "Communication/Thread.h"
#include "Communication/USART.h"
#include "Communication/UDP.h"
#include "Communication/BaseCommuPara.h"

HANDLE Mutex = NULL;

CCommuThread::CCommuThread(const char* comPara, const char* cpPara, u_short portPara, bool UDPStartFlag, bool USARTStratFlag)
{
	// Mutex为全局变量 (非类的成员变量)
	Mutex = CreateMutex(NULL, FALSE, NULL); // Mutex初始化
	if (UDPStartFlag)
		this->udp = new CUDP(cpPara, portPara);
	else
		this->udp = NULL;

	if (USARTStratFlag)
		this->usart = new CUSART(comPara);
	else
		this->usart = NULL;
	

	this->iter = 0;

	this->UDPStartFlag = UDPStartFlag;
	this->USARTStratFlag = USARTStratFlag;
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
	if (*this->GetUDP()->GetRunFlag() == false)
		return;
	UDPSendDataStruct* UDPData = this->GetUDP()->GetUDPData();
	UDPData->timeStep = this->GetIter() * sampleTime;
	memcpy(&UDPData->Omega_b, omega, sizeof(UDPData->Omega_b));
	memcpy(&UDPData->Qib, qib, sizeof(UDPData->Qib));
}

void CCommuThread::UpdateUSARTData(double* omega, double* qib)
{
	if (*this->GetUDP()->GetRunFlag() == false)
		return;
	USARTDataStruct* USARTData = this->GetUSART()->GetUSARTData();
	memcpy(&USARTData->Omega_b, omega, sizeof(USARTData->Omega_b));
	memcpy(&USARTData->Qib, qib, sizeof(USARTData->Qib));
}

void CCommuThread::SendUsartData()
{
	if (*this->GetUDP()->GetRunFlag() == false)
		return;
	this->GetUSART()->SendUsartData();
}

void CCommuThread::SendUdpData()
{
	if (*this->GetUDP()->GetRunFlag() == false)
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

bool CCommuThread::GetUDPStartFlag()
{
	return this->UDPStartFlag;
}

bool CCommuThread::GetUSARTStartFlag()
{
	return this->USARTStratFlag;
}

void CCommuThread::ThreadRun(bool udpStart, bool usartStart)
{
	if (udpStart)
	{
		HANDLE thread1 = CreateThread(NULL, 0, UDPServer, this->udp, 0, NULL);
		CloseHandle(thread1);
	}

	if (usartStart)
	{
		HANDLE thread2 = CreateThread(NULL, 0, USARTServer, this->usart, 0, NULL);
		CloseHandle(thread2);
	}
}
