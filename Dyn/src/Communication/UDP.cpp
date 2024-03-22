#include "Communication/UDP.h"
#include "Communication/BaseCommuPara.h"

#include <iostream>

using namespace std;


CUDP::CUDP(const char* cpPara, u_short portPara)
{
	memset(&UDPData, 0, sizeof(UDPData));
	memset(&faultPara, 0, sizeof(faultPara));
	// memset(&RecvBuff, 0, sizeof(RecvBuff));
	memset(&ServerAddr, 0, sizeof(ServerAddr));
	memset(&RecvAddr, 0, sizeof(RecvAddr));
	ServerSocket = 0;
	RecvCount = 0;
	injectFlag = false;
	runTimeTotal = -1;
	RunFlag = false;

	cp = cpPara;
	port = portPara;

	// ���÷������Ϣ
	ServerAddr.sin_family = AF_INET;
	ServerAddr.sin_addr.S_un.S_addr = inet_addr(cp);
	ServerAddr.sin_port = htons(port);

	//// ��ʼ���׽���
	//this->InitSocket();

	//// �����׽���
	//// AF_INET:�׽���Э��
	//// SCOK_DGRAM:�׽������ͣ���ʾ������UDP
	//this->SetServerSocket(socket(AF_INET, SOCK_DGRAM, 0));

	////��
	//if (bind(ServerSocket, (SOCKADDR*)&ServerAddr, sizeof(ServerAddr)) == SOCKET_ERROR)
	//{
	//	cout << "�׽��ְ�ʧ�ܣ�" << endl;
	//	WSACleanup();
	//}
	//else
	//	cout << "�׽��ְ󶨳ɹ���" << endl;
}

void CUDP::InitSocket()
{
	//��ʼ���׽��ֿ�
	WORD w_req = MAKEWORD(2, 2);//�汾��
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata); //�������/��������⣬����������⣬�������ĺ���/���ܲ���ʹ��
	if (err != 0) {
		cout << "��ʼ���׽��ֿ�ʧ��" << endl;
	}
	else {
		cout << "��ʼ���׽��ֿ�ɹ�" << endl;
	}
	//���汾��
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "�׽��ֿ�汾�Ų���" << endl;
		WSACleanup();
	}
	else {
		cout << "�׽��ֿ�汾��ȷ" << endl;
	}
}

DWORD WINAPI UDPServer(LPVOID lpParameter)
{
	CUDP* udp = (CUDP*)lpParameter;
	SOCKET* ServerSocket = udp->GetSocket();
	SOCKADDR_IN* ServerAddr = udp->GetServerAddr();
	// char RecvBuff = udp->GetRecvBuff();
	char RecvBuff[0x3FFF];
	SOCKADDR_IN* RecvAddr = udp->GetRecvAddr();
	UDPSendDataStruct* UDPData = udp->GetUDPData();
	faultParaStruct* faultPara = udp->GetfaultPara();
	bool* injectFlag = udp->GetInjectFlag();
	float* runTimeTotal = udp->GetRunTimeTotal();
	bool* RunFlag = udp->GetRunFlag();
	bool* SaveDataFlag = udp->GetSaveDataFlag();
	saveDataStruct* saveData = udp->GetSaveData();

	int* RecvCount = udp->GetRecvCount();

	DWORD WINAPI flag = 0;
	//���巢�ͻ������ͽ��ܻ�����
	int RecvLen = 0;
	int RecvAddrLen = 0;

	// �����׽���IOģʽ���ز���ָ��
	u_long iMode = 1;

	// ��ʼ���׽���
	udp->InitSocket();

	// �����׽���
	// AF_INET:�׽���Э��
	// SCOK_DGRAM:�׽������ͣ���ʾ������UDP
	udp->SetServerSocket(socket(AF_INET, SOCK_DGRAM, 0));

	//��
	if (bind(*ServerSocket, (SOCKADDR*)ServerAddr, sizeof(*ServerAddr)) == SOCKET_ERROR)
	{
		cout << "�׽��ְ�ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else
		cout << "�׽��ְ󶨳ɹ���" << endl;

	RecvAddrLen = sizeof(*RecvAddr);
	ioctlsocket(*ServerSocket, FIONBIO, &iMode);

	//��������
	while (1) {
		//������һ����������
		WaitForSingleObject(Mutex, INFINITE);

		memset(RecvBuff, 0, sizeof(RecvBuff));

		RecvLen = recvfrom(*ServerSocket, RecvBuff, sizeof(RecvBuff), 0, (sockaddr*)RecvAddr, &RecvAddrLen);

		if (RecvLen < 0)
		{
			//�ͷŻ�������
			ReleaseMutex(Mutex);
			continue;
		}

		// �������յ���UDP���ݱ�
		if (RecvBuff[0] == 'S' && RecvBuff[7] == 'E')
		{
			// ����ע����Ϣ 
			if (RecvBuff[1] == 'F' && RecvBuff[2] == 'I' && RecvBuff[3] == 'D' &&
				RecvBuff[4] == 'A' && RecvBuff[5] == 'T' && RecvBuff[6] == 'A')
			{
				memcpy(faultPara, RecvBuff + 8, sizeof(faultParaStruct));
				(*injectFlag) = true;
				std::cout << "����ע�����" << endl;
			}
			// ��������
			else if (RecvBuff[1] == 'R' && RecvBuff[2] == 'E' && RecvBuff[3] == 'D' &&
				RecvBuff[4] == 'A' && RecvBuff[5] == 'T' && RecvBuff[6] == 'A')
			{
				(*RecvCount)++;
				std::cout << "����������..." << endl;
			}
			// ��ʼ����
			else if (RecvBuff[1] == 'R' && RecvBuff[2] == 'U' && RecvBuff[3] == 'N' &&
				RecvBuff[4] == 'R' && RecvBuff[5] == 'U' && RecvBuff[6] == 'N')
			{
				memcpy(runTimeTotal, RecvBuff + 8, sizeof(float));
				(*RunFlag) = true;
				std::cout << "��ʼ����" << endl;
			}
			// ֹͣ����
			else if (RecvBuff[1] == 'S' && RecvBuff[2] == 'T' && RecvBuff[3] == 'O' &&
				RecvBuff[4] == 'P' && RecvBuff[5] == 'S' && RecvBuff[6] == 'T')
			{
				(*RunFlag) = false;
				std::cout << "ֹͣ����" << endl;
			}

			// ��������
			else if (RecvBuff[1] == 'S' && RecvBuff[2] == 'A' && RecvBuff[3] == 'V' &&
				RecvBuff[4] == 'E' && RecvBuff[5] == 'D' && RecvBuff[6] == 'A')
			{
				(*SaveDataFlag) = true;
				memcpy(saveData, RecvBuff + 8, sizeof(char)*NAMA_LEN_MAX);
				std::cout << "��ʼ��������" << endl;
			}
		}

		//�ͷŻ�������
		ReleaseMutex(Mutex);
		
	}
	//�ر��׽���
	closesocket(*ServerSocket);
	//�ͷ�DLL��Դ
	WSACleanup();
	return flag;
}



int CUDP::SendUDPData()
{
	int SendLen = 0;

	// ���ջ����������ڶ��壬�ᱻ�ⲿ����
	char SendBuff[200] = {0};

	memset(SendBuff, 0, sizeof(SendBuff));

	SendBuff[0] = 'P';// C:��������P�����᷽��

	memcpy(SendBuff + 1, &this->UDPData, sizeof(this->UDPData));
	SendBuff[1 + sizeof(this->UDPData)] = 'E';

	SendLen = sendto(*this->GetSocket(), SendBuff, sizeof(SendBuff), 0, (sockaddr*)&RecvAddr, sizeof(RecvAddr));

	if (SendLen < 0)
		cout << "����ʧ�ܣ�" << endl;

	return SendLen;
}


void CUDP::SetServerSocket(SOCKET soc)
{
	this->ServerSocket = soc;
}

UDPSendDataStruct* CUDP::GetUDPData()
{
	return &this->UDPData;
}

faultParaStruct* CUDP::GetfaultPara()
{
	return &this->faultPara;
}


SOCKET* CUDP::GetSocket()
{
	return &this->ServerSocket;
}

SOCKADDR_IN* CUDP::GetServerAddr()
{
	return &this->ServerAddr;
}


SOCKADDR_IN* CUDP::GetRecvAddr()
{
	return &this->RecvAddr;
}


//char* CUDP::GetRecvBuff()
//{
//	return this->RecvBuff;
//}


int* CUDP::GetRecvCount()
{
	return &this->RecvCount;
}


bool* CUDP::GetInjectFlag()
{
	return &this->injectFlag;
}


float* CUDP::GetRunTimeTotal()
{
	return &this->runTimeTotal;
}


bool* CUDP::GetRunFlag()
{
	return &this->RunFlag;
}

bool* CUDP::GetSaveDataFlag()
{
	return &this->SaveDataFlag;
}

saveDataStruct* CUDP::GetSaveData()
{
	return &this->saveData;
}
