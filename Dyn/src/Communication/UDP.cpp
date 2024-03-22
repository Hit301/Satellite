#include "Communication/UDP.h"
#include "Communication/BaseCommuPara.h"

#include <iostream>

using namespace std;


CUDP::CUDP(const char* cpPara, u_short portPara)
{
	memset(&UDPData, 0, sizeof(UDPData));
	memset(&RecvBuff, 0, sizeof(RecvBuff));
	memset(&ServerAddr, 0, sizeof(ServerAddr));
	memset(&RecvAddr, 0, sizeof(RecvAddr));
	ServerSocket = 0;
	RecvCount = 0;

	cp = cpPara;
	port = portPara;

	// ���÷������Ϣ
	ServerAddr.sin_family = AF_INET;
	ServerAddr.sin_addr.S_un.S_addr = inet_addr(cp);
	ServerAddr.sin_port = htons(port);
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
	char* RecvBuff = udp->GetRecvBuff();
	SOCKADDR_IN* RecvAddr = udp->GetRecvAddr();
	UDPSendDataStruct* UDPData = udp->GetUDPData();
	int* RecvCount = udp->GetRecvCount();

	DWORD WINAPI flag = 0;
	//���巢�ͻ������ͽ��ܻ�����
	int RecvLen = 0;
	int RecvAddrLen = 0;

	// �����׽���IOģʽ���ز���ָ��
	u_long iMode = 1;

	// ��ʼ���׽���
	udp->InitSocket();

	//�����׽���
	// AF_INET:�׽���Э��
	// SCOK_DGRAM:�׽������ͣ���ʾ������UDP
	udp->SetServerSocket(socket(AF_INET, SOCK_DGRAM, 0));

	//��
	if (bind(*ServerSocket, (SOCKADDR*)ServerAddr, sizeof(*ServerAddr)) == SOCKET_ERROR)
	{
		cout << "�׽��ְ�ʧ�ܣ�" << endl;
		WSACleanup();
		return flag;
	}

	cout << "�׽��ְ󶨳ɹ���" << endl;

	//��������
	while (1) {
		//������һ����������
		WaitForSingleObject(Mutex, INFINITE);

		memset(RecvBuff, 0, sizeof(RecvBuff));
		RecvAddrLen = sizeof(*RecvAddr);

		ioctlsocket(*ServerSocket, FIONBIO, &iMode);

		RecvLen = recvfrom(*ServerSocket, RecvBuff, sizeof(RecvBuff), 0, (sockaddr*)RecvAddr, &RecvLen);

		if (RecvLen < 0)
		{
			//�ͷŻ�������
			ReleaseMutex(Mutex);
			continue;
		}

		//�������յ���UDP���ݱ�
		if (RecvBuff[0] == 'S' && RecvBuff[7] == 'E')
		{
			//����ע����Ϣ 
			if (RecvBuff[1] == 'F' && RecvBuff[2] == 'A' && RecvBuff[3] == 'U')
			{
				(*RecvCount)++;
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


char* CUDP::GetRecvBuff()
{
	return this->RecvBuff;
}

int* CUDP::GetRecvCount()
{
	return &this->RecvCount;
}
