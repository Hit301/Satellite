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

	// 设置服务端信息
	ServerAddr.sin_family = AF_INET;
	ServerAddr.sin_addr.S_un.S_addr = inet_addr(cp);
	ServerAddr.sin_port = htons(port);
}

void CUDP::InitSocket()
{
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata); //打开网络库/启动网络库，启动了这个库，这个库里的函数/功能才能使用
	if (err != 0) {
		cout << "初始化套接字库失败" << endl;
	}
	else {
		cout << "初始化套接字库成功" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确" << endl;
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
	//定义发送缓冲区和接受缓冲区
	int RecvLen = 0;
	int RecvAddrLen = 0;

	// 定义套接字IO模式返回参数指针
	u_long iMode = 1;

	// 初始化套接字
	udp->InitSocket();

	//创建套接字
	// AF_INET:套接字协议
	// SCOK_DGRAM:套接字类型，表示无连接UDP
	udp->SetServerSocket(socket(AF_INET, SOCK_DGRAM, 0));

	//绑定
	if (bind(*ServerSocket, (SOCKADDR*)ServerAddr, sizeof(*ServerAddr)) == SOCKET_ERROR)
	{
		cout << "套接字绑定失败！" << endl;
		WSACleanup();
		return flag;
	}

	cout << "套接字绑定成功！" << endl;

	//接收数据
	while (1) {
		//请求获得一个互斥量锁
		WaitForSingleObject(Mutex, INFINITE);

		memset(RecvBuff, 0, sizeof(RecvBuff));
		RecvAddrLen = sizeof(*RecvAddr);

		ioctlsocket(*ServerSocket, FIONBIO, &iMode);

		RecvLen = recvfrom(*ServerSocket, RecvBuff, sizeof(RecvBuff), 0, (sockaddr*)RecvAddr, &RecvLen);

		if (RecvLen < 0)
		{
			//释放互斥量锁
			ReleaseMutex(Mutex);
			continue;
		}

		//解析接收到的UDP数据报
		if (RecvBuff[0] == 'S' && RecvBuff[7] == 'E')
		{
			//故障注入信息 
			if (RecvBuff[1] == 'F' && RecvBuff[2] == 'A' && RecvBuff[3] == 'U')
			{
				(*RecvCount)++;
			}
		}

		//释放互斥量锁
		ReleaseMutex(Mutex);
		
	}
	//关闭套接字
	closesocket(*ServerSocket);
	//释放DLL资源
	WSACleanup();
	return flag;
}



int CUDP::SendUDPData()
{
	int SendLen = 0;

	// 接收缓冲区在类内定义，会被外部调用
	char SendBuff[200] = {0};

	memset(SendBuff, 0, sizeof(SendBuff));

	SendBuff[0] = 'P';// C:本方法，P：所提方法

	memcpy(SendBuff + 1, &this->UDPData, sizeof(this->UDPData));
	SendBuff[1 + sizeof(this->UDPData)] = 'E';

	SendLen = sendto(*this->GetSocket(), SendBuff, sizeof(SendBuff), 0, (sockaddr*)&RecvAddr, sizeof(RecvAddr));

	if (SendLen < 0)
		cout << "发送失败！" << endl;

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
