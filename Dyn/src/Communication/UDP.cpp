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

	// 设置服务端信息
	ServerAddr.sin_family = AF_INET;
	ServerAddr.sin_addr.S_un.S_addr = inet_addr(cp);
	ServerAddr.sin_port = htons(port);

	//// 初始化套接字
	//this->InitSocket();

	//// 创建套接字
	//// AF_INET:套接字协议
	//// SCOK_DGRAM:套接字类型，表示无连接UDP
	//this->SetServerSocket(socket(AF_INET, SOCK_DGRAM, 0));

	////绑定
	//if (bind(ServerSocket, (SOCKADDR*)&ServerAddr, sizeof(ServerAddr)) == SOCKET_ERROR)
	//{
	//	cout << "套接字绑定失败！" << endl;
	//	WSACleanup();
	//}
	//else
	//	cout << "套接字绑定成功！" << endl;
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
	//定义发送缓冲区和接受缓冲区
	int RecvLen = 0;
	int RecvAddrLen = 0;

	// 定义套接字IO模式返回参数指针
	u_long iMode = 1;

	// 初始化套接字
	udp->InitSocket();

	// 创建套接字
	// AF_INET:套接字协议
	// SCOK_DGRAM:套接字类型，表示无连接UDP
	udp->SetServerSocket(socket(AF_INET, SOCK_DGRAM, 0));

	//绑定
	if (bind(*ServerSocket, (SOCKADDR*)ServerAddr, sizeof(*ServerAddr)) == SOCKET_ERROR)
	{
		cout << "套接字绑定失败！" << endl;
		WSACleanup();
	}
	else
		cout << "套接字绑定成功！" << endl;

	RecvAddrLen = sizeof(*RecvAddr);
	ioctlsocket(*ServerSocket, FIONBIO, &iMode);

	//接收数据
	while (1) {
		//请求获得一个互斥量锁
		WaitForSingleObject(Mutex, INFINITE);

		memset(RecvBuff, 0, sizeof(RecvBuff));

		RecvLen = recvfrom(*ServerSocket, RecvBuff, sizeof(RecvBuff), 0, (sockaddr*)RecvAddr, &RecvAddrLen);

		if (RecvLen < 0)
		{
			//释放互斥量锁
			ReleaseMutex(Mutex);
			continue;
		}

		// 解析接收到的UDP数据报
		if (RecvBuff[0] == 'S' && RecvBuff[7] == 'E')
		{
			// 故障注入信息 
			if (RecvBuff[1] == 'F' && RecvBuff[2] == 'I' && RecvBuff[3] == 'D' &&
				RecvBuff[4] == 'A' && RecvBuff[5] == 'T' && RecvBuff[6] == 'A')
			{
				memcpy(faultPara, RecvBuff + 8, sizeof(faultParaStruct));
				(*injectFlag) = true;
				std::cout << "故障注入完成" << endl;
			}
			// 传输数据
			else if (RecvBuff[1] == 'R' && RecvBuff[2] == 'E' && RecvBuff[3] == 'D' &&
				RecvBuff[4] == 'A' && RecvBuff[5] == 'T' && RecvBuff[6] == 'A')
			{
				(*RecvCount)++;
				std::cout << "传输数据中..." << endl;
			}
			// 开始运行
			else if (RecvBuff[1] == 'R' && RecvBuff[2] == 'U' && RecvBuff[3] == 'N' &&
				RecvBuff[4] == 'R' && RecvBuff[5] == 'U' && RecvBuff[6] == 'N')
			{
				memcpy(runTimeTotal, RecvBuff + 8, sizeof(float));
				(*RunFlag) = true;
				std::cout << "开始运行" << endl;
			}
			// 停止运行
			else if (RecvBuff[1] == 'S' && RecvBuff[2] == 'T' && RecvBuff[3] == 'O' &&
				RecvBuff[4] == 'P' && RecvBuff[5] == 'S' && RecvBuff[6] == 'T')
			{
				(*RunFlag) = false;
				std::cout << "停止运行" << endl;
			}

			// 保存数据
			else if (RecvBuff[1] == 'S' && RecvBuff[2] == 'A' && RecvBuff[3] == 'V' &&
				RecvBuff[4] == 'E' && RecvBuff[5] == 'D' && RecvBuff[6] == 'A')
			{
				(*SaveDataFlag) = true;
				memcpy(saveData, RecvBuff + 8, sizeof(char)*NAMA_LEN_MAX);
				std::cout << "开始保存数据" << endl;
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
