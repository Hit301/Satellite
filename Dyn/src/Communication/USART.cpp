#include "Communication/USART.h"
#include "Communication/BaseCommuPara.h"
#include <stdio.h>
#include <iostream>

using namespace std;


CUSART::CUSART(const char* comPara)
{
	this->USART_RX_STA = 0;
	memset(this->USART_RX_BUF, 0, sizeof(USART_RX_BUF));
	this->RecvEndFlag = 0;
	this->RecvEndFlag = 0;
	this->hCom = NULL;

	this->hCom = Serial_open(comPara, 115200);
	if (this->hCom == NULL)
	{
		exit(0);
	}
}

int CUSART::SendUsartData()
{
	const int dataBytes = sizeof(USARTData) + 8 + 8;
	unsigned char byte[dataBytes];
	byte[0] = 'S';
	byte[1] = 'd';
	byte[2] = 'a';
	byte[3] = 't';
	byte[4] = 'a';
	byte[5] = 'U';
	byte[6] = 'S';
	byte[7] = 'A';

	unsigned char endByte[8];
	endByte[0] = 'E';
	endByte[1] = 'E';
	endByte[2] = 'E';
	endByte[3] = 'E';
	endByte[4] = 'E';
	endByte[5] = 'E';
	endByte[6] = 'E';
	endByte[7] = 'E';

	memcpy(byte + 8, &USARTData, sizeof(USARTData));

	memcpy(byte + 8 + sizeof(USARTData), endByte, sizeof(endByte));

	int sendLen = Serial_write(this->hCom, byte, sizeof(byte));
	cout << "发送字节数：" << sendLen << endl;
	return sendLen;
}

DWORD WINAPI USARTServer(LPVOID lpParameter)
{
	CUSART* usart = (CUSART*)lpParameter;
	unsigned char USART_RX_BUF[0X3FFF] = { 0 };
	int* RecvEndFlag = usart->GetEndFlag();
	int* USART_RX_STA = usart->GetRecvFlag();
	int* RecvCount = usart->GetRecvCount();
	HANDLE hCom = usart->GetHCom();

	char readbyte[90] = { 0 };
	unsigned char u_readbyte[90] = { 0 };

	unsigned long temp;
	int uart_flag = 0;
	unsigned char uart_char1[1] = { 0 };
	char uart_char21[89] = { 0 };

	int data[5] = { 0 };
	double T_recv[3] = { 0 };

	COMMTIMEOUTS TimeOuts;
	memset(&TimeOuts, 0, sizeof(TimeOuts));
	TimeOuts.ReadIntervalTimeout = MAXDWORD;
	TimeOuts.ReadTotalTimeoutConstant = 2;
	TimeOuts.ReadTotalTimeoutMultiplier = 2;
	TimeOuts.WriteTotalTimeoutConstant = 0;
	TimeOuts.WriteTotalTimeoutMultiplier = 0;

	SetCommTimeouts(hCom, &TimeOuts);

	while (1)
	{
		int ret;
		WaitForSingleObject(Mutex, INFINITE);

		ret = usart->Serial_read(hCom, uart_char1, 1);

		if (!ret)
		{
			ReleaseMutex(Mutex);
			continue;
		}

		if ((*USART_RX_STA & 0x8000) == 0)
		{
			if (*USART_RX_STA & 0x4000)
			{
				if (uart_char1[0] != 'E') // E为中间字符，接收继续
				{
					*RecvEndFlag = 0;
					USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
					(*USART_RX_STA)++;
					*USART_RX_STA &= 0xbfff; // 接收标志位复位
				}
				else
				{
					(*RecvEndFlag)++;
					USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
					(*USART_RX_STA)++;
				}

				if (*RecvEndFlag == 8) // 连续8个E，结束标志
				{
					*RecvEndFlag = 0;
					
					(*RecvCount)++;
					cout << "接收成功" << endl;

					memcpy(usart->GetRecvBuff(), USART_RX_BUF, sizeof(USART_RX_BUF));

					*USART_RX_STA = 0;
				}
			}
			else
			{
				if (uart_char1[0] == 'S' && *USART_RX_STA == 0)
				{
					USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
					(*USART_RX_STA)++;
				}
				else if (*USART_RX_STA == 1 && uart_char1[0] == 'A') 
				{
					USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
					(*USART_RX_STA)++;
				}
				else if (*USART_RX_STA >= 2 && USART_RX_BUF[0] == 'S' && USART_RX_BUF[1] == 'A')
				{
					if (uart_char1[0] == 'E')
					{
						(*RecvEndFlag)++; // 1
						USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
						(*USART_RX_STA)++;
						*USART_RX_STA |= 0x4000;
					}
					else
					{
						USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
						(*USART_RX_STA)++;
						if (*USART_RX_STA > (0X3FFF))
						{
							*USART_RX_STA = 0;
							memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
						}
					}
				}
				else
				{
					*USART_RX_STA = 0;
					memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
					if (uart_char1[0] == 'S' && *USART_RX_STA == 0)
					{
						USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
						(*USART_RX_STA)++;
					}
				}
			}

		}
		else
		{
			*USART_RX_STA = 0;
			memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
			if (uart_char1[0] == 'S' && *USART_RX_STA == 0)
			{
				USART_RX_BUF[*USART_RX_STA & 0X3FFF] = uart_char1[0];
				(*USART_RX_STA)++;
			}
		}

		ReleaseMutex(Mutex);
	}
	return 0L;
}



HANDLE CUSART::Serial_open(const char* COMx, int BaudRate, char parity, char databit, char stopbit, char synchronizeflag)
{
	HANDLE hCom = NULL;
	DCB dcb = { 0 };
	if (synchronizeflag == 0)
	{
		hCom = CreateFile(COMx,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			0, // 0：同步方式; FILE_FLAG_OVERLAPPED：异步方式
			0
		);
	}
	else if (synchronizeflag == 1)
	{
		hCom = CreateFile(COMx,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_FLAG_OVERLAPPED, // 0：同步方式; FILE_FLAG_OVERLAPPED：异步方式
			0
		);
	}
	else {
		printf("同步异步模式参数未知");
		return NULL;
	}
	

	if (hCom == INVALID_HANDLE_VALUE)
	{
		DWORD dwError = GetLastError();
		printf("串口%s打开失败\n", COMx);
		return NULL;
		exit(0);
	}
	else
		printf("串口%s打开成功, 波特率: % d!\n", COMx, BaudRate);

	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState(hCom, &dcb))
	{
		DWORD dwError = GetLastError();
		return(HANDLE)(-1);
	}

	dcb.BaudRate = BaudRate; // 波特率
	dcb.ByteSize = databit; // 数据位

	switch (parity) //校验位
	{
	case 0:
		dcb.Parity = NOPARITY; //无校验
		break;
	case 1:
		dcb.Parity = ODDPARITY; //奇校验
		break;
	case 2:
		dcb.Parity = EVENPARITY; //偶校验
		break;
	case 3:
		dcb.Parity = MARKPARITY; //标记校验
		break;
	}

	switch (stopbit) //停止位
	{
	case 1:
		dcb.StopBits = ONESTOPBIT; //1位停止位
		break;
	case 2:
		dcb.StopBits = TWOSTOPBITS; //2位停止位
		break;
	case 3:
		dcb.StopBits = ONE5STOPBITS; //1.5位停止位
		break;
	}


	if (!SetCommState(hCom, &dcb))
	{
		DWORD dwError = GetLastError();
		return(HANDLE)(-1);
	}
	if (!PurgeComm(hCom, PURGE_RXCLEAR))   return(HANDLE)(-1);

	// 配置缓冲区大小
	if (!SetupComm(hCom, 1024, 1024))
	{
		return NULL;
	}

	// 超时处理
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = MAXDWORD; //读间隔超时
	// 把间隔超时设为最大，把总超时设为0将导致ReadFile立即返回并完成操作 

	timeouts.ReadTotalTimeoutMultiplier = 500; //读时间系数
	timeouts.ReadTotalTimeoutConstant = 500; //读时间常量
	timeouts.WriteTotalTimeoutMultiplier = 500; // 写时间系数
	timeouts.WriteTotalTimeoutConstant = 2000; //写时间常量
	//总的读/写超时时间 = Read(Write)TotalTimeoutMultiplier x 要读/写的字节数 + Read(Write)TotalTimeoutConstant. 
	if (SetCommTimeouts(hCom, &timeouts) == false)
	{
		return NULL;
	}

	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//清空串口缓冲区

	return hCom;;
}


int CUSART::Serial_read(HANDLE hCom, void* OutBuf, int size)
{
	DWORD cnt = 0;

	ReadFile(hCom, OutBuf, size, &cnt, 0);
	/*if (!cnt)
	{
		std::cout << "读串口失败" << std::endl;
	}*/
	return cnt;

}


int CUSART::Serial_write(HANDLE hCom, const void* Buf, int size)
{
	DWORD dw;
	WriteFile(hCom, Buf, size, &dw, NULL);
	return dw;
}


void CUSART::Serial_close(HANDLE hCom)
{
	CloseHandle(hCom);
}

USARTDataStruct* CUSART::GetUSARTData()
{
	return &this->USARTData;
}

HANDLE CUSART::GetHCom()
{
	return this->hCom;
}

int* CUSART::GetRecvCount()
{
	return &this->RecvCount;
}

int* CUSART::GetRecvFlag()
{
	return &this->USART_RX_STA;
}

unsigned char* CUSART::GetRecvBuff()
{
	return USART_RX_BUF;
}

int* CUSART::GetEndFlag()
{
	return &this->RecvEndFlag;
}

