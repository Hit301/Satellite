#pragma once
#include <windows.h>
#include <winsock.h>
#pragma comment(lib,"ws2_32.lib")


typedef struct
{
	double Omega_b[3];
	double Qib[4];
}USARTDataStruct;


class CUSART
{
public:
	CUSART(const char* comPara="COM1"); // 初始化函数
	
	// 打开串口,成功返回句柄，失败返回NULL
	// portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等
	// baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200 
	// parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验
	// databit(数据位): 4-8，通常为8位
	// stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位
	// synchronizable(同步、异步): 0为同步步，1为异步
	HANDLE Serial_open(const char* COMx, int BaudRate=115200, char parity=0, char databit=8, char stopbit=1, char synchronizeflag=0);

	int Serial_read(HANDLE hCom, void* OutBuf, int size);
	int SendUsartData(); // 向串口发送数据
	int Serial_write(HANDLE hCom, const void* Buf, int size);
	void Serial_close(HANDLE hCom);

	USARTDataStruct* GetUSARTData();
	HANDLE GetHCom();
	int* GetRecvCount();
	int* GetRecvFlag();
	unsigned char* GetRecvBuff();
	int* GetEndFlag();


private:
	USARTDataStruct USARTData;
	HANDLE hCom;
	
	int RecvCount;
	int USART_RX_STA; // 串口接收状态标志位
	unsigned char USART_RX_BUF[0X3FFF]; // USATRT接收缓冲区
	int RecvEndFlag; // 串口接收结束标志
};

DWORD WINAPI USARTServer(LPVOID lpParameter);
