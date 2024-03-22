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
	CUSART(const char* comPara="COM1"); // ��ʼ������
	
	// �򿪴���,�ɹ����ؾ����ʧ�ܷ���NULL
	// portname(������): ��Windows����"COM1""COM2"�ȣ���Linux����"/dev/ttyS1"��
	// baudrate(������): 9600��19200��38400��43000��56000��57600��115200 
	// parity(У��λ): 0Ϊ��У�飬1Ϊ��У�飬2ΪżУ�飬3Ϊ���У��
	// databit(����λ): 4-8��ͨ��Ϊ8λ
	// stopbit(ֹͣλ): 1Ϊ1λֹͣλ��2Ϊ2λֹͣλ,3Ϊ1.5λֹͣλ
	// synchronizable(ͬ�����첽): 0Ϊͬ������1Ϊ�첽
	HANDLE Serial_open(const char* COMx, int BaudRate=115200, char parity=0, char databit=8, char stopbit=1, char synchronizeflag=0);

	int Serial_read(HANDLE hCom, void* OutBuf, int size);
	int SendUsartData(); // �򴮿ڷ�������
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
	int USART_RX_STA; // ���ڽ���״̬��־λ
	unsigned char USART_RX_BUF[0X3FFF]; // USATRT���ջ�����
	int RecvEndFlag; // ���ڽ��ս�����־
};

DWORD WINAPI USARTServer(LPVOID lpParameter);
