//file***************************************************************************************************
//                                                                                                       
//                                                                                                       
//        UDP_Export.cpp for  Master                  
//                                                                                                       
//                                                                                                       
//*******************************************************************************************************
//               For UDP Communication (PC<=>DSP)                                                                                        
//******* Copyright(C) 2015 GIBH ******************************************************************
//                                                                                                       
//      Ver 1.00    2015.01.14   Initial coding                                            Enjoy_Lu     
//*******************************************************************************************************

#pragma once
#include "stdafx.h"
#include "stdio.h"
#include "UDP_Rerro.h"
#include "UDP_Export.h"
#include <WinSock2.h> 

#pragma comment(lib,"ws2_32.lib") 


SOCKET sino_socket;
HANDLE sdv_hThread;
HANDLE sdv_hThread1;

HANDLE mutex_udp_send;
CRITICAL_SECTION M_lock;

StreamProc gCallBack;
StreamProc gPumpCallBack;
void*pUserData1;
SOCKADDR_IN addrFrom;
SOCKADDR_IN addrscoket5;
SOCKADDR_IN addrFrom1[255];
SOCKADDR_IN Send_addr[255];
char connet_buf[2];
int test_sock(SOCKADDR_IN kk);

static DWORD WINAPI UDP_ThreadProc1(void* pCtrl);
static DWORD WINAPI UDP_ThreadProc2(void* pCtrl);
unsigned char Sendbuffer[2048];
HANDLE	sdv_hTerminate1 = CreateEvent(NULL, TRUE, FALSE, NULL);
HANDLE	sdv_hTerminate2;

long __stdcall UDP_InitNet()
{
	WSADATA wsData;
	SOCKADDR_IN addrscoket;
	int retdate;
	WSAStartup(MAKEWORD(2, 2), &wsData);
	sino_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (INVALID_SOCKET == sino_socket)
	{
		return SDV_RESULT_SOCK_ERR;
	}
	connet_buf[0] = 0xcc;
	connet_buf[1] = 0xcc;
	bool opt = true;
	memset(Send_addr, 0, sizeof(SOCKADDR_IN));
	// 	addrscoket5.sin_addr.S_un.S_addr=inet_addr("192.168.1.191");
	// 	addrscoket5.sin_family=AF_INET;
	// 	addrscoket5.sin_port=htons(5000);
	addrscoket.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	addrscoket.sin_family = AF_INET;
	addrscoket.sin_port = htons(5050);
	setsockopt(sino_socket, SOL_SOCKET, SO_BROADCAST, (char FAR*)&opt, sizeof(opt));
	retdate = bind(sino_socket, (SOCKADDR*)&addrscoket, sizeof(SOCKADDR));
	if (retdate == SOCKET_ERROR)
	{
		closesocket(sino_socket);
		return SDV_RESULT_BIND_ERR;
	}
	return SDV_RESULT_OK;
}

long  __stdcall SetPumpStreamHOOK(StreamProc lpStreamFun)
{
	gPumpCallBack = lpStreamFun;

	return  SDV_RESULT_OK;
}

long  __stdcall SetStreamHOOK(StreamProc lpStreamFun, void*pUserData)
{
	if (lpStreamFun == NULL)
	{
		return SDV_RESULT_HOOK_ERR;
	}
	gCallBack = lpStreamFun;
	pUserData1 = pUserData;
	sdv_hTerminate2 = CreateEvent(NULL, TRUE, FALSE, NULL);
	mutex_udp_send = CreateMutex(NULL, TRUE, NULL);
	//InitializeCriticalSection(&M_lock);
	InitializeCriticalSectionAndSpinCount(&M_lock, 4000);
	SetEvent(sdv_hTerminate1);
	SetEvent(sdv_hTerminate2);
	sdv_hThread = CreateThread(NULL, 0, UDP_ThreadProc1, NULL, CREATE_SUSPENDED, NULL);
	//sdv_hThread1=CreateThread(NULL,0,UDP_ThreadProc2,NULL,0,NULL);
	SetThreadPriority(sdv_hThread, THREAD_PRIORITY_ABOVE_NORMAL);
	ResumeThread(sdv_hThread);
	if (sdv_hThread == FALSE)
		return SDV_RESULT_THREAD_ERR;
	else
	{
		CloseHandle(sdv_hThread);
		return  SDV_RESULT_OK;

	}
}

DWORD WINAPI	UDP_ThreadProc1(void* pCtrl)
{

	SOCKET sock = sino_socket;
	int lengh = sizeof(SOCKADDR);
	int retval;
	UCHAR m_Ip;
	int Recvlen[2];
	//	DWORD        bytes; 
	UCHAR recvBuffer[1024];
	UCHAR mm_buf[1024] = { 0 };
	while (WaitForSingleObject(sdv_hTerminate1, INFINITE) == WAIT_OBJECT_0)
	{
		retval = recvfrom(sock, (char*)recvBuffer, 1000, 0, (SOCKADDR*)&addrFrom, &lengh);

		m_Ip = unsigned char((addrFrom.sin_addr.S_un.S_addr) >> 24);
		if (m_Ip == 186)
		{
			//printf("ip=186 len=%d\n", retval);
			if (retval<24)
				continue;
			if (recvBuffer[0] == 0xAA && recvBuffer[1] == 0xAA)
			{
				addrFrom1[m_Ip] = addrFrom;
				Recvlen[0] = retval;
				(*gPumpCallBack)(recvBuffer, Recvlen);
			}
			continue;
		}

		if (m_Ip == 20)
		{
			//printf("ip=186 len=%d\n", retval);
			if (retval<24)
				continue;

			addrFrom1[m_Ip] = addrFrom;
			Recvlen[0] = retval;
			(*gCallBack)(recvBuffer, Recvlen);

			continue;
		}
		else
		{
			continue;
		}

		if ((recvBuffer[0] != 0xaa && recvBuffer[0] != 0xcc) && (recvBuffer[1] != 0xaa && recvBuffer[1] != 0xcc))
			continue;

		//addrFrom.sin_port=htons(addrFrom.sin_port);
		//addrFrom.sin_addr.S_un.S_addr=htonl(addrFrom.sin_addr.S_un.S_addr);
		addrFrom1[m_Ip] = addrFrom;
		if (recvBuffer[0] == 0xcc && recvBuffer[1] == 0xcc)
			continue;
		mm_buf[0] = m_Ip;
		mm_buf[1] = (UCHAR)recvBuffer[3] * 4;
		memcpy(&mm_buf[2], &recvBuffer[4], mm_buf[1]);
		(*gCallBack)(mm_buf, pUserData1);
	}
	return FALSE;
}


DWORD WINAPI	UDP_ThreadProc2(void* pCtrl)
{
	DWORD ret;

	while (1)
	{
		ret = WaitForSingleObject(sdv_hTerminate2, 3000);
		if (ret == 0)
		{
			/*for(int i=0;i<255;i++)
			{
			if (Send_addr[i].sin_addr.S_un.S_addr!=0)
			{
			int ret=sendto(sino_socket,connet_buf,2,0,(SOCKADDR*)&(Send_addr[i]),sizeof(SOCKADDR));

			}
			Sleep(10);
			}*/
			//etEvent(sdv_hTerminate2);
			ResetEvent(sdv_hTerminate2);
		}
		else if (ret == 258)
		{


		}
		Sleep(1000);

	}
}

void UDPSendTest(void)
{
	int ret;

	for (int i = 0; i<255; i++)
	{
		if (Send_addr[i].sin_addr.S_un.S_addr != 0)
		{
			ret = sendto(sino_socket, connet_buf, 2, 0, (SOCKADDR*)&(Send_addr[i]), sizeof(SOCKADDR));
			if (ret<0)
			{
				printf("test send udp data fail\n");
			}
		}
		Sleep(10);
	}
}

long __stdcall UDP_DualCoreSendData(char*Buffer, unsigned int Length, unsigned char Ip_flg)
{
	int Num;
	EnterCriticalSection(&M_lock);
	Num = 0;

	if (Buffer != NULL)
	{
		memcpy(&Sendbuffer[0], Buffer, Length);
		Num = sendto(sino_socket, (char*)Sendbuffer, Length, 0, (SOCKADDR*)&(addrFrom1[Ip_flg]), sizeof(SOCKADDR));
		if (Num<0)
		{
			printf("DualCore send udp data fail\n");
			Num = 0;
		}
		else
		{

		}
	}
	LeaveCriticalSection(&M_lock);
	//ReleaseMutex(mutex_udp_send);
	return Num;
}

// long __stdcall UDP_SendData(char*Buffer, unsigned char Length, unsigned char Ip_flg)
// {
// 	int Num;
// 	//EnterCriticalSection(&M_lock);
// 	//WaitForSingleObject(mutex_udp_send,INFINITE);
// 	Num = 0;
// 	Sendbuffer[0] = 0xaa;
// 	Sendbuffer[1] = 0xaa;
// 	Sendbuffer[2] = (Length / 4);
// 	Sendbuffer[3] = (Length / 4);
// 	//printf("send udp data·¢ËÍ\n");
// 	if (Buffer != NULL)
// 	{
// 		memcpy(&Sendbuffer[4], Buffer, Length);
// 		Num = sendto(sino_socket, (char*)Sendbuffer, Length + 4, 0, (SOCKADDR*)&(addrFrom1[Ip_flg]), sizeof(SOCKADDR));
// 		if (Num<0)
// 		{
// 			printf("send udp data fail\n");
// 			Num = 0;
// 		}
// 		else
// 		{
// 
// 		}
// 	}
// 	//LeaveCriticalSection(&M_lock);
// 	//ReleaseMutex(mutex_udp_send);
// 	return Num;
// }
long __stdcall UDP_SendData(char*Buffer, unsigned int Length, unsigned char Ip_flg)
{
	int Num;
	if (Buffer != NULL)
	{
		memcpy(&Sendbuffer[0], Buffer, Length);
		Num = sendto(sino_socket, (char*)Sendbuffer, Length, 0, (SOCKADDR*)&(Send_addr[Ip_flg]), sizeof(SOCKADDR));
		if (Num < 0)
		{
			return 0;
		}
		else
		{
			return Num;
		}
	}
}
long __stdcall UDP_PumpSendData(char*Buffer, unsigned char Length, unsigned char Ip_flg)
{
	int Num;
	EnterCriticalSection(&M_lock);
	//WaitForSingleObject(mutex_udp_send,INFINITE);
	Num = 0;
	Sendbuffer[0] = 0xaa;
	Sendbuffer[1] = 0xaa;
	Sendbuffer[2] = (Length / 4);
	Sendbuffer[3] = (Length / 4);
	//printf("send udp data·¢ËÍ\n");
	if (Buffer != NULL)
	{
		memcpy(&Sendbuffer[4], Buffer, Length);
		Num = sendto(sino_socket, (char*)Sendbuffer, Length + 4, 0, (SOCKADDR*)&(Send_addr[Ip_flg]), sizeof(SOCKADDR));
		if (Num<0)
		{
			printf("send udp data fail\n");
			Num = 0;
		}
		else
		{
			//printf("UDP_PumpSendData len=%d\n", Length + 4);
		}
	}
	LeaveCriticalSection(&M_lock);
	//ReleaseMutex(mutex_udp_send);
	return Num;
}

long __stdcall UDP_SetConnetIp(unsigned char Ip_connet)
{
	if (Ip_connet>0 && Ip_connet<255)
	{
		Send_addr[Ip_connet].sin_family = AF_INET;
		Send_addr[Ip_connet].sin_port = htons(5000);
		Send_addr[Ip_connet].sin_addr.S_un.S_addr = (Ip_connet << 24) | 0x01a8c0;
		return SDV_RESULT_OK;
	}

	return SDV_RESULT_CMD_ERR;
}
