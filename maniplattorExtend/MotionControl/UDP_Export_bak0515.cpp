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
//      Ver 1.00    2016.01.05   Initial coding                                            Enjoy_Lu     
//*******************************************************************************************************

#pragma once
//#include "stdafx.h"
#include "stdafx.h"
#include <afx.h>
#include "UDP_Rerro.h"
#include "UDP_Export.h"
#include <WinSock2.h> 
#pragma comment(lib,"ws2_32.lib") 
#include "..//Common.h"

//#include "..//manipulatorDlg_include.h"

CRITICAL_SECTION M_lock;
SOCKET sino_socket;
HANDLE sdv_hThread;
HANDLE sdv_hThread1;
StreamProc gCallBack;
void*pUserData1;
SOCKADDR_IN addrFrom;
SOCKADDR_IN addrscoket5;
SOCKADDR_IN addrFrom1[255];
SOCKADDR_IN Send_addr[255];
SOCKADDR_IN Send_addr_ELMO[255];
char connet_buf[2];
char connet_buf_ELMO_FP1[6];
int test_sock(SOCKADDR_IN kk);

extern Int32 DualCore_RxBuf[700];
extern Int32 DualCore_TxBuf[700];
static DWORD WINAPI UDP_ThreadProc1(void* pCtrl) ;
unsigned char Sendbuffer[2048];
HANDLE	sdv_hTerminate1= CreateEvent(NULL, TRUE, FALSE, NULL);
long __stdcall UDP_InitNet()
{
	WSADATA wsData;
	SOCKADDR_IN addrscoket;
	int retdate;
	WSAStartup(MAKEWORD(2,2),&wsData);
	sino_socket=socket(AF_INET,SOCK_DGRAM,0);
	if(INVALID_SOCKET==sino_socket)
	{
		return SDV_RESULT_SOCK_ERR;
	}
	connet_buf[0]=0xcc;
	connet_buf[1]=0xcc;
	bool opt=true;
	memset(Send_addr,0,sizeof(SOCKADDR_IN));
// 	addrscoket5.sin_addr.S_un.S_addr=inet_addr("192.168.1.191");
// 	addrscoket5.sin_family=AF_INET;
// 	addrscoket5.sin_port=htons(5000);
	addrscoket.sin_addr.S_un.S_addr=htonl(INADDR_ANY);
	addrscoket.sin_family=AF_INET;
	addrscoket.sin_port=htons(5050);
	setsockopt(sino_socket,SOL_SOCKET,SO_BROADCAST,(char FAR*)&opt,sizeof(opt));
	retdate=bind(sino_socket,(SOCKADDR*)&addrscoket,sizeof(SOCKADDR));
	if(retdate==SOCKET_ERROR)
	{
		closesocket(sino_socket);
		return SDV_RESULT_BIND_ERR;
	}
	return SDV_RESULT_OK;
}
/*
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
*/
long  __stdcall SetStreamHOOK(StreamProc lpStreamFun,void*pUserData)
{
	if (lpStreamFun==NULL)
	{
		return SDV_RESULT_HOOK_ERR;
	}
	gCallBack=lpStreamFun;
	pUserData1=pUserData;
	SetEvent(sdv_hTerminate1);
	sdv_hThread=CreateThread(NULL,0,UDP_ThreadProc1,NULL,0,NULL);
	if (sdv_hThread==FALSE)
		return SDV_RESULT_THREAD_ERR;
	else
	{
		CloseHandle(sdv_hThread);
		return  SDV_RESULT_OK;

	}
}

DWORD WINAPI	UDP_ThreadProc1(void* pCtrl)
{
	int Receive_buffer[512];
	int Receive_buffer_htonl[512];

	char Receive_buffer_ELMO[20];
	
	SOCKET sock=sino_socket;
	int lengh=sizeof(SOCKADDR);
	int retval;
	UCHAR m_Ip;
	DWORD        bytes; 
	UCHAR recvBuffer[2048];
	UCHAR mm_buf[2048]={0};
	int recvBufferLength=0;
	INT40AXIS AbsCoordinate_LinearMotor;

	CString str;

	while(WaitForSingleObject(sdv_hTerminate1,INFINITE) == WAIT_OBJECT_0)
	{
		retval=recvfrom(sock,(char*)recvBuffer,1000, 0,(SOCKADDR*)&addrFrom,&lengh);
		if(retval > 0)
		{
			m_Ip=unsigned char((addrFrom.sin_addr.S_un.S_addr)>>24);

			//memcpy(&mm_buf,&recvBuffer,retval);
			//(*gCallBack)(mm_buf,pUserData1);
			
			if(m_Ip == IP_Dual_Core)
			{
				memcpy(&Receive_buffer,&recvBuffer,retval);
				for(int i=0;i<retval/4;i++)
				{
					DualCore_RxBuf[i] = htonl(Receive_buffer[i]);
				}
				//(*gCallBack)(Receive_buffer_htonl,pUserData1);
				(*gCallBack)(DualCore_RxBuf, pUserData1);
			}
		}
	}

	return FALSE;
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
		if (Num < 0)
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
long __stdcall UDP_SendData(char*Buffer,unsigned int Length,unsigned char Ip_flg  )
{
	int Num;
	if (Buffer!=NULL)
	{
		memcpy(&Sendbuffer[0],Buffer,Length);
		Num=sendto(sino_socket,(char*)Sendbuffer,Length,0,(SOCKADDR*)&(Send_addr[Ip_flg]),sizeof(SOCKADDR));
		if (Num<0)
		{
			return 0;
		}
		else
		{
			return Num;
		}
	}
}

long __stdcall UDP_SetConnetIp(unsigned char Ip_connet )
{
	if (Ip_connet>0&&Ip_connet<255)
	{
		Send_addr[Ip_connet].sin_family=AF_INET;
		Send_addr[Ip_connet].sin_port=htons(5000);
		Send_addr[Ip_connet].sin_addr.S_un.S_addr=(Ip_connet<<24)|0x01a8c0;
		return SDV_RESULT_OK;
	}
	
	return SDV_RESULT_CMD_ERR;
}
