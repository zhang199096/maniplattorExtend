// ManipulatorCmd.cpp : 定义控制台应用程序的入口点。
//
//#include "afx.h"
#include "afxtempl.h"
//#include "stdafx.h"
#include "afxinet.h"
//#include <windows.h>
#include "stdio.h"
//#include "afxwin.h"//Cstring类
//#include ".//common//vld.h"
//#include "afx.h"//CFile类
//#include "afxwin.h"//Cstring类
#include "Common.h"
#include "MotionControl/UDP_Export.h"
#include "MotionControl/UDP_Rerro.h"
#include "MotionControl/Clone/DIAControl_Clone.h"

HOMING			Homing;
OVERALLSIGN		OverallSign;		// 总体标志
SYSTEM			System;			//系统参数结构对象
ERROR_Main		Error_main;			//报警结构对象

Int32 DualCore_RxBuf[700];
Int32 DualCore_TxBuf[700];
int Rxlength;



long Recv_Handler(LPVOID arg, void*pCtrl)
{
	Int32 m_buffer[1024];
	unsigned int crc_data, data;
	int Rxlength;
	int packlen;

	UCHAR *pDate = (UCHAR*)arg;

	//DIAserial*pDlg1=(DIAserial*)pCtrl;
	//  CmanipulatorDlg*pDlg1=(CmanipulatorDlg*)pCtrl;
	packlen = *(int*)(pCtrl);

	memcpy(m_buffer, pDate, packlen);
	for (int i = 0; i < (packlen / 4); i++)
	{
		DualCore_RxBuf[i] = htonl(m_buffer[i]);
	}


	OverallSign_Clone.m_ReciveSuccess = false;
	OverallSign_Clone.m_ReciveLostCounter++;

	Rxlength = DualCore_RxBuf[1];

	if ((DualCore_RxBuf[0] == 0xaa) &&
		//(pDlg1->RxBuf[3]==0xb0)&&
		(DualCore_RxBuf[4] == 0x0f) &&
		(DualCore_RxBuf[Rxlength + 3] == 0xbb))
	{
		if (DualCore_RxBuf[3] == IP_Dual_Core)
		{
			crc_data = DualCore_RxBuf[Rxlength + 1];
			data = crc32((Uint32*)(DualCore_RxBuf), Rxlength);

			if (data == crc_data)
			{
#if 0
				SYSTEMTIME st;
				GetLocalTime(&st);
				DWORD mCurrentTick = st.wMilliseconds;
				DWORD mDelta = mCurrentTick - m_Control_Clone.m_ReadTime;
				m_Control_Clone.m_ReadTime = st.wMilliseconds;
				TRACE("DSP_com_BackCMD_b0 time : %d mDelta    = %d - %d\r", mDelta, st.wMilliseconds, m_Control_Clone.m_ReadTime);
#endif

#if 0
				m_Control_Clone.DSP_com_BackCMD_b0();//信息处理
#endif
				//for rapid send B2 packet
				OverallSign_Clone.MicroEDataBaseStartCnt_DSP = DualCore_RxBuf[16];

				OverallSign_Clone.m_ReciveSuccess = true;
				OverallSign_Clone.m_ReciveLostCounter = 0;
				OverallSign_Clone.m_ReciveSuccessCounter++;
			}
			else
			{
				OverallSign_Clone.m_ReciveLostCounter = OverallSign_Clone.m_ReciveLostCounter;
			}
		}
	}
	else
	{
		OverallSign_Clone.m_ReciveLostCounter = OverallSign_Clone.m_ReciveLostCounter;
	}
	return TRUE;
}
int _tmain(int argc, _TCHAR* argv[])
{
	int kl,ret;
	kl = UDP_InitNet();
	//ret = UDP_SetConnetIp(IP_Dual_Core);//20
	if (kl == 0)
	{
		SetStreamHOOK(Recv_Handler, NULL);

		ret = UDP_SetConnetIp(IP_Dual_Core);//20
		//OverallSign.PC_DSP_COM_OK = TRUE;
		OverallSign_Clone.PC_DSP_COM_OK = TRUE;

		
	}
	else
	{
		printf("Net Port Not Connect\n");
	}

	return 0;
}

