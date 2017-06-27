// maniplattorExtend.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
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


extern HANDLE	event_DualCore_Clone_send;

ERROR_Main		Error_main_Clone;			//报警结构对象
HOMING			Homing_Clone;
Int32 DualCore_RxBuf[700];
Int32 DualCore_TxBuf[700];

INT40AXIS MonitorBuffer1[2000], MonitorBuffer2[2000];
Uint32	MonitorBuffer1Cnt = 0;
long Recv_Handler(LPVOID arg, void*pCtrl)
{
	Int32 m_buffer[1024];
	unsigned int crc_data, data;
	int Rxlength;
	int packlen;

	UCHAR *pDate = (UCHAR*)arg;

// 	DSP_com_MainCMD_b0();
// 	Sleep(10);
// 	OverallSign_Clone.packsendflag |= 0x00000001;
// 	SetEvent(event_DualCore_Clone_send);
// 	Sleep(5);
	
// 	if (OverallSign_Clone.PC_DSP_COM_OK)
// 	{
// 
// 		DSP_com_MainCMD_b0();
// 		Sleep(20);
// 	}
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
		if (DualCore_RxBuf[3] ==IP_Dual_Core) //IP_Dual_Core 20
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
				//DualCore_DSP_com_BackCMD_b0();
				DSP_com_BackCMD_b0();

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
	DualCore_UDP_ResendDeal();
	return TRUE;
}

/********Init Net & Para ****
设置板卡网络，建立连接，建立通讯事件
****************************/
void Optical_Panel_Init()
{
	ParameterSave_Read();
	ini_sys_para_Clone();
	Udp_DualCore_init();
	int kl = UDP_InitNet();
	if (kl == 0)
	{
		SetStreamHOOK(Recv_Handler, NULL);

		int ret = UDP_SetConnetIp(IP_Dual_Core);//21
		//ret = UDP_SetConnetIp(IP_Mult_Pump);//186
		//OverallSign.PC_DSP_COM_OK = TRUE;
		OverallSign_Clone.PC_DSP_COM_OK = TRUE;
	}
}
/********Init Net & Para ****
设置通道电压
****************************/
void SetChannelVoltage(int channel, int Volatge)
{
	DACONVERT DA_Convert1;
	DA_Convert1.VotageChannel = channel;
	DA_Convert1.VotageValue = Volatge/2;
	DA_Convert1.AutoSign = 0;
	OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
	OverallSign_Clone.STDSendCount = OverallSign_Clone.STDSendCount;
	OverallSign_Clone.NCSign = STDCODERUN;
	DSPCodePacket_Code_G106(DA_Convert1);
	OverallSign_Clone.packsendflag |= 0x00000004;
	SetEvent(event_DualCore_Clone_send);
	Sleep(10);

	OverallSign_Clone.packsendflag |= 0x00000001;
	SetEvent(event_DualCore_Clone_send);
	printf(" CH3 all homimg\n");
}
/********Init Net & Para ****
复位光学板
****************************/
bool ResetOpticalPannel(void)
{

		OverallSign_Clone.packsendflag |= 0x00000001;
		SetEvent(event_DualCore_Clone_send);

	//OnBnClickedReset();
	DIAControl_DualCore_Clone_Reset();
	Sleep(10);

		OverallSign_Clone.packsendflag |= 0x00000001;
		SetEvent(event_DualCore_Clone_send);

	int cnt = 0;
	while (1)
	{
		if (Error_main_Clone.MainErrorSign == false)
		{
			return true;
		}
		if (cnt > 10)
		{
			return false;
			break;
		}
		cnt++;
		Sleep(10);
		//return TRUE;
	}
	//SetEvent(event_DualCore_Clone_send);
	printf("All reset\n");
}
/********Init Net & Para ****
设置特定波形类型
****************************/
void SenDefultWaveData(Int32 Channel,int WaveType,int HigthVol,int LowVol,Int32 CycleCnt){
	DACONVERT DA_Convert1;
	DA_Convert1.VotageChannel = Channel;
	//DA_Convert1.VotageValue = Volatge;
	DA_Convert1.AutoSign = 2;
	DA_Convert1.Voltage_Max = HigthVol/2;
	DA_Convert1.Voltage_Start = HigthVol/2;
	DA_Convert1.CycleNum = CycleCnt;
	DA_Convert1.WaveSelect = WaveType;
	OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
	OverallSign_Clone.STDSendCount = OverallSign_Clone.STDSendCount;
	OverallSign_Clone.NCSign = STDCODERUN;
	DSPCodePacket_Code_G106(DA_Convert1);
	OverallSign_Clone.packsendflag |= 0x00000004;
	SetEvent(event_DualCore_Clone_send);
	Sleep(10);

	OverallSign_Clone.packsendflag |= 0x00000001;
	SetEvent(event_DualCore_Clone_send);
	printf(" CH3 all homimg\n");
}
void for_SendB0()
{
	OverallSign_Clone.packsendflag |= 0x00000001;
	SetEvent(event_DualCore_Clone_send);
	Sleep(5);
}
/********Init Net & Para ****
传输自定义波形数据并出波形
****************************/
void SenWaveData(Int32 DataCnt[]){
	for (int i = 0; i < 2000; i++)
	{
		MonitorBuffer1[i].Axis19_Clone_Optical_LensUpDown = DataCnt[i];
	}

	OverallSign_Clone.MicroEDataBaseStartCnt = 0;
	OverallSign_Clone.MicroEDataBaseTotalCnt = 2000;
	MonitorBuffer1Cnt = 2000;
	OverallSign_Clone.packsendflag |= 0x00000002;
	SetEvent(event_DualCore_Clone_send);
	Sleep(10);

	OverallSign_Clone.packsendflag |= 0x00000001;
	SetEvent(event_DualCore_Clone_send);
	printf("Test B2\n");
}
int _tmain(int argc, _TCHAR* argv[])
{
	Int32 testData[2000];
	for (int i = 0; i < 2000;i++)
	{
		if (i<1000)
		{
			testData[i] = i;
		}
		else
		{
			testData[i] = 2000 - i;
		}
		
	}
	Optical_Panel_Init();
	char buf[20];
	while (1)
	{
		gets_s(buf);
		if (strcmp(buf, "reset") == 0)
		{
			if (ResetOpticalPannel())
			{
				printf("Reset Ture!!\n");
			}
			else
			{
				printf("Reset Error ,pelease reset again!!!\n");
			}
			//ResetOpticalPannel();
		}
		if (strcmp(buf, "CH5") == 0)
		{
			SetChannelVoltage(5,300);
		}
		else if (strcmp(buf, "B2") == 0)
		{
			SenWaveData(testData);
		}
		else if (strcmp(buf, "CB2") == 0)
		{
			SenDefultWaveData(5,4,700,300,20);
		}
		else if (strcmp(buf, "B0") == 0)
		{
			for_SendB0();
		}
	}
// 	ParameterSave_Read();
// 	ini_sys_para_Clone();
// 	Udp_DualCore_init();
// 	char buf[20];
// 	int kl = UDP_InitNet();
// 	if (kl == 0)
// 	{
// 		SetStreamHOOK(Recv_Handler, NULL);
// 
// 		int ret = UDP_SetConnetIp(IP_Dual_Core);//21
// 		//ret = UDP_SetConnetIp(IP_Mult_Pump);//186
// 		//OverallSign.PC_DSP_COM_OK = TRUE;
// 		OverallSign_Clone.PC_DSP_COM_OK = TRUE;
// 	}
// 	//memcpy(MonitorBuffer1, 0x00, sizeof(MonitorBuffer1));
// 	for (int i = 0; i < 2000;i++)
// 	{
// 		MonitorBuffer1[i].Axis19_Clone_Optical_LensUpDown = i;
// 	}
// 	while (1)
// 	{
// 		
// 		gets_s(buf);
// 		if (strcmp(buf, "CH5") == 0)
// 		{
// 			DACONVERT DA_Convert1;
// 			DA_Convert1.VotageChannel = 5;
// 			DA_Convert1.VotageValue = 200;
// 			DA_Convert1.AutoSign = 0;
// 			DA_Convert1.Voltage_Max = 500;
// 			DA_Convert1.Voltage_Start = 400;
// 			DA_Convert1.CycleNum = 10;
// 			DA_Convert1.RiseTime = 100;
// 			DA_Convert1.FallTime = 100;
// 			DA_Convert1.HighKeepTime = 50;
// 			DA_Convert1.LowKeepTime = 50;
// 			DA_Convert1.PWMPhase = 1000;
// 			OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
// 			OverallSign_Clone.STDSendCount = OverallSign_Clone.STDSendCount;
// 			OverallSign_Clone.NCSign = STDCODERUN;
// 			DSPCodePacket_Code_G106(DA_Convert1);
// 			OverallSign_Clone.packsendflag |= 0x00000004;
// 			SetEvent(event_DualCore_Clone_send);
// 			printf(" CH3 all homimg\n");
// 		}
// 		if (strcmp(buf, "CH4") == 0)
// 		{
// 			DACONVERT DA_Convert1;
// 			DA_Convert1.VotageChannel = 4;
// 			DA_Convert1.VotageValue = 100;
// 			DA_Convert1.AutoSign = 0;
// 			DA_Convert1.Voltage_Max = 500;
// 			DA_Convert1.Voltage_Start = 400;
// 			DA_Convert1.CycleNum = 10;
// 			DA_Convert1.RiseTime = 100;
// 			DA_Convert1.FallTime = 100;
// 			DA_Convert1.HighKeepTime = 50;
// 			DA_Convert1.LowKeepTime = 50;
// 			DA_Convert1.PWMPhase = 1000;
// 			OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
// 			OverallSign_Clone.STDSendCount = OverallSign_Clone.STDSendCount;
// 			OverallSign_Clone.NCSign = STDCODERUN;
// 			DSPCodePacket_Code_G106(DA_Convert1);
// 			OverallSign_Clone.packsendflag |= 0x00000004;
// 			SetEvent(event_DualCore_Clone_send);
// 			printf("CH4 all homimg\n");
// 		}
// 		if (strcmp(buf, "CH5") == 0)
// 		{
// 			DACONVERT DA_Convert1;
// 			DA_Convert1.VotageChannel = 5;
// 			DA_Convert1.VotageValue = 100;
// 			DA_Convert1.AutoSign = 0;
// 			DA_Convert1.Voltage_Max = 500;
// 			DA_Convert1.Voltage_Start = 400;
// 			DA_Convert1.CycleNum = 10;
// 			DA_Convert1.RiseTime = 100;
// 			DA_Convert1.FallTime = 100;
// 			DA_Convert1.HighKeepTime = 50;
// 			DA_Convert1.LowKeepTime = 50;
// 			DA_Convert1.PWMPhase = 1000;
// 			OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
// 			OverallSign_Clone.STDSendCount = OverallSign_Clone.STDSendCount;
// 			OverallSign_Clone.NCSign = STDCODERUN;
// 			DSPCodePacket_Code_G106(DA_Convert1);
// 			OverallSign_Clone.packsendflag |= 0x00000004;
// 			SetEvent(event_DualCore_Clone_send);
// 			printf(" CH5 all homimg\n");
// 		}
// 		if (strcmp(buf, "CH6") == 0)
// 		{
// 			DACONVERT DA_Convert1;
// 			DA_Convert1.VotageChannel = 6;
// 			DA_Convert1.VotageValue = 100;
// 			DA_Convert1.AutoSign = 0;
// 			DA_Convert1.Voltage_Max = 500;
// 			DA_Convert1.Voltage_Start = 400;
// 			DA_Convert1.CycleNum = 10;
// 			DA_Convert1.RiseTime = 100;
// 			DA_Convert1.FallTime = 100;
// 			DA_Convert1.HighKeepTime = 50;
// 			DA_Convert1.LowKeepTime = 50;
// 			DA_Convert1.PWMPhase = 1000;
// 			OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
// 			OverallSign_Clone.STDSendCount = OverallSign_Clone.STDSendCount;
// 			OverallSign_Clone.NCSign = STDCODERUN;
// 			DSPCodePacket_Code_G106(DA_Convert1);
// 			OverallSign_Clone.packsendflag |= 0x00000004;
// 			SetEvent(event_DualCore_Clone_send);
// 			printf(" Ch6 all homimg\n");
// 		}
// 		else if (strcmp(buf, "G113") == 0)
// 		{
// 			OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
// 			OverallSign_Clone.STDSendCount = OverallSign_Clone.STDSendCount;
// 			OverallSign_Clone.NCSign = STDCODERUN;
// 			DSPCodePacket_Code_G113(1,0,500,500,1000);
// 			OverallSign_Clone.packsendflag |= 0x00000004;
// 			SetEvent(event_DualCore_Clone_send);
// 			printf("all stop\n");
// 		}
// 		else if (strcmp(buf, "reset") == 0)
// 		{
// 			OnBnClickedReset();
// 			Sleep(10);
// 			//SetEvent(event_DualCore_Clone_send);
// 			printf("All reset\n");
// 		}
// 		else if (strcmp(buf, "G101") == 0)
// 		{
// 			DSPCodePacket_Code_G101(500,500);
// 			SetEvent(event_DualCore_Clone_send);
// 			printf("PlateTransfering_ClampHand_Close\n");
// 		}
// 		else if (strcmp(buf, "B2") == 0)
// 		{
// 			OverallSign_Clone.MicroEDataBaseStartCnt = 0;
// 			OverallSign_Clone.MicroEDataBaseTotalCnt = 2000;
// 			MonitorBuffer1Cnt = 2000;
// 			OverallSign_Clone.packsendflag |= 0x00000002;
// 			SetEvent(event_DualCore_Clone_send);
// 			printf("Test B2\n");
// 		}
// 		//ForTestB3Cycle();
// 	}
	return 0;
}

