/////file***************************************************************************************************
//                                                                                                       
//                                                                                                       
//        DIAControl_Clone.cpp for  Master                  
//                                                                                                       
//                                                                                                       
//*******************************************************************************************************
//    Please refer to the following document for more detials.                                           
//       "ModuleCommunicationProtocal_DualCore.pdf "                
//       "UsualCommandCodeDescription_B3Packet_DualCore"                        
//                                                                                                       
//******* Copyright(C) 2015 GIBH ******************************************************************
//                                                                                                       
//      Ver 1.00    2015.11.02   Initial coding                                            Enjoy_Lu     
//*******************************************************************************************************
// DIAControl_Clone.cpp : 实现文件
//
#include "stdafx.h"
#include "stdio.h"
#include <stdio.h>
#include <fstream> 
#include <iostream>  
#include <ostream>
//#include <iosfwd.h>
//#include <fstream.h>  
#include <stdlib.h>  
//#include <fiostream.h> 
#include "afx.h"//CFile类
#include "afxwin.h"//Cstring类OVE
#include "..//..//manipulatorDlg_include.h"
#include "DIAControl_Clone.h"
#include "..//..//Common.h"
#include <iostream>  
#include <fstream> 
using namespace std;
//#include "..//MotionControl/UDP_Export.h"
#define SystemPrameterPackageLength_B0 32
#define SystemPrameterPackageLength_B1 492	
#define SystemPrameterPackageLength_B2 210
#define SystemPrameterPackageLength_B3 70	
#define SystemPrameterPackageLength_B4 38

#define MAX_SENDUDPSEQUENCE  300000



GCODE 	DualCore_GCodeBuffer[DUALCORE_STDGCODE_MOD];	//区域计算代码缓冲数组

MAINSTATUSSIGN_REG FPGA_Home_Register_Clone;
MAINSTATUSSIGN_REG FPGA_Coin_Register_Clone;
MAINSTATUSSIGN_REG TrackRunOver_Register_Clone;
MAINSTATUSSIGN_REG SoftLimitPOS_Register_Clone;
MAINSTATUSSIGN_REG SoftLimitNEG_Register_Clone;
MAINSTATUSSIGN_REG InterpolationOver_Register_Clone;
MAINSTATUSSIGN_REG CompensationDataCheck_Register_Clone;
DUALCORE_MAINSTATUSSIGN_REG FPGA_EncoderAlarm_Register_Clone;
DUALCORE_MAINSTATUSSIGN_REG FPGA_ServoAlarm_Register_Clone;
DUALCORE_MAINSTATUSSIGN_REG FPGA_HardLimitPOS_Register_Clone;
DUALCORE_MAINSTATUSSIGN_REG FPGA_HardLimitNEG_Register_Clone;


DUALCORE_MAINSTATUSSIGN_REG MainCommand_Register_Clone;
DUALCORE_MAINSTATUSSIGN_REG MainCommand_Register_Clone2;
DUALCORE_MAINSTATUSSIGN_REG MainStatus_Register_Clone;
DUALCORE_MAINSTATUSSIGN_REG MainStatus_Register_Clone2;

DUALCORE_MAINSTATUSSIGN_REG FPGA_SoftLimitPOS_Register_Clone;
DUALCORE_MAINSTATUSSIGN_REG FPGA_SoftLimitNEG_Register_Clone;
int UDP_DualCore_WaitRespFlag;
HANDLE	event_DualCore_UDP_Rsend;
CRITICAL_SECTION dualcore_udp_lock;

int st_udpdualcore_curms;
int st_udpdualcore_lastms;
int udpdualcore_resend_times;
SYSTEMTIME st_udpdualcore_resend;
DWORD m_lastReadTime;

static DWORD WINAPI UDP_DualCore_Clone_SendThreadProc(void* pCtrl);
HANDLE	thread_DualCore_Clone_send;
HANDLE	event_DualCore_Clone_send;

CString str_Information_Clone;
Int32 CounterForLED;
Int8 LensCompensationSign = 0;
//	DWORD m_lastReadTime;
float64AXIS m_SET_ABS;
LINEARCOMPENSATION 		LinearCompensation;
 MAINCOMMANDSIGN_REG Input_Register_Clone;
 MAINCOMMANDSIGN_REG Input_Register_Clone2;
 MAINCOMMANDSIGN_REG Output_Register_Clone;
 MAINCOMMANDSIGN_REG Output_Register_Clone2;
 DOUBLEJOINTS MACH_COORDINATE_Clone;
 DOUBLEJOINTS ABS_COORDINATE_Clone; //缁濆鍧愭爣
 OVERALLSIGN	OverallSign_Clone;
 SYSTEM			System_Clone;

 Int32 DualCore_ResendTxBuf[700];
 Int32 DualCore_ResendLength;
 extern Int32 DualCore_RxBuf[700];
 extern Int32 DualCore_TxBuf[700];
//	bool m_MotorRun_POSDown;//判断正转按键是否按下
//	bool m_MotorRun_NEGDown;//判断反转按键是否按下

double m_JogSpeed;
double m_JOGDistance;
double m_speedinfo_Set;

//	DWORD m_ReadTime;
int SendUDPSequence_B0 = 0;
int SendUDPSequence_B1 = 0;
int SendUDPSequence_B2 = 0;
int SendUDPSequence_B3 = 0;
int SendUDPSequence_B4 = 0;
int SendUDPSequence_B5 = 0;

int RecvUDPSequence_B0 = 0;
int RecvUDPSequence_B1 = 0;
int RecvUDPSequence_B2 = 0;
int RecvUDPSequence_B3 = 0;
int RecvUDPSequence_B4 = 0;
int RecvUDPSequence_B5 = 0;
int *UUint32Pointer1;
int ParaReadBuf[700];
Int32 SCIA_SendDim[25];
Int32 TxBuf[700];
Int32 RxBuf[700];
//	Int32 RxBuf[700];

//GCODE			SPCGCodeBuffer[SPCGCODE_MOD];	//SPC G-code 结构缓冲数组
GCODE 			GCodeBuffer[STDGCODE_MOD];	//区域计算代码缓冲数组
#define  MicroESendDataSign 1 //0:for test ;1:real run
void ParameterSave_Read()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������??
	/*��ȡ����*/
	int i = 0;
	CString str, str1;
	CStdioFile filesave;
	CString buf[1024]; // ������������

	if (!filesave.Open((_T(".\\MotionControl\\Clone\\Parameter_Clone.txt")), CFile::modeRead))
	{
		//MessageBox(_T("�����ļ����ļ�ʧ��"));
		return;
	}

	while (1)
	{
		CString csText;
		if (!filesave.ReadString(csText))
		{
			break; // ��������ˣ��˳�ѭ��??
		}

		buf[i] = csText; // ��������
		ParaReadBuf[i++] = _wtoi(csText);
	}

	filesave.Close();
}
void ini_sys_para_Clone(void)
{	//need read from database
	int data32;
	Int16 	i;		
	
	UUint32Pointer1 = &ParaReadBuf[7];
	
	System_Clone.Tsample		 = *UUint32Pointer1++;//�岹���� ��λ: 0.1ms //�趨��Χ����5-20������ʼֵ:10
	System_Clone.SlaveMAX				 = *UUint32Pointer1++;//��վ���� ��λ���� //�趨��Χ����0-25������ʼֵ:0
	System_Clone.TrackRunOutRangeSQR	 = *UUint32Pointer1++;//λ��ƫ�� ��λ��mm 	//�趨��Χ����0-100������ʼֵ:10
	System_Clone.acc_SET.Axis1_Clone_Linear_Z = *UUint32Pointer1++;			//��1���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis2_Clone_Linear_A = *UUint32Pointer1++;			//��2���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;			//��3���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;			//��4���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis5 = *UUint32Pointer1++;//��5���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000

	System_Clone.acc_SET.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis13 = *UUint32Pointer1++;//��13���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis14 = *UUint32Pointer1++;//��14���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000

	System_Clone.acc_SET.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis23 = *UUint32Pointer1++;//��23���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis24 = *UUint32Pointer1++;//��24���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.acc_SET.Axis25 = *UUint32Pointer1++;//��25���ٶ�1 ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000

	System_Clone.dec_SET.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis5 = *UUint32Pointer1++;//��5���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000

	System_Clone.dec_SET.Axis13 = *UUint32Pointer1++;//��13���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis14 = *UUint32Pointer1++;//��14���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000

	System_Clone.dec_SET.Axis23 = *UUint32Pointer1++;//��23���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis24 = *UUint32Pointer1++;//��24���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.dec_SET.Axis25 = *UUint32Pointer1++;//��25���ٶ� ��λ��mm/s^2;//�趨��Χ����100-10000������ʼֵ: 4000
	System_Clone.VeerDeltaV			 = *UUint32Pointer1++;//��С�ٶ� ��λ:0.1mm/s ;//�趨��Χ����5-100������ʼֵ:30
	System_Clone.NicetyVeerDeltaV		 = *UUint32Pointer1++;//��׼��С�ٶ�	��λ:0.1mm/s;//�趨��Χ����5-50������ʼֵ:10
	System_Clone.VeerDeltaT				 = *UUint32Pointer1++;//ת�����ʱ�䳣��???��λ:0.1s;//�趨��Χ����5-100������ʼֵ:30
	System_Clone.NicetyVeerDeltaT		 = *UUint32Pointer1++;//��׼ת�����ʱ�䳣��???��λ:0.1s;//�趨��Χ����5-50������ʼֵ:10
	System_Clone.LinearAxisMinUnit		 = *UUint32Pointer1++;//����С���뵥λ ��λnm;//�趨��Χ����50~10000������ʼֵ:1000
	System_Clone.LinearAxisOutUnitEQU	 = *UUint32Pointer1++;//����С������?��λnm/pulse;//�趨��Χ����50~10000������ʼֵ:1000
	System_Clone.G0Speed				 = *UUint32Pointer1++;//���ٽ����ٶ�1 ��λmm/s;//�趨��Χ����10~2000������ʼֵ:100

	System_Clone.G0Speed_2			 = *UUint32Pointer1++;//���ٽ����ٶ�2 ��λmm/s;//�趨��Χ����10~2000������ʼֵ:100
	System_Clone.G1Speed				 = *UUint32Pointer1++;//���������ٶ�1 ��λmm/s;//�趨��Χ����1~1000������ʼֵ:10
	System_Clone.G1Speed_2			 = *UUint32Pointer1++;//���������ٶ�2 ��λmm/s;//�趨��Χ����1~1000������ʼֵ:10
	System_Clone.SRefSpeed				 = *UUint32Pointer1++;//ԭ�㸴���ٶ�1 ��λmm/s;//�趨��Χ����1~50������ʼֵ:20
	System_Clone.SRefSpeedBack			 = *UUint32Pointer1++;//ԭ�㸴������ٶ�??? ��λmm/s	//�趨��Χ����1~5������ʼֵ:1
	System_Clone.SRefBack				 = *UUint32Pointer1++;//ԭ�㸴����˾���??? ��λmm;//�趨��Χ����5~30������ʼֵ:15
	System_Clone.SRefSpeed_2			 = *UUint32Pointer1++;//ԭ�㸴���ٶ�2 ��λmm/s;//�趨��Χ����1~50������ʼֵ:20
	System_Clone.SRefSpeedBack_2		 = *UUint32Pointer1++;//ԭ�㸴������ٶ�??? ��λmm/s	//�趨��Χ����1~5������ʼֵ:1
	System_Clone.SRefBack_2			 = *UUint32Pointer1++;//ԭ�㸴����˾���??? ��λmm;//�趨��Χ����5~30������ʼֵ:15
	System_Clone.REFStopVariable.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis5 = *UUint32Pointer1++;//��5ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis13 = *UUint32Pointer1++;//��13ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis14 = *UUint32Pointer1++;//��14ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis23 = *UUint32Pointer1++;//��23ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis24 = *UUint32Pointer1++;//��24ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.REFStopVariable.Axis25 = *UUint32Pointer1++;//��25ԭ�㸴��ֹͣλ��У����?0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:0

	System_Clone.RefDir.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0	
	System_Clone.RefDir.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0	
	System_Clone.RefDir.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis5 = *UUint32Pointer1++;//��5ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0	
	System_Clone.RefDir.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9ԭ�㸴�鷽������ 0��������1��������	;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis13 = *UUint32Pointer1++;//��13ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0	
	System_Clone.RefDir.Axis14 = *UUint32Pointer1++;//��14ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0	
	System_Clone.RefDir.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis23 = *UUint32Pointer1++;//��23ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0	
	System_Clone.RefDir.Axis24 = *UUint32Pointer1++;//��24ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0
	System_Clone.RefDir.Axis25 = *UUint32Pointer1++;//��25ԭ�㸴�鷽������ 0��������1�������� 	//�趨��Χ����0,1������ʼֵ:0

	System_Clone.EncoderCheckChoose.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis5 = *UUint32Pointer1++;//��5������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9������/��դ�߼���־ 0����У�飻1��У��	;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis13 = *UUint32Pointer1++;//��13������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis14 = *UUint32Pointer1++;//��14������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis23 = *UUint32Pointer1++;//��23������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis24 = *UUint32Pointer1++;//��24������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1
	System_Clone.EncoderCheckChoose.Axis25 = *UUint32Pointer1++;//��25������/��դ�߼���־ 0����У�飻1��У��;//�趨��Χ����0,1������ʼֵ:1

	System_Clone.MotorChangeDir.Axis1_Clone_Linear_Z = *UUint32Pointer1++;// ��1�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis2_Clone_Linear_A = *UUint32Pointer1++;// ��2�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;// ��3�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;// ��4�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis5 = *UUint32Pointer1++;// ��5�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;// ��6�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;// ��7�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;// ��8�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;// ��9�������� 0���л���1���л� ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis10_Plate_Transfer_A = *UUint32Pointer1++;// ��10�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;// ��11�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;// ��12�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis13 = *UUint32Pointer1++;// ��13�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis14 = *UUint32Pointer1++;// ��14�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis15_Clone_Linear_X = *UUint32Pointer1++;// ��15�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis16_Clone_Linear_Y = *UUint32Pointer1++;// ��16�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis17_Clone_Optical_X = *UUint32Pointer1++;// ��17�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis18_Clone_Optical_Y = *UUint32Pointer1++;// ��18�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;// ��19�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;// ��20�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis21_Plate_Transfer_X = *UUint32Pointer1++;// ��21�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;// ��22�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis23 = *UUint32Pointer1++;// ��23�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis24 = *UUint32Pointer1++;// ��24�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.MotorChangeDir.Axis25 = *UUint32Pointer1++;// ��25�������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0

	System_Clone.EncoderRDDir.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis5 = *UUint32Pointer1++;//��5������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9������/��դ�߶������� 0���л���1���л�  ;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis13 = *UUint32Pointer1++;//��13������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis14 = *UUint32Pointer1++;//��14������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis23 = *UUint32Pointer1++;//��23������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis24 = *UUint32Pointer1++;//��24������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0
	System_Clone.EncoderRDDir.Axis25 = *UUint32Pointer1++;//��25������/��դ�߶������� 0���л���1���л�;//�趨��Χ����0,1������ʼֵ:0

	System_Clone.AxisResolution.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis5 = *UUint32Pointer1++;//��5������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9������/��դ�߷ֱ��� ��λ��nm;//�趨��Χ����50,20000������ʼֵ:1000
	System_Clone.AxisResolution.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:
	System_Clone.AxisResolution.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:
	System_Clone.AxisResolution.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:
	System_Clone.AxisResolution.Axis13 = *UUint32Pointer1++;//��13������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:
	System_Clone.AxisResolution.Axis14 = *UUint32Pointer1++;//��14������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:
	System_Clone.AxisResolution.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1250:
	System_Clone.AxisResolution.Axis23 = *UUint32Pointer1++;//��23������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:
	System_Clone.AxisResolution.Axis24 = *UUint32Pointer1++;//��24������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:
	System_Clone.AxisResolution.Axis25 = *UUint32Pointer1++;//��25������/��դ�߷ֱ��� ��λ��nm ;//�趨��Χ����50,20000������ʼֵ:1000:

	System_Clone.CoordORG.Axis1_Clone_Linear_Z = *UUint32Pointer1++;// ��1��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis2_Clone_Linear_A = *UUint32Pointer1++;// ��2��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;// ��3��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;// ��4��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis5 = *UUint32Pointer1++;// ��5��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;// ��6��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;// ��7��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;// ��8��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;// ��9��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.CoordORG.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis13 = *UUint32Pointer1++;//��13��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis14 = *UUint32Pointer1++;//��14��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis23 = *UUint32Pointer1++;//��23��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis24 = *UUint32Pointer1++;//��24��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.CoordORG.Axis25 = *UUint32Pointer1++;//��25��������ԭ�� ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0:

	System_Clone.OffsetCoordinate.Axis1_Clone_Linear_Z = *UUint32Pointer1++;// ��1ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis2_Clone_Linear_A = *UUint32Pointer1++;// ��2ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;// ��3ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;// ��4ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis5 = *UUint32Pointer1++;// ��5ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;// ��6ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;// ��7ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;// ��8ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;// ��9ƫ������ ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.OffsetCoordinate.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis13 = *UUint32Pointer1++;//��13ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis14 = *UUint32Pointer1++;//��14ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis23 = *UUint32Pointer1++;//��23ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis24 = *UUint32Pointer1++;//��24ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.OffsetCoordinate.Axis25 = *UUint32Pointer1++;//��25ƫ������ ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:

	System_Clone.SafeCoordinate.Axis1_Clone_Linear_Z = *UUint32Pointer1++;// ��1��ȫλ������ ��λ����С���뵥λ	//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis2_Clone_Linear_A = *UUint32Pointer1++;// ��2��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;// ��3��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;// ��4��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis5 = *UUint32Pointer1++;// ��5��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;// ��6��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;// ��7��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;// ��8��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;// ��9��ȫλ������ ��λ����С���뵥λ//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.SafeCoordinate.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis13 = *UUint32Pointer1++;//��13��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis14 = *UUint32Pointer1++;//��14��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis23 = *UUint32Pointer1++;//��23��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis24 = *UUint32Pointer1++;//��24��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.SafeCoordinate.Axis25 = *UUint32Pointer1++;//��25��ȫλ������ ��λ����С���뵥λ�趨��Χ����-1000000,1000000������ʼֵ:0:

	System_Clone.BackCoordinate.Axis1_Clone_Linear_Z = *UUint32Pointer1++;// ��1�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis2_Clone_Linear_A = *UUint32Pointer1++;// ��2�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;// ��3�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;// ��4�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis5 = *UUint32Pointer1++;// ��5�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;// ��6�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;// ��7�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;// ��8�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;// ��9�������� ��λ����С���뵥λ ;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.BackCoordinate.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis13 = *UUint32Pointer1++;//��13�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis14 = *UUint32Pointer1++;//��14�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis23 = *UUint32Pointer1++;//��23�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis24 = *UUint32Pointer1++;//��24�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.BackCoordinate.Axis25 = *UUint32Pointer1++;//��25�������� ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:

	System_Clone.PositionCoordinate1.Axis1_Clone_Linear_Z = *UUint32Pointer1++;// ��1��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis2_Clone_Linear_A = *UUint32Pointer1++;// ��2��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;// ��3��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;// ��4��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis5 = *UUint32Pointer1++;// ��5��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;// ��6��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;// ��7��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;// ��8��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;// ��9��λ����1(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate1.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis13 = *UUint32Pointer1++;//��13��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis14 = *UUint32Pointer1++;//��14��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis23 = *UUint32Pointer1++;//��23��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis24 = *UUint32Pointer1++;//��24��λ����1(������)  ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate1.Axis25 = *UUint32Pointer1++;//��25��λ����1(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:

	System_Clone.PositionCoordinate2.Axis1_Clone_Linear_Z = *UUint32Pointer1++;// ��1��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis2_Clone_Linear_A = *UUint32Pointer1++;// ��2��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;// ��3��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;// ��4��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis5 = *UUint32Pointer1++;// ��5��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;// ��6��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;// ��7��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;// ��8��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;// ��9��λ����2(������) ��λ����С���뵥λ	;//�趨��Χ����-1000000,1000000������ʼֵ:0
	System_Clone.PositionCoordinate2.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11��λ����2(������) ��λ����С����?��?;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis13 = *UUint32Pointer1++;//��13��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis14 = *UUint32Pointer1++;//��14��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis23 = *UUint32Pointer1++;//��23��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis24 = *UUint32Pointer1++;//��24��λ����2(������)  ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:
	System_Clone.PositionCoordinate2.Axis25 = *UUint32Pointer1++;//��25��λ����2(������) ��λ����С���뵥λ;//�趨��Χ����-1000000,1000000������ʼֵ:0:

	System_Clone.MAXSpeed.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis5 = *UUint32Pointer1++;//��5�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9�������ٶ� ��λ��mm/s	;//�趨��Χ����10,2000������ʼֵ:100
	System_Clone.MAXSpeed.Axis10_Plate_Transfer_A= *UUint32Pointer1++ ;//��10�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis11_PlateWaste_Source_X= *UUint32Pointer1++ ;//��11�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis12_PlateWaste_Source_Y= *UUint32Pointer1++ ;//��12�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis13= *UUint32Pointer1++ ;//��13�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis14= *UUint32Pointer1++ ;//��14�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis15_Clone_Linear_X= *UUint32Pointer1++ ;//��15�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis16_Clone_Linear_Y= *UUint32Pointer1++ ;//��16�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis17_Clone_Optical_X= *UUint32Pointer1++ ;//��17�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis18_Clone_Optical_Y= *UUint32Pointer1++ ;//��18�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis19_Clone_Optical_LensUpDown= *UUint32Pointer1++ ;//��19�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis20_PlateEmpty_Destination_Z2_L= *UUint32Pointer1++ ;//��20�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis21_Plate_Transfer_X= *UUint32Pointer1++ ;//��21�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis22_Plate_Transfer_Y= *UUint32Pointer1++ ;//��22�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis23= *UUint32Pointer1++ ;//��23�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis24= *UUint32Pointer1++ ;//��24�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:
	System_Clone.MAXSpeed.Axis25= *UUint32Pointer1++ ;//��25�������ٶ� ��λ��mm/s;//�趨��Χ����10,2000������ʼֵ:100:

	System_Clone.REFStopPosition.Axis1_Clone_Linear_Z = *UUint32Pointer1++;//��1ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis2_Clone_Linear_A = *UUint32Pointer1++;//��2ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis3_Clone_Optical_LensTrans = *UUint32Pointer1++;//��3ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis4_Clone_Optical_FilterTrans = *UUint32Pointer1++;//��4ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis5 = *UUint32Pointer1++;//��5ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis6_PlateEmpty_Destination_Z1_R = *UUint32Pointer1++;//��6ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis7_PlateEmpty_Destination_Y = *UUint32Pointer1++;//��7ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis8_PlateEmpty_Destination_X = *UUint32Pointer1++;//��8ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis9_Plate_Transfer_Z = *UUint32Pointer1++;//��9ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ	;//�趨��Χ����-30000,30000������ʼֵ:0
	System_Clone.REFStopPosition.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis13 = *UUint32Pointer1++;//��13ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis14 = *UUint32Pointer1++;//��14ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis23 = *UUint32Pointer1++;//��23ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis24 = *UUint32Pointer1++;//��24ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 
	System_Clone.REFStopPosition.Axis25 = *UUint32Pointer1++;//��25ԭ�㸴��ֹͣλ�� ��λ����С���뵥λ;//�趨��Χ����-30000,30000������ʼֵ:0: 

	System_Clone.SLimitPos.Axis1_Clone_Linear_Z =   *UUint32Pointer1++;//��1��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis2_Clone_Linear_A =   *UUint32Pointer1++;//��2��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis3_Clone_Optical_LensTrans =   *UUint32Pointer1++;//��3��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis4_Clone_Optical_FilterTrans =   *UUint32Pointer1++;//��4��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis5 =   *UUint32Pointer1++;//��5��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis6_PlateEmpty_Destination_Z1_R =   *UUint32Pointer1++;//��6��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis7_PlateEmpty_Destination_Y =   *UUint32Pointer1++;//��7��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis8_PlateEmpty_Destination_X =   *UUint32Pointer1++;//��8��������λ ��λ����С���뵥λ	;//�趨��Χ����0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis9_Plate_Transfer_Z =   *UUint32Pointer1++;//��9��������λ ��λ����С����?��?	;//�??趨��Χ����??0,1000000������ʼֵ:1000000
	System_Clone.SLimitPos.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis13 = *UUint32Pointer1++;//��13��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis14 = *UUint32Pointer1++;//��14��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis23 = *UUint32Pointer1++;//��23��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis24 = *UUint32Pointer1++;//��24��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 
	System_Clone.SLimitPos.Axis25 = *UUint32Pointer1++;//��25��������λ ��λ����С���뵥λ 	//�趨��Χ����0,1000000������ʼֵ:1000000: 

	System_Clone.SLimitNeg.Axis1_Clone_Linear_Z =   *UUint32Pointer1++;//��1��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis2_Clone_Linear_A =   *UUint32Pointer1++;//��2��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis3_Clone_Optical_LensTrans =   *UUint32Pointer1++;//��3��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis4_Clone_Optical_FilterTrans =   *UUint32Pointer1++;//��4��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis5 =   *UUint32Pointer1++;//��5��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis6_PlateEmpty_Destination_Z1_R =   *UUint32Pointer1++;//��6��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis7_PlateEmpty_Destination_Y =   *UUint32Pointer1++;//��7��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis8_PlateEmpty_Destination_X =   *UUint32Pointer1++;//��8��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis9_Plate_Transfer_Z =   *UUint32Pointer1++;//��9��������λ ��λ����С���뵥λ	;//�趨��Χ����-1000000,0������ʼֵ:-1000000
	System_Clone.SLimitNeg.Axis10_Plate_Transfer_A = *UUint32Pointer1++;//��10��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis11_PlateWaste_Source_X = *UUint32Pointer1++;//��11��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis12_PlateWaste_Source_Y = *UUint32Pointer1++;//��12��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis13 = *UUint32Pointer1++;//��13��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis14 = *UUint32Pointer1++;//��14��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis15_Clone_Linear_X = *UUint32Pointer1++;//��15��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis16_Clone_Linear_Y = *UUint32Pointer1++;//��16��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis17_Clone_Optical_X = *UUint32Pointer1++;//��17��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis18_Clone_Optical_Y = *UUint32Pointer1++;//��18��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis19_Clone_Optical_LensUpDown = *UUint32Pointer1++;//��19��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis20_PlateEmpty_Destination_Z2_L = *UUint32Pointer1++;//��20��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis21_Plate_Transfer_X = *UUint32Pointer1++;//��21��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis22_Plate_Transfer_Y = *UUint32Pointer1++;//��22��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis23 = *UUint32Pointer1++;//��23��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis24 = *UUint32Pointer1++;//��24��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 
	System_Clone.SLimitNeg.Axis25 = *UUint32Pointer1++;//��25��������λ ��λ����С���뵥λ 	//�趨��Χ����-1000000,0������ʼֵ:-1000000: 

	System_Clone.FunctionSelect01 = *UUint32Pointer1++;// ����ѡ��1	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect02 = *UUint32Pointer1++;// ����ѡ��2	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect03 = *UUint32Pointer1++;// ����ѡ��3	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect04 = *UUint32Pointer1++;// ����ѡ��4	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect05 = *UUint32Pointer1++;// ����ѡ��5	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect06 = *UUint32Pointer1++;// ����ѡ��6	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect07 = *UUint32Pointer1++;// ����ѡ��7	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect08 = *UUint32Pointer1++;// ����ѡ��8	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect09 = *UUint32Pointer1++;// ����ѡ��9	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:
	System_Clone.FunctionSelect10 = *UUint32Pointer1++;// ����ѡ��10	//�趨��Χ����0,0xFFFFFFFF������ʼֵ:0:

	System_Clone.LEDFlashTime1 = *UUint32Pointer1++;//added 20150112
	System_Clone.LEDFlashTime2 = *UUint32Pointer1++;//added 20150112
	UUint32Pointer1++; 	
	UUint32Pointer1++; 	
	UUint32Pointer1++; 
	UUint32Pointer1++;
	UUint32Pointer1++;	
	
}






/*
%a,%A 读入�??个浮点�??(仅C99有效) �??�??
%c 读入�??个字�?? �??�??
%d 读入十进制整�?? �??�??
%i 读入十进制，八进制，十六进制整数 �??�??
%o 读入八进制整�?? �??�??
%x,%X 读入十六进制整数 �??�??
%s 读入�??个字符串，遇空格、制表符或换行符结束�?? �??�??
%f,%F,%e,%E,%g,%G 用来输入实数，可以用小数形式或指数形式输入�?? �??�??
%p 读入�??个指�?? �??�??
%u 读入�??个无符号十进制整�?? �??�??
%n 至此已读入�?�的等价字符�?? �??�??
%[] 扫描字符集合 �??�??
%% �??%符号
*/




/*  Function: ����д          */
// void ParameterSave_Write()
// {
// 	// TODO: �ڴ���ӿؼ�֪ͨ����������??
// 	/*�����������ڵ�ǰ������*/
// 	int i;
// 	TCHAR szFilters[]= _T("MyType Files (*.txt)|*.txt|All Files (*.*)|*.*||");
// 	//CFileDialog fileDlg(FALSE, _T(""), _T(""),OFN_FILEMUSTEXIST | OFN_HIDEREADONLY, szFilters);
// 	CString pathName,filename, str,str1;
// 	CStdioFile filesave;
// 
// 	//if(fileDlg.DoModal() == IDOK)
// 	{
// 		//pathName= fileDlg.GetPathName(_T("d:\\Parameter_Clone.txt"));
// 		//filename = fileDlg.GetFileName();
// 		
// 		//if( !filesave.Open( pathName , CFile::modeCreate | CFile::modeWrite) )
// 		//if( !filesave.Open( (_T("d:\\Parameter_Clone.txt")) , CFile::modeCreate | CFile::modeReadWrite) )
// 		if( !filesave.Open( (_T(".\\MotionControl\\Clone\\Parameter_Clone.txt")) , CFile::modeCreate | CFile::modeReadWrite) )
// 		{
// 			//MessageBox(_T("�����ļ����ļ�ʧ��"));
// 			return;
// 		}
// 		
// 		//for( i=0; i<612; i++ )
// 		for( i=0; i<1000; i++ )//need change while save parameter 2016.01.08
// 		{
// 			/*
// 			str1.Empty();
// 			str.Format(_T("parameter number:%d"), i);
// 			str1 = str1+str;
// 			str.Format(_T("      parameter value:%d"), TxBuf[i]);
// 			str1 = str1+str;
// 			filesave.WriteString(str1);
// 			*/
// 			//str.Format(_T("%d"), TxBuf[i]);
//                     str.Format(_T("%d"), ParaWriteBuf[i]);
//                     
// 			filesave.WriteString(str);			
// 			filesave.WriteString( _T("\n") );
// 		}
// 		filesave.Close();
// 		//MessageBox(_T("����ɹ�??!"));
// 	}
// }


int ParaCheckLinearCompensation(Uint16 AxisCount,Int32 Linear_Buf[PARABUF_LINEARSUB],Int32 Laser_Buf[PARABUF_LINEARSUB])
{
	int j=0;	
	
	//ParaCheck_Linear
	if(AxisCount < 2)
	{
		////MessageBox(_T("Axis Linear Compensation buffer Error"));
		return FALSE;
	}
	
	for(j=0;j<AxisCount;j++)
	{
		if(abs(Linear_Buf[j]-Laser_Buf[j]) > Linear_DeltaValue)
		{
			////MessageBox(_T("Axis Linear Compensation Delta Value too large"));
			return FALSE;
		}
	}
	
	if(Linear_Buf[0]<Linear_Buf[1])
	{	//positive compensation
		for(j=0;j<(AxisCount-1);j++)
		{
			if(Linear_Buf[j]>=Linear_Buf[j+1])
			{
			//	//MessageBox(_T("Axis Linear Compensation buffer Error"));
				return FALSE;
			}
			if(Laser_Buf[j]>=Laser_Buf[j+1])
			{
				////MessageBox(_T("Axis Linear Compensation buffer Error"));
				return FALSE;
			}			
		}
	}
	else
	{	//negative compensation
		for(j=0;j<(AxisCount-1);j++)
		{
			if(Linear_Buf[j]<=Linear_Buf[j+1])
			{
				//MessageBox(_T("Axis Linear Compensation buffer Error"));
				return FALSE;
			}
			if(Laser_Buf[j]<=Laser_Buf[j+1])
			{
				//MessageBox(_T("Axis Linear Compensation buffer Error"));
				return FALSE;
			}			
		}				
	}

}




unsigned short crc16(unsigned char *data, unsigned short length)
{
	unsigned short i;
	unsigned short crc_result=0xffff;
	while(length--)
	{
		crc_result ^= *data++;
		for(i=0;i<8;i++)
		{
			if(crc_result & 0x01)
			{
				crc_result = (crc_result>>1) ^ 0xa001;
			}
			else
			{
				crc_result = crc_result>>1;
			}			
		}
	}
	return (crc_result);		
}

unsigned int crc32(unsigned int *data, unsigned short length)
{
	unsigned short i;
	unsigned int crc_result=0xffffffff;
	while(length--)
	{
		crc_result ^= *data++;
		for(i=0;i<8;i++)
		{
			if(crc_result & 0x0001)
			{
				crc_result = (crc_result>>1) ^ 0xa0000001;
			}
			else
			{
				crc_result = crc_result>>1;
			}			
		}
	}
	return (crc_result);		
}

/***************************************************************************/
/*  Function name: JOGModle_DSP()                                   */
/*  Argument:NO     	                                    */
/*  Return value:No/
/*  Function: JOGModle_DSP...��λ�����ֶ�������ز�������λ�����˶�??                                           */
/* 	Module JOG Modle ,control motion at DSP 				*/
/***************************************************************************/

/***************************************************************************/
/*  Function name: DSP_com_BackCMD_b0()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: DSP_com_BackCMD_b0                                           */
/*	DSP 	Back	 Command B0 */
/*	Common data packet ,always connect ,com. DSP -> PC*/
/***************************************************************************/
int DSP_com_BackCMD_b0(void)
{
	unsigned int temp, temp1;
	double xvalue, yvalue, zvalue;
	CString str, str1;

	short i;
	int data32;
	int *UUint32Pointer1;
	//UUint32Pointer1 =(unsigned int *) &RxBuf[7];
	UUint32Pointer1 = &DualCore_RxBuf[7];

	//版本日期返回 
	str.Empty();
	temp = *UUint32Pointer1++;
	str.Format(_T("%2d"), temp);
	str1 = _T("") + str + _T(".");

	temp = *UUint32Pointer1++;
	str.Format(_T("%2d"), temp);
	str1 = str1 + str + _T(".");

	temp = *UUint32Pointer1++;
	str.Format(_T("%2d"), temp);
	str1 = str1 + str + _T(".");


	temp = *UUint32Pointer1++;
	str.Format(_T("%2d"), temp);
	str1 = str1 + str + _T(".");
	str = str1;
	//GetDlgItem(IDC_Clone_DSP_Version)->SetWindowText(str);

	temp = *UUint32Pointer1++;
	temp1 = temp & 0x03;
	if (temp1 == 1)
	{
		OverallSign_Clone.NCSign_bak = STDCODERUN;
	}
	else if (temp1 == 2)
	{
		OverallSign_Clone.NCSign_bak = SPCCODERUN;
	}
	else
	{
		OverallSign_Clone.NCSign_bak = NOCODERUN;
	}

	OverallSign_Clone.STDReceiveDSPRunCount = *UUint32Pointer1++;// 1
	OverallSign_Clone.SPCReceiveDSPRunCount = *UUint32Pointer1++;// 2
	if ((OverallSign_Clone.m_OperationMode == JOG) || (OverallSign_Clone.m_OperationMode == REFERENCE))
	{
		OverallSign_Clone.SPCSendCount = OverallSign_Clone.SPCReceiveDSPRunCount;
		OverallSign_Clone.SPCReceiveDSPSendCount = OverallSign_Clone.SPCReceiveDSPRunCount;
	}

	OverallSign_Clone.STDReceiveDSPSendCount = *UUint32Pointer1++;// 3
	OverallSign_Clone.SPCReceiveDSPSendCount = *UUint32Pointer1++;// 4

	//OverallSign.MicroEDataBaseStartCnt
	OverallSign_Clone.MicroEDataBaseStartCnt_DSP = *UUint32Pointer1++;

	UUint32Pointer1++;//Feedrate

	data32 = 0;
	data32 = *UUint32Pointer1++;	//Feedrate.Axis1
	str.Empty();
	str.Format(_T("%d"), data32);
	//GetDlgItem(IDC_Clone_speedinfo_Real)->SetWindowText(str);//速度

	if (OverallSign_Clone.m_OperationMode == REFERENCE)
	{
		//GetDlgItem(IDC_Clone_speedinfo_Set)->SetWindowText(str);//速度
	}
	//Feedrate.Axis2~25 as follows
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;

	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;

	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;
	UUint32Pointer1++;

	ABS_COORDINATE_Clone.Axis1_Clone_Linear_Z = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis2_Clone_Linear_A = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	if (abs(ABS_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans - 128.660) > 0.1)
	{
		ABS_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans = ABS_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans;
	}
	ABS_COORDINATE_Clone.Axis5 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis9_Plate_Transfer_Z = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis10_Plate_Transfer_A = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate

	ABS_COORDINATE_Clone.Axis11_PlateWaste_Source_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis12_PlateWaste_Source_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis13 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis14 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis15_Clone_Linear_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis16_Clone_Linear_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis17_Clone_Optical_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis18_Clone_Optical_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate

	ABS_COORDINATE_Clone.Axis21_Plate_Transfer_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis22_Plate_Transfer_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis23 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis24 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis25 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate

	//各轴机床坐标
	MACH_COORDINATE_Clone.Axis1_Clone_Linear_Z = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis2_Clone_Linear_A = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis5 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis9_Plate_Transfer_Z = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis10_Plate_Transfer_A = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate

	MACH_COORDINATE_Clone.Axis11_PlateWaste_Source_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis12_PlateWaste_Source_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis13 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis14 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis15_Clone_Linear_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis16_Clone_Linear_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis17_Clone_Optical_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis18_Clone_Optical_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate

	MACH_COORDINATE_Clone.Axis21_Plate_Transfer_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis22_Plate_Transfer_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis23 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis24 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis25 = (*UUint32Pointer1++);//PMT1 Value

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis1_Clone_Linear_Z);
	//GetDlgItem(IDC_Clone_POS_MachineAxis1)->SetWindowText(str);

	//str.Format(_T("%.3f"),(MACH_COORDINATE_Clone.Axis1-System_Clone.PositionCoordinate1.Axis1/1000.0));
	////m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis1)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis2_Clone_Linear_A);
	//GetDlgItem(IDC_Clone_POS_MachineAxis2)->SetWindowText(str);

	//str.Format(_T("%.3f"),(MACH_COORDINATE_Clone.Axis2-System_Clone.PositionCoordinate1.Axis2/1000.0));
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis2)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans);
	//GetDlgItem(IDC_Clone_POS_MachineAxis3)->SetWindowText(str);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis3)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans);
	//GetDlgItem(IDC_Clone_POS_MachineAxis4)->SetWindowText(str);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis4)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis5);
	//GetDlgItem(IDC_Clone_POS_MachineAxis5)->SetWindowText(str);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis5)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R);
	////GetDlgItem(IDC_Clone_POS_MachineAxis6)->SetWindowText(str);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis6)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y);
	//GetDlgItem(IDC_Clone_POS_MachineAxis7)->SetWindowText(str);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis7)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X);
	//GetDlgItem(IDC_Clone_POS_MachineAxis8)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis8_PlateEmpty_Destination_X)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis9_Plate_Transfer_Z);
	//GetDlgItem(IDC_Clone_POS_MachineAxis9)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis9)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis10_Plate_Transfer_A);
	//GetDlgItem(IDC_Clone_POS_MachineAxis10)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis10)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis11_PlateWaste_Source_X);
	//GetDlgItem(IDC_Clone_POS_MachineAxis11)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis11)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis12_PlateWaste_Source_Y);
	//GetDlgItem(IDC_Clone_POS_MachineAxis12)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis12)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis13);
	//GetDlgItem(IDC_Clone_POS_MachineAxis13)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis13)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis14);
	//GetDlgItem(IDC_Clone_POS_MachineAxis14)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis14)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis15_Clone_Linear_X);
	//GetDlgItem(IDC_Clone_POS_MachineAxis15)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis15)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis16_Clone_Linear_Y);
	//GetDlgItem(IDC_Clone_POS_MachineAxis16)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis16)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis17_Clone_Optical_X);
	//GetDlgItem(IDC_Clone_POS_MachineAxis17)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis17)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis18_Clone_Optical_Y);
	//GetDlgItem(IDC_Clone_POS_MachineAxis18)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis18)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown);
	//GetDlgItem(IDC_Clone_POS_MachineAxis19)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis19)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L);
	//GetDlgItem(IDC_Clone_POS_MachineAxis20)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis20)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis21_Plate_Transfer_X);
	//GetDlgItem(IDC_Clone_POS_MachineAxis21)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis21)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis22_Plate_Transfer_Y);
	//GetDlgItem(IDC_Clone_POS_MachineAxis22)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis22)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis23);
	//GetDlgItem(IDC_Clone_POS_MachineAxis23)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis23)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis24);
	//GetDlgItem(IDC_Clone_POS_MachineAxis24)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis24)->SetWindowText(str);

	str.Format(_T("%.3f"), MACH_COORDINATE_Clone.Axis25);
	//GetDlgItem(IDC_Clone_POS_MachineAxis25)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_POS_MachineAxis25)->SetWindowText(str);


	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis1_Clone_Linear_Z);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis1)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis2_Clone_Linear_A);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis2)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis3)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis4)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis5);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis5)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis6)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis7)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis8)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis9_Plate_Transfer_Z);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis9)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis10_Plate_Transfer_A);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis10)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis11_PlateWaste_Source_X);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis11)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis12_PlateWaste_Source_Y);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis12)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis13);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis13)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis14);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis14)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis15_Clone_Linear_X);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis15)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis16_Clone_Linear_Y);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis16)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis17_Clone_Optical_X);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis17)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis18_Clone_Optical_Y);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis18)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis19)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis20)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis21_Plate_Transfer_X);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis21)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis22_Plate_Transfer_Y);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis22)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis23);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis23)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis24);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis24)->SetWindowText(str);

	str.Format(_T("%.3f"), ABS_COORDINATE_Clone.Axis25);
	//m_Control_MainInterface.GetDlgItem(IDC_Clone_POS_MachineAxis25)->SetWindowText(str);


	//added 20150408 for test COM. time
	SYSTEMTIME ComTime;
	GetLocalTime(&ComTime);
	DWORD mCurrentTick = ComTime.wMilliseconds;
	DWORD mDelta = mCurrentTick - m_lastReadTime;

	//TRACE("NowValue:%f , Elapsed time : %d milliseconds    = %d - %d\r",ABS_COORDINATE_Clone.Axis1,mDelta,mCurrentTick , m_lastReadTime);
	m_lastReadTime = ComTime.wMilliseconds;

	if ((*UUint32Pointer1) == 0x01)
	{
		OverallSign_Clone.CanSendCodeSign_B4 = true;
	}
	else
	{
		OverallSign_Clone.CanSendCodeSign_B4 = false;
	}
	UUint32Pointer1++;

	if ((*UUint32Pointer1) == 0x01)
	{
		OverallSign_Clone.CanSendCodeSign_B3 = true;
	}
	else
	{
		OverallSign_Clone.CanSendCodeSign_B3 = false;
	}
	UUint32Pointer1++;


	str_Information_Clone.Empty();
	if ((*UUint32Pointer1) == 0x01)
	{
		Error_main_Clone.MainErrorSign = true;
		str_Information_Clone = str_Information_Clone + _T("DSP main error");
	}
	else
	{
		Error_main_Clone.MainErrorSign = false;
	}
	UUint32Pointer1++;

	if ((*UUint32Pointer1) == 0x01)
	{	//获取稳定坐标 
		OverallSign_Clone.GetPositionSign = true;
		//GetDlgItem(IDC_Clone_GetPosition)->SetWindowText(_T("1"));
	}
	else
	{	//未获取稳定坐�??
		OverallSign_Clone.GetPositionSign = false;
		//GetDlgItem(IDC_Clone_GetPosition)->SetWindowText(_T("0"));
	}
	UUint32Pointer1++;

	if ((*UUint32Pointer1) == 0x01)
	{	//�??后一条代码走完标�??  1－走�??   
		OverallSign_Clone.LastCodeOverSign = true;
		//GetDlgItem(IDC_Clone_CodeOver)->SetWindowText(_T("1"));
	}
	else
	{	//�??后一条代码走完标�??  1－走�??   
		OverallSign_Clone.LastCodeOverSign = false;
		//GetDlgItem(IDC_Clone_CodeOver)->SetWindowText(_T("0"));
	}
	UUint32Pointer1++;

	temp = *UUint32Pointer1++;
	if (temp == 0x01)
	{
		OverallSign_Clone.ParameterWriteCompleteSign = true;
		if (OverallSign_Clone.NeedParameterWriteSign == true)
		{
			OverallSign_Clone.NeedParameterWriteSign = false;
		}
	}
	else
	{
		OverallSign_Clone.ParameterWriteCompleteSign = false;
	}

	//use 32bit to receive positive hard limit,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	FPGA_HardLimitPOS_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_Limit_HPOS)->SetWindowText(str);

	//use 32bit to receive negative hard limit,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	FPGA_HardLimitNEG_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_Limit_HNEG)->SetWindowText(str);

	//use 32bit to receive position complete signal,Max. aixs=32 axis
	temp = 0xF87FFFFF & (*UUint32Pointer1++);
	FPGA_Coin_Register_Clone.all = temp;
	OverallSign_Clone.EveryAxisCoinValid = (temp & 0x80000000) ? (1) : (0);
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_COIN)->SetWindowText(str);

	//use 32bit to receive Home complete signal,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	FPGA_Home_Register_Clone.all = temp;
	Homing_Clone.Findzero = (temp & 0x80000000) ? (1) : (0);
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_FindRef)->SetWindowText(str);
	if (Homing_Clone.Findzero)
	{
		str_Information_Clone = str_Information_Clone + _T("已找到参考点 ");
	}
	else
	{
		str_Information_Clone = str_Information_Clone + _T("无参考点 ");
	}

	//use 32bit to receive other alarm,Max. =32
	temp = *UUint32Pointer1++;
	temp1 = temp & 0x01;
	if (temp1 == 0x01)
	{
		Error_main_Clone.InsideRAMCheckError = true;		//BIT0  DSP芯片内部RAM�??验出错报�??  		1: 报警有效
		str_Information_Clone = str_Information_Clone + _T("DSP芯片内部RAM�??验出�?? ");
	}
	temp1 = temp & 0x02;
	if (temp1 == 0x02)
	{
		Error_main_Clone.OutsideRAMCheckError = true;	//BIT1  DSP芯片外部RAM�??验出错报�??  		1: 报警有效
		str_Information_Clone = str_Information_Clone + _T("DSP芯片外部RAM�??验出�?? ");
	}
	temp1 = temp & 0x0004;
	if (temp1 == 0x0004)
	{
		Error_main_Clone.NCCodeBreak = 1;			//BIT2  PC向DSP发�?�代码不连续报警1: 报警有效
		str_Information_Clone = str_Information_Clone + _T("发�?�代码不连续 ");
	}
	temp1 = temp & 0x0008;
	if (temp1 == 0x0008)
	{
		Error_main_Clone.TrackRunOutError = true;				//BIT3  DSP轨迹超程报警  			1: 报警有效
		str_Information_Clone = str_Information_Clone + _T("轨迹超程报警 ");
	}
	temp1 = temp & 0x0010;
	if (temp1 == 0x0010)
	{
		Error_main_Clone.InterpolationOver = true; 		//BIT4  DSP插补量过大报�??  		1: 报警有效
		str_Information_Clone = str_Information_Clone + _T("插补量过大报�?? ");
	}
	temp1 = temp & 0x0020;
	if (temp1 == 0x0020)
	{
		Error_main_Clone.AlmHLimitPos = true; 		//BIT5  DSP   AlmHLimitPos  1:valid
		str_Information_Clone = str_Information_Clone + _T("Positive Hard Limit Alarm,check Positive Hard Limit");
	}
	temp1 = temp & 0x0040;
	if (temp1 == 0x0040)
	{
		Error_main_Clone.AlmHLimitNeg = true; 		//BIT6  DSP   AlmHLimitNeg  1:valid
		str_Information_Clone = str_Information_Clone + _T("Negative Hard Limit Alarm,check Negative Hard Limit");
	}
	temp1 = temp & 0x0080;
	if (temp1 == 0x0080)
	{
		Error_main_Clone.AlmSLimitPos = true; 		//BIT7  DSP   AlmSLimitPos  1:valid
		str_Information_Clone = str_Information_Clone + _T("Positive Soft Limit Alarm,check Positive Soft Limit");
	}
	temp1 = temp & 0x0100;
	if (temp1 == 0x0100)
	{
		Error_main_Clone.AlmSLimitNeg = true; 		//BIT8  DSP   AlmHLimitNeg  1:valid
		str_Information_Clone = str_Information_Clone + _T("Negative Soft Limit Alarm,check Positive Soft Limit");
	}
	temp1 = temp & 0x0200;
	if (temp1 == 0x0200)
	{
		Error_main_Clone.CompDataCheckError = true; 		//BIT9  DSP   DSP_CompDataCheckError  1:valid
		str_Information_Clone = str_Information_Clone + _T("DSP_CompDataCheckError...");
	}
	temp1 = temp & 0x0400;
	if (temp1 == 0x0400)
	{
		Error_main_Clone.OpticalFlashTriggerBreakError = true; 		//BIT10  DSP   DSP_OpticalFlashTriggerBreakError  1:valid
		str_Information_Clone = str_Information_Clone + _T("DSP_OpticalFlashTriggerBreakError...");
	}
	temp1 = temp & 0x0800;
	if (temp1 == 0x0800)
	{
		Error_main_Clone.CameraShutterTriggerBreakError = true; 		//BIT11  DSP   DSP_CameraShutterTriggerBreakError  1:valid
		str_Information_Clone = str_Information_Clone + _T("DSP_CameraShutterTriggerBreakError...");
	}
	temp1 = temp & 0x1000;
	if (temp1 == 0x1000)
	{
		Error_main_Clone.LensPositionBreakError = true; 		//BIT12  DSP   DSP_LensPositionBreakError  1:valid
		str_Information_Clone = str_Information_Clone + _T("DSP_LensPositionBreakError...");
	}

	//use 32bit to receive alarm signal,Max. aixs=32 axis
	//temp=0x7FFFFF-*UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	FPGA_ServoAlarm_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_ServoAlarm)->SetWindowText(str);
	if (temp & 0x1FFFFFF)
	{
		str_Information_Clone = str_Information_Clone + _T("轴伺服报�?? ");
	}

	temp = *UUint32Pointer1++;
	FPGA_EncoderAlarm_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_EncoderAlarm)->SetWindowText(str);

	if (temp & 0x1FFFFFF)
	{
		str_Information_Clone = str_Information_Clone + _T("轴编码器报警 ");
	}

	//GPIN1
	temp = 0xFFFFFFFF - (*UUint32Pointer1++);
	Input_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_GPIN)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_GPIOIN1)->SetWindowText(str);
	//GPIN2
	temp = 0xFFFFFFFF - (*UUint32Pointer1++);
	Input_Register_Clone2.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_GPIN2)->SetWindowText(str);
	//m_Control_MainInterface.//GetDlgItem(IDC_Clone_GPIOIN2)->SetWindowText(str);

	//GPOU1
	temp = 0xFFFFFFFF - *UUint32Pointer1++;
	Output_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_GPOUT)->SetWindowText(str);
	//GPOU2
	temp = *UUint32Pointer1++;
	Output_Register_Clone2.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_GPOUT2)->SetWindowText(str);

	//MainCmd1
	temp = *UUint32Pointer1++;
	MainCommand_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainCMD)->SetWindowText(str);
	//MainCmd2
	temp = *UUint32Pointer1++;
	MainCommand_Register_Clone2.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainCMD2)->SetWindowText(str);

	//MainStatus1
	temp = *UUint32Pointer1++;
	MainStatus_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainStatus)->SetWindowText(str);
	//MainStatus2
	temp = *UUint32Pointer1++;
	MainStatus_Register_Clone2.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainStatus2)->SetWindowText(str);

	//use 32bit to receive positive soft limit,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	SoftLimitPOS_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_Limit_SPOS)->SetWindowText(str);

	//use 32bit to receive negative soft limit,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	SoftLimitNEG_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_Limit_SNEG)->SetWindowText(str);

	temp = *UUint32Pointer1++;
	TrackRunOver_Register_Clone.all = temp;
	if (temp & 0x1FFFFFF)
	{
		str_Information_Clone = str_Information_Clone + _T("TrackRunOver!");
	}

	temp = *UUint32Pointer1++;
	InterpolationOver_Register_Clone.all = temp;
	if (temp & 0x1FFFFFF)
	{
		str_Information_Clone = str_Information_Clone + _T("InterpolationOver!");
	}

	temp = *UUint32Pointer1++;
	CompensationDataCheck_Register_Clone.all = temp;
	if (temp & 0x1FFFFFF)
	{
		str_Information_Clone = str_Information_Clone + _T("CompensationDataCheckError!");
	}

	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;

	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;

	temp = *UUint32Pointer1++;//[PacketCheckCodeBuf_B4]
	temp = *UUint32Pointer1++;//[PacketCheckCodeBuf_B3]
	temp = *UUint32Pointer1++;//[PacketCheckCodeBuf_B2]
	temp = *UUint32Pointer1++; //[PacketCheckCodeBuf_B1]
	temp = *UUint32Pointer1++;//[PacketCheckCodeBuf_B0]




	if (OverallSign_Clone.m_ReciveSuccess == false)
	{
		if (OverallSign_Clone.m_ReciveLostCounter > 200)
		{
			//GetDlgItem(IDC_Clone_rec)->SetWindowText(_T("PC与DSP通讯错误"));
		}
	}

	str.Format(_T("%d"), OverallSign_Clone.m_ReciveLostCounter);
	//GetDlgItem(IDC_Clone_DSPCom_CounterError)->SetWindowText(str);
	str.Format(_T("%d"), OverallSign_Clone.m_ReciveSuccessCounter);
	//GetDlgItem(IDC_Clone_DSPCom_CounterOK)->SetWindowText(str);
	str.Format(_T("%d"), OverallSign_Clone.sendcount);
	//GetDlgItem(IDC_Clone_DSPSend_Counter)->SetWindowText(str);

	//GetDlgItem(IDC_Clone_rec)->SetWindowText(str_Information_Clone);

	return 1;
}







int DSP_com_MainCMD_b0(void) //1:run
{
	
	unsigned int data;
	unsigned int temp;
	unsigned int length;
	Int32  *SendUUint32Pointer1;

	length = SystemPrameterPackageLength_B0;	//32

	//SendUUint32Pointer1 = (unsigned int *)TxBuf;
	SendUUint32Pointer1 = TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length;
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Dual_Core;				//站点地址
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb0;				//
	*SendUUint32Pointer1++ = 0x01;				//

	OverallSign_Clone.ServoOn = true;//for test ,always servo on...
	*SendUUint32Pointer1++ = OverallSign_Clone.ServoOn;// 1

	*SendUUint32Pointer1++ = OverallSign_Clone.NCSign;// 2

	if (OverallSign_Clone.RapidFeedrateOverride == 0)
	{
		temp = 0;
	}
	else
	{
		temp = 10;
	}
	*SendUUint32Pointer1++ = temp; // 3

	if (OverallSign_Clone.FeedrateOverride == 0)
	{
		temp = 0;
	}
	else
	{
		temp = 10;
	}
	*SendUUint32Pointer1++ = temp; // 4

	//for test LED Flash//5,6,7,8
	*SendUUint32Pointer1++ = 0;// LEDFlashCtr1 (1:ctrl LED1 Flash)
	*SendUUint32Pointer1++ = 0;// LEDFlashCtr2 (1:ctrl LED2 Flash)
	*SendUUint32Pointer1++ = 0x500; //LEDFlashTime1 unit:um; 0xffff=always on
	*SendUUint32Pointer1++ = 0x500; //LEDFlashTime2 unit:um; 0xffff=always on

	*SendUUint32Pointer1++ = (OverallSign_Clone.JOGAxisSelect & 0x1fff);// 9 JOGAxis 

	temp = 0;
	if (OverallSign_Clone.CanSendCodeSign_B4 == true)
	{
// 		if (m_MotorRun_POSDown == true)
// 		{
// 			temp = 1;//正转
// 			m_MotorRun_POSDown = false;
// 		}
// 		if (m_MotorRun_NEGDown == true)
// 		{
// 			temp = 2;//反转
// 			m_MotorRun_NEGDown = false;
// 		}
		temp = 1;
	}
	else
	{
		temp = 2;
	}
	*SendUUint32Pointer1++ = 0; // 10  如果此项不为0 则b4包也可以发送，即SPC

	*SendUUint32Pointer1++ = (m_JOGDistance * 1000); // 11

	if (OverallSign_Clone.m_OperationMode == JOG)
	{
		temp = m_JogSpeed;//mm/s
	}
	else
	{
		temp = m_speedinfo_Set;
	}
	*SendUUint32Pointer1++ = (temp); 	// 12

	temp = 0;
	if (OverallSign_Clone.NeedGetPositionSign == true)
	{
		OverallSign_Clone.Initial_STDSign = true;
		OverallSign_Clone.Initial_SPCSign = true;
	}
	if (OverallSign_Clone.Initial_STDSign == true)
	{//bit0
		temp |= 0x0001;
	}

	if (OverallSign_Clone.Initial_SPCSign == true)
	{//bit1
		temp |= 0x0002;
	}

	if ((OverallSign_Clone.Initial_STDSign == true) || (OverallSign_Clone.Initial_SPCSign == true))
	{//bit2
		if ((OverallSign_Clone.GetPositionSign == 0)
			&& (OverallSign_Clone.STDReceiveDSPRunCount == 0)
			&& (OverallSign_Clone.SPCReceiveDSPRunCount == 0)
			&& (OverallSign_Clone.STDReceiveDSPSendCount == 0)
			&& (OverallSign_Clone.SPCReceiveDSPSendCount == 0)
			)
		{
			OverallSign_Clone.NeedGetPositionSign = false;
		}
		OverallSign_Clone.Initial_STDSign = false;
		OverallSign_Clone.Initial_SPCSign = false;
		//OverallSign_Clone.STDSendCount = 0;
		//OverallSign_Clone.SPCSendCount = 0;
		temp |= 0x0004;
	}
	*SendUUint32Pointer1++ = temp;// 13

	temp = 0;
	//bit0
	if (OverallSign_Clone.CameraScanSign == true)
	{
		temp |= 0x0001;			// 1:CameraScan
	}
	//bit1
	if (OverallSign_Clone.CameraStaticCaptureSign == true)
	{
		temp |= 0x0002;			// 1:CameraStaticCapture=camera stepper
	}
	//bit2
	if (OverallSign_Clone.MicroEScanSign == true)
	{
		temp |= 0x0004;			// 1:MicroEScan
	}
	//bit3
	if (OverallSign_Clone.CameraSoftTriggerLedFlashSign == true)
	{
		temp |= 0x0008;			// 1:CameraSoftTriggerLedFlashSign
	}
	*SendUUint32Pointer1++ = temp;// 14

	temp = 0;
	//if((Homing_Clone.Findzero == 0)&&(OverallSign_Clone.m_OperationMode==REFERENCE))
	if (OverallSign_Clone.m_OperationMode == REFERENCE)
	{
		if (Homing_Clone.NeedFindAxisSign.Axis1_Clone_Linear_Z)
		{
			temp |= 0x01;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis2_Clone_Linear_A)
		{
			temp |= 0x02;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis3_Clone_Optical_LensTrans)
		{
			temp |= 0x04;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis4_Clone_Optical_FilterTrans)
		{
			temp |= 0x08;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis5)
		{
			temp |= 0x010;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis6_PlateEmpty_Destination_Z1_R)
		{
			temp |= 0x020;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis7_PlateEmpty_Destination_Y)
		{
			temp |= 0x040;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis8_PlateEmpty_Destination_X)
		{
			temp |= 0x080;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis9_Plate_Transfer_Z)
		{
			temp |= 0x0100;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis10_Plate_Transfer_A)
		{
			temp |= 0x0200;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis11_PlateWaste_Source_X)
		{
			temp |= 0x0400;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis12_PlateWaste_Source_Y)
		{
			temp |= 0x0800;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis13)
		{
			temp |= 0x01000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis14)
		{
			temp |= 0x02000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis15_Clone_Linear_X)
		{
			temp |= 0x04000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis16_Clone_Linear_Y)
		{
			temp |= 0x08000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis17_Clone_Optical_X)
		{
			temp |= 0x00010000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis18_Clone_Optical_Y)
		{
			temp |= 0x00020000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis19_Clone_Optical_LensUpDown)
		{
			temp |= 0x00040000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis20_PlateEmpty_Destination_Z2_L)
		{
			temp |= 0x00080000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis21_Plate_Transfer_X)
		{
			temp |= 0x00100000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis22_Plate_Transfer_Y)
		{
			temp |= 0x00200000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis23)
		{
			temp |= 0x00400000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis24)
		{
			temp |= 0x00800000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis25)
		{
			temp |= 0x01000000;
		}

		if (Homing_Clone.SearchRefSign == true)
		{
			temp |= 0x80000000;
		}
	}
	*SendUUint32Pointer1++ = temp; // 15

	//*SendUUint32Pointer1++ = 0x00000001;//16 FunctionSelectionSwitch = 0;//for test all;FunctionSelectionSwitch = 1;//for test Signal
	*SendUUint32Pointer1++ = 0;//16 FunctionSelectionSwitch = 0;//for test all;FunctionSelectionSwitch = 1;//for test Signal
	//*SendUUint32Pointer1++ = 0xABCD;//16 FunctionSelectionSwitch = 0xABCD;//for test all;FunctionSelectionSwitch = 1;//for test Signal
	*SendUUint32Pointer1++ = 0x10;//17//for test
	*SendUUint32Pointer1++ = 0x11;//18//for test

	*SendUUint32Pointer1++ = 0x12;//19//for test
	*SendUUint32Pointer1++ = 0x13;//20//for test
	*SendUUint32Pointer1++ = 0x14;//21//for test
	*SendUUint32Pointer1++ = 0x15;//22//for test
	*SendUUint32Pointer1++ = 0x16;//23//for test
	*SendUUint32Pointer1++ = 0x17;//24//for test
	*SendUUint32Pointer1++ = 0x18;//25//for test
	*SendUUint32Pointer1++ = 0x19;//26//for test

	data = crc32((unsigned int*)TxBuf, length);//length=26-1+7=32	
	*SendUUint32Pointer1++ = data;
	*SendUUint32Pointer1++ = OverallSign_Clone.sendcount++;
	*SendUUint32Pointer1 = 0xbb;

	Int32 send_buffer[700];

	//ofstream out("out.txt");
	//ofstream GetHex;
	//GetHex.open("b.txt", ios::out | ios::hex, _SH_DENYNO);
	//out << hex;
	for (int i = 0; i <= (length + 3); i++)
	{
		//GetHex << TxBuf[i] << " ";
		send_buffer[i] = htonl(TxBuf[i]);	
		//out <<TxBuf[i]<<" ";
	}
	//GetHex.close();
	//out.close();

	UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);

	return 1;
}


/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b1()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: DSP_com_MainCMD_b1                                           */
/*	PC 	Main	 Command B1 */
/*	reserved data packet ,always connect ,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_b1(void) //reserved
{
	unsigned int data;
	int temp;
	unsigned short length;
	Int32  *SendUUint32Pointer1;

	OverallSign_Clone.NeedParameterWriteSign = true;

	length = SystemPrameterPackageLength_B1;//492	
	SendUUint32Pointer1 = TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length;
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Dual_Core;				//站点地址
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb1;				//
	*SendUUint32Pointer1++ = 0x01;				//


	*SendUUint32Pointer1++ = System_Clone.Tsample;	//
	*SendUUint32Pointer1++ = System_Clone.SlaveMAX;//
	*SendUUint32Pointer1++ = System_Clone.TrackRunOutRangeSQR;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.VeerDeltaV;//
	*SendUUint32Pointer1++ = System_Clone.NicetyVeerDeltaV;//
	*SendUUint32Pointer1++ = System_Clone.VeerDeltaT;//
	*SendUUint32Pointer1++ = System_Clone.NicetyVeerDeltaT;//
	*SendUUint32Pointer1++ = System_Clone.LinearAxisMinUnit;//
	*SendUUint32Pointer1++ = System_Clone.LinearAxisOutUnitEQU;//
	*SendUUint32Pointer1++ = System_Clone.G0Speed;//
	*SendUUint32Pointer1++ = System_Clone.G0Speed_2;//
	*SendUUint32Pointer1++ = System_Clone.G1Speed;//
	*SendUUint32Pointer1++ = System_Clone.G1Speed_2;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeed;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeedBack;//
	*SendUUint32Pointer1++ = System_Clone.SRefBack;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeed_2;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeedBack_2;//
	*SendUUint32Pointer1++ = System_Clone.SRefBack_2;//

	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis1_Clone_Linear_Z;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis2_Clone_Linear_A;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis3_Clone_Optical_LensTrans;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis4_Clone_Optical_FilterTrans;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis5;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis6_PlateEmpty_Destination_Z1_R;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis7_PlateEmpty_Destination_Y;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis8_PlateEmpty_Destination_X;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis9_Plate_Transfer_Z;//�??
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis11_PlateWaste_Source_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis12_PlateWaste_Source_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis13;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis14;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis15_Clone_Linear_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis16_Clone_Linear_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis17_Clone_Optical_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis18_Clone_Optical_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis19_Clone_Optical_LensUpDown;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis20_PlateEmpty_Destination_Z2_L;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis21_Plate_Transfer_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis22_Plate_Transfer_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis23;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis24;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis25;// 

	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis1_Clone_Linear_Z;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis2_Clone_Linear_A;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis3_Clone_Optical_LensTrans;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis4_Clone_Optical_FilterTrans;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis5;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis6_PlateEmpty_Destination_Z1_R;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis7_PlateEmpty_Destination_Y;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis8_PlateEmpty_Destination_X;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis9_Plate_Transfer_Z;// �??
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.FunctionSelect01;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect02;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect03;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect04;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect05;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect06;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect07;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect08;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect09;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect10;// 

	*SendUUint32Pointer1++ = System_Clone.LEDFlashTime1;
	*SendUUint32Pointer1++ = System_Clone.LEDFlashTime2;
	*SendUUint32Pointer1++ = 102;
	*SendUUint32Pointer1++ = 103;
	*SendUUint32Pointer1++ = 104;
	*SendUUint32Pointer1++ = 105;
	*SendUUint32Pointer1++ = 106;

	data = crc32((unsigned int *)TxBuf, length);//length=495-3
	*SendUUint32Pointer1++ = data;
	*SendUUint32Pointer1++ = OverallSign_Clone.sendcount++;

	*SendUUint32Pointer1++ = 0xbb;

	Int32 send_buffer[700];
	ofstream out("b3.txt");
// 	out << hex;
// 	unsigned char Hex32 = 0x00;
// 	unsigned char Hex24 = 0x00;
// 	unsigned char Hex16 = 0x00;
// 	unsigned char Hex8 = 0x00;
	for (int i = 0; i <= (length + 3); i++)
	{
// 		 Hex32 = 0x00;
// 		 Hex24 = 0x00;
// 		 Hex16 = 0x00;
// 		 Hex8 = 0x00;
// 		Hex32 |= TxBuf[i] >> 24;
// 		Hex24 |= TxBuf[i] >> 16;
// 		Hex16 |= TxBuf[i] >> 8;
// 		Hex8 |= TxBuf[i] ;
		send_buffer[i] = htonl(TxBuf[i]);
/*		out << TxBuf[i]<<endl;*/
		//out << Hex32 << Hex24<< Hex16 << Hex8 ;
	}
	out.close();
	UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);

	return 1;
}
/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b2()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: DSP_com_MainCMD_b2                                           */
/*	PC 	Main	 Command B2 */
/*	compensation data packet ,always connect ,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_b2(void) //PC->DSP分段补偿数据通讯包协�??
{
	unsigned int data;
	unsigned short length;
	Int32  *SendUINT32UPointer1;
	Uint32	temp_count, MicroEDataBaseStartCnt;

	OverallSign_Clone.NeedMicroEDataBaseSendSign = true;

	length = SystemPrameterPackageLength_B2;//210
	SendUINT32UPointer1 = TxBuf;
	*SendUINT32UPointer1++ = 0xaa;				//START
	*SendUINT32UPointer1++ = length;
	*SendUINT32UPointer1++ = 0xdd;				//ʶ����0xdd
	*SendUINT32UPointer1++ = IP_Dual_Core;				//站点地址
	*SendUINT32UPointer1++ = 0x01;				//
	*SendUINT32UPointer1++ = 0xb2;				//
	*SendUINT32UPointer1++ = 0x01;				//

	MicroEDataBaseStartCnt = OverallSign_Clone.MicroEDataBaseStartCnt;
	OverallSign_Clone.MicroEDataBaseEndCnt = OverallSign_Clone.MicroEDataBaseStartCnt + (SystemPrameterPackageLength_B2 - 10);

	if (OverallSign_Clone.MicroEDataBaseEndCnt > OverallSign_Clone.MicroEDataBaseTotalCnt)
	{
		OverallSign_Clone.MicroEDataBaseEndCnt = OverallSign_Clone.MicroEDataBaseTotalCnt;
	}

	*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseStartCnt;
	*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseEndCnt;
	*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseTotalCnt;

	for (temp_count = 0; temp_count < (SystemPrameterPackageLength_B2 - 10); temp_count++)
	{
		if (OverallSign_Clone.MicroEDataBaseStartCnt < OverallSign_Clone.MicroEDataBaseTotalCnt)
		{
#if MicroESendDataSign
			//*SendUINT32UPointer1++ = MonitorBuffer1[OverallSign_Clone.MicroEDataBaseStartCnt++].Z-MonitorBuffer1[0].Z+System_Clone_Optical.OffsetCoordinate.Z;
			*SendUINT32UPointer1++ = MonitorBuffer1[OverallSign_Clone.MicroEDataBaseStartCnt++].Axis19_Clone_Optical_LensUpDown;
#else
			//for test
			*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseStartCnt;
#endif

			//OverallSign_Clone.MicroEDataBaseStartCnt++;
		}
		else
		{
			*SendUINT32UPointer1++ = 0;
		}
	}
	OverallSign_Clone.MicroEDataBaseEndCnt = OverallSign_Clone.MicroEDataBaseStartCnt;

	*SendUINT32UPointer1++ = 0;

	data = crc32((unsigned int *)TxBuf, length);
	*SendUINT32UPointer1++ = data;
	*SendUINT32UPointer1++ = OverallSign_Clone.sendcount++;

	*SendUINT32UPointer1++ = 0xbb;//51

	Int32 send_buffer[700];
	for (int i = 0; i <= (length + 3); i++)
	{
		send_buffer[i] = htonl(TxBuf[i]);
	}

	UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);

	return 1;
}
/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b3()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_b3                                           */
/*	PC 	Main	 Command B3 */
/*	STD batch Code packet ,only connect if have STD batch Code,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_b3(void) //PC->DSP自动模式代码组数据�?�讯包协�??
{	
	Int32 data;
	Uint32 temp;
	Int16 length;
	Int32  *SendUUint32Pointer1;
	GCODE  *GCodeSIPointer; 
	Uint32 si;

	length = 70;
	
	 SendUUint32Pointer1 = TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length ;
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Dual_Core;				//绔欑偣鍦板潃
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb3;				//
	*SendUUint32Pointer1++ = 0x01;				//

	si = (OverallSign_Clone.STDSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	//OverallSign_Clone.STDSendCount++;//for test B3 packet
	*SendUUint32Pointer1++ = OverallSign_Clone.STDSendCount;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Main_CMD;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Sub_CMD1;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Sub_CMD2;			


	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis5;
	GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R = 0;
	GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y = 0 ;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis8_PlateEmpty_Destination_X;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis9_Plate_Transfer_Z;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis10_Plate_Transfer_A;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis11_PlateWaste_Source_X;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis12_PlateWaste_Source_Y;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis13;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis14;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis15_Clone_Linear_X;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis17_Clone_Optical_X;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis18_Clone_Optical_Y;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis19_Clone_Optical_LensUpDown;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis20_PlateEmpty_Destination_Z2_L;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis21_Plate_Transfer_X;		
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis22_Plate_Transfer_Y;	
	//GCodeSIPointer->EndPoint.Axis23 = 123456;//for test
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis23;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis24;			
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis25;		
	
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis1_Clone_Linear_Z;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis2_Clone_Linear_A;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis3_Clone_Optical_LensTrans;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis4_Clone_Optical_FilterTrans;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis5;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis6_PlateEmpty_Destination_Z1_R;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis7_PlateEmpty_Destination_Y;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis8_PlateEmpty_Destination_X;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis9_Plate_Transfer_Z;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis10_Plate_Transfer_A;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis11_PlateWaste_Source_X;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis12_PlateWaste_Source_Y;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis13;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis14;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis15_Clone_Linear_X;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis16_Clone_Linear_Y;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis17_Clone_Optical_X;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis18_Clone_Optical_Y;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis19_Clone_Optical_LensUpDown;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis20_PlateEmpty_Destination_Z2_L;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis21_Plate_Transfer_X;		
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis22_Plate_Transfer_Y;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis23;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis24;			
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis25;		

	//GCodeSIPointer->HoldTime = 334455;//for test
	*SendUUint32Pointer1++ = GCodeSIPointer->HoldTime;
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved	
	*SendUUint32Pointer1++ = 0;//reserved 
	*SendUUint32Pointer1++ = 0;//reserved 
	*SendUUint32Pointer1++ = 0;//reserved 
	*SendUUint32Pointer1++ = 0;//reserved 	

	data = crc32((unsigned int *)TxBuf,length);//length=64-1+7=70
	*SendUUint32Pointer1++ = data;		
	*SendUUint32Pointer1++ = OverallSign_Clone.sendcount++; 

	*SendUUint32Pointer1 = 0xbb;		
	ofstream out("out.txt");
	//ofstream GetHex;
	//GetHex.open("b.txt", ios::out | ios::hex, _SH_DENYNO);
	//out << hex;
	static unsigned char Hex32 = 0x00;
	static unsigned char Hex24 = 0x00;
	static unsigned char Hex16 = 0x00;
	static unsigned char Hex8 = 0x00;
	Int32 send_buffer[700];
	for(int i=0;i<=(length+3);i++)
	{
		Hex32 = 0x00;
		Hex24 = 0x00;
		Hex16 = 0x00;
		Hex8 = 0x00;
		Hex32 |= TxBuf[i] >> 24;
		Hex24 |= TxBuf[i] >> 16;
		Hex16 |= TxBuf[i] >> 8;
		Hex8 |= TxBuf[i];
	   send_buffer[i]= htonl(TxBuf[i]);
	   out << TxBuf[i] <<endl;
	   //out << Hex32 << Hex24 << Hex16 << Hex8;
	}
	out.close();
	UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);

	return 1;
}

/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b4()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_b4                                           */
/*	PC 	Main	 Command B4 */
/*	SPC single Code packet ,connect only have SPC code,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_b4(void) //PC->DSP手动模式代码组数据�?�讯包协�??
{	
	Int32 data;
	Uint32 temp;
	Int16 length;
	//Uint32  *SendUUint32Pointer1;
	Int32  *SendUUint32Pointer1;
	CString  		str;

	length = 38;

	 SendUUint32Pointer1 = TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length ;
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Dual_Core;				//站点地址
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb4;				//
	*SendUUint32Pointer1++ = 0x01;				//

	*SendUUint32Pointer1++ = OverallSign_Clone.SPCSendCount;
	
	//for test
	//OverallSign_Clone.SPCSendCount++;
	m_SET_ABS.Axis1_Clone_Linear_Z = 1;
	m_SET_ABS.Axis2_Clone_Linear_A = 2;
	m_SET_ABS.Axis3_Clone_Optical_LensTrans = 3;
	m_SET_ABS.Axis4_Clone_Optical_FilterTrans = 4;	
	m_SET_ABS.Axis5 = 5;
	m_SET_ABS.Axis6_PlateEmpty_Destination_Z1_R = 6;	
	m_SET_ABS.Axis7_PlateEmpty_Destination_Y = 7;
	m_SET_ABS.Axis8_PlateEmpty_Destination_X = 8;	
	m_SET_ABS.Axis9_Plate_Transfer_Z = 9;
	m_SET_ABS.Axis10_Plate_Transfer_A = 10;	
	m_SET_ABS.Axis11_PlateWaste_Source_X = 11;
	m_SET_ABS.Axis12_PlateWaste_Source_Y = 12;
	m_SET_ABS.Axis13 = 13;
	m_SET_ABS.Axis14 = 14;	
	m_SET_ABS.Axis15_Clone_Linear_X = 15;
	m_SET_ABS.Axis16_Clone_Linear_Y = 16;	
	m_SET_ABS.Axis17_Clone_Optical_X = 17;
	m_SET_ABS.Axis18_Clone_Optical_Y = 18;	
	m_SET_ABS.Axis19_Clone_Optical_LensUpDown = 19;
	m_SET_ABS.Axis20_PlateEmpty_Destination_Z2_L = 20;
	m_SET_ABS.Axis21_Plate_Transfer_X = 21;
	m_SET_ABS.Axis22_Plate_Transfer_Y = 22;
	m_SET_ABS.Axis23 = 23;
	m_SET_ABS.Axis24 = 24;	
	m_SET_ABS.Axis25 = 25;
	
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis1_Clone_Linear_Z*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis2_Clone_Linear_A*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis3_Clone_Optical_LensTrans*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis4_Clone_Optical_FilterTrans*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis5*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis6_PlateEmpty_Destination_Z1_R*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis7_PlateEmpty_Destination_Y*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis8_PlateEmpty_Destination_X*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis9_Plate_Transfer_Z*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis10_Plate_Transfer_A*1000.0);		

	*SendUUint32Pointer1++ = (m_SET_ABS.Axis11_PlateWaste_Source_X*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis12_PlateWaste_Source_Y*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis13*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis14*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis15_Clone_Linear_X*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis16_Clone_Linear_Y*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis17_Clone_Optical_X*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis18_Clone_Optical_Y*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis19_Clone_Optical_LensUpDown*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis20_PlateEmpty_Destination_Z2_L*1000.0);		

	*SendUUint32Pointer1++ = (m_SET_ABS.Axis21_Plate_Transfer_X*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis22_Plate_Transfer_Y*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis23*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis24*1000.0);		
	*SendUUint32Pointer1++ = (m_SET_ABS.Axis25*1000.0);		

	if(OverallSign_Clone.m_OperationMode==JOG)
	{
		temp=m_JogSpeed;
	}
	else
	{
		temp=m_speedinfo_Set;
	}
	*SendUUint32Pointer1++ = (temp);	

	*SendUUint32Pointer1++ = 0;	
	*SendUUint32Pointer1++ = 0;		
	*SendUUint32Pointer1++ = 0;		
	*SendUUint32Pointer1++ = 0;
	*SendUUint32Pointer1++ = 0;

	data = crc32((unsigned int *)TxBuf,length);//length = 32-1+7=38
	*SendUUint32Pointer1++ = data;
	*SendUUint32Pointer1++ = OverallSign_Clone.sendcount++;

	*SendUUint32Pointer1 = 0xbb;

	Int32 send_buffer[700];
	for(int i=0;i<=(length+3);i++)
	{
		send_buffer[i]= htonl(TxBuf[i]);
	}
	 
	 
	UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);

	return 1;
}


/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b5()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_b5                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	DSP_com_MainCMD_b5 */
/*	system para. Linear compensation value packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
//�����ϵ��ʼ������??(�ֶβ���)
int DSP_com_MainCMD_b5(void)		
{
	unsigned int data;
	int temp;
	unsigned short length;
	Int32  *SendUUint32Pointer1;
	int i;

	Uint16 ArrayN;
	OverallSign_Clone.PacketSendLinearSign = TRUE;

	length = 8;//temp value
	
	 SendUUint32Pointer1 = TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length;				
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Clone;				//站点地址
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb5;				//linear compensation
	*SendUUint32Pointer1++ = 0x01;				//

	*SendUUint32Pointer1++ = LinearCompensation.AxisID;	//
	switch(LinearCompensation.AxisID)
	{		
		case(1):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis1_Clone_Linear_Z; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis1_Clone_Linear_Z;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis1_Clone_Linear_Z;	
			}
			break;
		
		case(2):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis2_Clone_Linear_A; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis2_Clone_Linear_A;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis2_Clone_Linear_A;	
			}
			break;
		
		case(3):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis3_Clone_Optical_LensTrans; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis3_Clone_Optical_LensTrans;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis3_Clone_Optical_LensTrans;	
			}
			break;

		case(4):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis4_Clone_Optical_FilterTrans; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis4_Clone_Optical_FilterTrans;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis4_Clone_Optical_FilterTrans;	
			}
			break;

		case(5):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis5; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis5;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis5;	
			}
			break;	

		case(6):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis6_PlateEmpty_Destination_Z1_R; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis6_PlateEmpty_Destination_Z1_R;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis6_PlateEmpty_Destination_Z1_R;	
			}
			break;
		
		case(7):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis7_PlateEmpty_Destination_Y; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis7_PlateEmpty_Destination_Y;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis7_PlateEmpty_Destination_Y;	
			}
			break;
		
		case(8):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis8_PlateEmpty_Destination_X; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis8_PlateEmpty_Destination_X;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis8_PlateEmpty_Destination_X;	
			}
			break;
		
		case(9):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis9_Plate_Transfer_Z; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis9_Plate_Transfer_Z;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis9_Plate_Transfer_Z;	
			}
			break;
		
		case(10):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis10_Plate_Transfer_A; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis10_Plate_Transfer_A;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis10_Plate_Transfer_A;	
			}
			break;	

		case(11):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis11_PlateWaste_Source_X; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis11_PlateWaste_Source_X;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis11_PlateWaste_Source_X;	
			}
			break;
		
		case(12):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis12_PlateWaste_Source_Y; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis12_PlateWaste_Source_Y;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis12_PlateWaste_Source_Y;	
			}
			break;
		
		case(13):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis13; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis13;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis13;	
			}
			break;
		
		case(14):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis14; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis14;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis14;	
			}
			break;
		
		case(15):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis15_Clone_Linear_X; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis15_Clone_Linear_X;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis15_Clone_Linear_X;	
			}
			break;	
		
		case(16):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis16_Clone_Linear_Y; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis16_Clone_Linear_Y;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis16_Clone_Linear_Y;	
			}
			break;
		
		case(17):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis17_Clone_Optical_X; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis17_Clone_Optical_X;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis17_Clone_Optical_X;	
			}
			break;
		
		case(18):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis18_Clone_Optical_Y; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis18_Clone_Optical_Y;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis18_Clone_Optical_Y;	
			}
			break;
		
		case(19):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis19_Clone_Optical_LensUpDown; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis19_Clone_Optical_LensUpDown;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis19_Clone_Optical_LensUpDown;	
			}
			break;
		
		case(20):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis20_PlateEmpty_Destination_Z2_L; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis20_PlateEmpty_Destination_Z2_L;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis20_PlateEmpty_Destination_Z2_L;	
			}
			break;	
			
		case(21):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis21_Plate_Transfer_X; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis21_Plate_Transfer_X;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis21_Plate_Transfer_X;	
			}
			break;
		
		case(22):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis22_Plate_Transfer_Y; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis22_Plate_Transfer_Y;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis22_Plate_Transfer_Y;	
			}
			break;
		
		case(23):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis23; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis23;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis23;	
			}
			break;
		
		case(24):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis24; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis24;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis24;	
			}
			break;
		
		case(25):		
			*SendUUint32Pointer1 = LinearCompensation.AxisCount.Axis25; 	//
			ArrayN = *SendUUint32Pointer1++;//
			
			for(i=0;i<ArrayN;i++)
			{
				*SendUUint32Pointer1++ = LinearCompensation.PARALinear_Buf[i].Axis25;	
				*SendUUint32Pointer1++ = LinearCompensation.PARALaser_Buf[i].Axis25;	
			}
			break;	
		default:
			ArrayN = 0;
			*SendUUint32Pointer1++ = 0;
			*SendUUint32Pointer1++ = 0;
			*SendUUint32Pointer1++ = 0;
			//MessageBox(_T("Linear Compensation AxisID select error"));
			break;	

	}

	length = ArrayN*2+8; //ArrayN*2+2+6
	TxBuf[1] = length;

	data = crc32((unsigned int *)TxBuf,length);//length
	*SendUUint32Pointer1++ = data;
	*SendUUint32Pointer1++ = OverallSign_Clone.sendcount++;

	*SendUUint32Pointer1++ = 0xbb;

	Int32 send_buffer[700];
	for(int i=0;i<=(length+3);i++)
	{
	   send_buffer[i]= htonl(TxBuf[i]);
	}
	 
	 
	UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);

	return 1;
}


/***************************************************************************/
/*  Function name: DSP_com_MainCMD_50()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_50                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	PC 	Main	 Command 50 */
/*	system para. Code 50 packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_50(void) //PRAMETER
{
	return 1;
}

/***************************************************************************/
/*  Function name: DSP_com_MainCMD_51()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_1                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	PC 	Main	 Command 51 */
/*	system para. Code 51 packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_51(void) 
{
	return 1;
}


/***************************************************************************/
/*  Function name: DSP_com_MainCMD_52()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_52                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	PC 	Main	 Command 52 */
/*	system para. Code 52 packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_52(void) 
{
	return 1;
}


/***************************************************************************/
/*  Function name: DSP_com_MainCMD_53()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_53                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	PC 	Main	 Command 53 */
/*	system para. Code 53 packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_53(void) 
{
	return 1;
}


/***************************************************************************/
/*  Function name: DSP_com_MainCMD_54()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_54                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	PC 	Main	 Command 54 */
/*	system para. Code 54 packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_54(void) 
{
	return 1;
}


/***************************************************************************/
/*  Function name: DSP_com_MainCMD_55()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_55                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	PC 	Main	 Command 55 */
/*	system para. Code 55 packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_55(void) 
{return 1;}

/***************************************************************************/
/*  Function name: DSP_com_MainCMD_56()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSP_com_MainCMD_56                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	PC 	Main	 Command 56 */
/*	system para. Code 56 packet ,connect only modify para,com. PC -> DSP		*/
/***************************************************************************/
int DSP_com_MainCMD_56(void) 
{return 1;}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G0()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G0                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	G0 Code ,FeedRate = G0Speed*/
/*	Packeting G0 code ,PC -> DSP		*/
/***************************************************************************/
Uint32 DSPCodePacket_Code_G0(void)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;

	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 0;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD2 = 0;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,System_Clone.G0Speed);
		
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);	
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;

	return si;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G1()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G1                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	G1 Code ,FeedRate = G1Speed*/
/*	Packeting G1 code ,PC -> DSP		*/
/***************************************************************************/
Uint32 DSPCodePacket_Code_G1(void)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;

	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 1;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD2 = 0;

	DSPCodePacket_Code_Feedrate(GCodeSIPointer,System_Clone.G1Speed);

	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	
	
	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
		
	return si;
}

/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G4()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G4                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*	G4 Code ,Motion Hold,HoldTime uint:ms*/
/*	Packeting G4 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G4(Uint32	HoldTime)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;

	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 4;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD1 = 0;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->HoldTime = HoldTime;
	//for test
	GCodeSIPointer->HoldTime = 100;
	//GCodeSIPointer->HoldTime = 2000;
	
	return TRUE;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G100()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G100          ��ʱΪ  �մ������??                               */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G100 code                                         */
/*	Packeting G100 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G100(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 100;//0x64
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD2 = 0;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}



/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G101()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G101                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G101 code   (LED flash)        ��FlashTime1=Sub_CMD1*0.1us   ;FlashTime2=Sub_CMD2*0.1us                          */
/*  if Sub_CMD1=500,then FlashTime1=500*0.1us=50us                          */
/*	Packeting G101 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G101(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 101;//0x65
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;//FlashTime1;unit 0.1us
	GCodeSIPointer->Sub_CMD2 = Sub_CMD2;//FlashTime2;unit 0.1us
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G102()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G102                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G102 code   (need check input port,such as sensor signal if avaliable)                                        */
/*	Packeting G102 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G102(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	//Main_CMD = 0x66  . need check input port...	
	// Sub_CMD1 (bit0~31 IO input)IN1-IN32
	//Sub_CMD1 bit0= 1	 =>in1
	//Sub_CMD1 bit1= 1	 =>in2
	//~~~~~~~~~~~~~~~~~~~
	// Sub_CMD2 (bit0~31 IO input)IN33-IN64
	//Sub_CMD2 bit0= 1	 =>in33
	//Sub_CMD2 bit1= 1	 =>in34
	//~~~~~~~~~~~~~~~~~~~

	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 102;//0x66
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;

	
	GCodeSIPointer->Sub_CMD1 = 0;
	//for test
	//GCodeSIPointer->Sub_CMD1 = 0x03;//need check Input_Register_Clone.bit.IN1 && Input_Register_Clone.bit.IN2
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	
	if(Input_Register_Clone.bit.SIGN1 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x01;	// need check Input_Register_Clone.bit.IN1
	}
	if(Input_Register_Clone.bit.SIGN2 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x02;	// need check Input_Register_Clone.bit.IN2
	}
	if(Input_Register_Clone.bit.SIGN3 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x04;	// need check Input_Register_Clone.bit.IN3
	}
	if(Input_Register_Clone.bit.SIGN4 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x08;	// need check Input_Register_Clone.bit.IN4
	}
	if(Input_Register_Clone.bit.SIGN5 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x010;	// need check Input_Register_Clone.bit.IN5
	}
	if(Input_Register_Clone.bit.SIGN6 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x020;	// need check Input_Register_Clone.bit.IN6
	}
	if(Input_Register_Clone.bit.SIGN7 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x040;	// need check Input_Register_Clone.bit.IN7
	}
	if(Input_Register_Clone.bit.SIGN8 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x080;	// need check Input_Register_Clone.bit.IN8
	}
	if(Input_Register_Clone.bit.SIGN9 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0100;	// need check Input_Register_Clone.bit.IN9
	}
	if(Input_Register_Clone.bit.SIGN10 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0200;	// need check Input_Register_Clone.bit.IN10
	}
	if(Input_Register_Clone.bit.SIGN11 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0400;	// need check Input_Register_Clone.bit.IN11
	}
	if(Input_Register_Clone.bit.SIGN12 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0800;	// need check Input_Register_Clone.bit.IN12
	}
	if(Input_Register_Clone.bit.SIGN13 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x1000;	// need check Input_Register_Clone.bit.IN13
	}
	if(Input_Register_Clone.bit.SIGN14 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x2000;	// need check Input_Register_Clone.bit.IN14
	}
	if(Input_Register_Clone.bit.SIGN15 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x4000;	// need check Input_Register_Clone.bit.IN15
	}
	if(Input_Register_Clone.bit.SIGN16 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x8000;	// need check Input_Register_Clone.bit.IN16
	}
	if(Input_Register_Clone.bit.SIGN17 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x010000;	// need check Input_Register_Clone.bit.IN17
	}
	if(Input_Register_Clone.bit.SIGN18 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x020000;	// need check Input_Register_Clone.bit.IN18
	}
	if(Input_Register_Clone.bit.SIGN19 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x040000;	// need check Input_Register_Clone.bit.IN19
	}
	if(Input_Register_Clone.bit.SIGN20 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x080000;	// need check Input_Register_Clone.bit.IN20
	}
	if(Input_Register_Clone.bit.SIGN21 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0100000;	// need check Input_Register_Clone.bit.IN21
	}
	if(Input_Register_Clone.bit.SIGN22 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0200000;	// need check Input_Register_Clone.bit.IN22
	}
	if(Input_Register_Clone.bit.SIGN23 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0400000;	// need check Input_Register_Clone.bit.IN23
	}
	if(Input_Register_Clone.bit.SIGN24 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x0800000;	// need check Input_Register_Clone.bit.IN24
	}
	if(Input_Register_Clone.bit.SIGN25 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x01000000;	// need check Input_Register_Clone.bit.IN25
	}
	if(Input_Register_Clone.bit.SIGN26 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x02000000;	// need check Input_Register_Clone.bit.IN26
	}
	if(Input_Register_Clone.bit.SIGN27 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x04000000;	// need check Input_Register_Clone.bit.IN27
	}
	if(Input_Register_Clone.bit.SIGN28 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x08000000;	// need check Input_Register_Clone.bit.IN28
	}
	if(Input_Register_Clone.bit.SIGN29 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x10000000;	// need check Input_Register_Clone.bit.IN29
	}
	if(Input_Register_Clone.bit.SIGN30 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x20000000;	// need check Input_Register_Clone.bit.IN30
	}
	if(Input_Register_Clone.bit.SIGN31 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x40000000;	// need check Input_Register_Clone.bit.IN31
	}
	if(Input_Register_Clone.bit.SIGN32 == TRUE)
	{
		GCodeSIPointer->Sub_CMD1 |=0x80000000;	// need check Input_Register_Clone.bit.IN32
	}

	GCodeSIPointer->Sub_CMD2 = 0;
	//for test
	//GCodeSIPointer->Sub_CMD2 = 0x03;//need check Input_Register_Clone2.bit.IN1 && Input_Register_Clone2.bit.IN2
	GCodeSIPointer->Sub_CMD2 = Sub_CMD2;
	
	if(Input_Register_Clone2.bit.SIGN1 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x01;	// need check Input_Register_Clone2.bit.IN1
	}
	if(Input_Register_Clone2.bit.SIGN2 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x02;	// need check Input_Register_Clone2.bit.IN2
	}
	if(Input_Register_Clone2.bit.SIGN3 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x04;	// need check Input_Register_Clone2.bit.IN3
	}
	if(Input_Register_Clone2.bit.SIGN4 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x08;	// need check Input_Register_Clone2.bit.IN4
	}
	if(Input_Register_Clone2.bit.SIGN5 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x010;	// need check Input_Register_Clone2.bit.IN5
	}
	if(Input_Register_Clone2.bit.SIGN6 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x020;	// need check Input_Register_Clone2.bit.IN6
	}
	if(Input_Register_Clone2.bit.SIGN7 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x040;	// need check Input_Register_Clone2.bit.IN7
	}
	if(Input_Register_Clone2.bit.SIGN8 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x080;	// need check Input_Register_Clone2.bit.IN8
	}
	if(Input_Register_Clone2.bit.SIGN9 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0100;	// need check Input_Register_Clone2.bit.IN9
	}
	if(Input_Register_Clone2.bit.SIGN10 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0200;	// need check Input_Register_Clone2.bit.IN10
	}
	if(Input_Register_Clone2.bit.SIGN11 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0400;	// need check Input_Register_Clone2.bit.IN11
	}
	if(Input_Register_Clone2.bit.SIGN12 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0800;	// need check Input_Register_Clone2.bit.IN12
	}
	if(Input_Register_Clone2.bit.SIGN13 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x1000;	// need check Input_Register_Clone2.bit.IN13
	}
	if(Input_Register_Clone2.bit.SIGN14 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x2000;	// need check Input_Register_Clone2.bit.IN14
	}
	if(Input_Register_Clone2.bit.SIGN15 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x4000;	// need check Input_Register_Clone2.bit.IN15
	}
	if(Input_Register_Clone2.bit.SIGN16 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x8000;	// need check Input_Register_Clone2.bit.IN16
	}
	if(Input_Register_Clone2.bit.SIGN17 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x010000;	// need check Input_Register_Clone2.bit.IN17
	}
	if(Input_Register_Clone2.bit.SIGN18 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x020000;	// need check Input_Register_Clone2.bit.IN18
	}
	if(Input_Register_Clone2.bit.SIGN19 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x040000;	// need check Input_Register_Clone2.bit.IN19
	}
	if(Input_Register_Clone2.bit.SIGN20 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x080000;	// need check Input_Register_Clone2.bit.IN20
	}
	if(Input_Register_Clone2.bit.SIGN21 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0100000;	// need check Input_Register_Clone2.bit.IN21
	}
	if(Input_Register_Clone2.bit.SIGN22 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0200000;	// need check Input_Register_Clone2.bit.IN22
	}
	if(Input_Register_Clone2.bit.SIGN23 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0400000;	// need check Input_Register_Clone2.bit.IN23
	}
	if(Input_Register_Clone2.bit.SIGN24 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x0800000;	// need check Input_Register_Clone2.bit.IN24
	}
	if(Input_Register_Clone2.bit.SIGN25 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x01000000;	// need check Input_Register_Clone2.bit.IN25
	}
	if(Input_Register_Clone2.bit.SIGN26 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x02000000;	// need check Input_Register_Clone2.bit.IN26
	}
	if(Input_Register_Clone2.bit.SIGN27 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x04000000;	// need check Input_Register_Clone2.bit.IN27
	}
	if(Input_Register_Clone2.bit.SIGN28 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x08000000;	// need check Input_Register_Clone2.bit.IN28
	}
	if(Input_Register_Clone2.bit.SIGN29 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x10000000;	// need check Input_Register_Clone2.bit.IN29
	}
	if(Input_Register_Clone2.bit.SIGN30 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x20000000;	// need check Input_Register_Clone2.bit.IN30
	}
	if(Input_Register_Clone2.bit.SIGN31 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x40000000;	// need check Input_Register_Clone2.bit.IN31
	}
	if(Input_Register_Clone2.bit.SIGN32 == TRUE)
	{
		GCodeSIPointer->Sub_CMD2 |=0x80000000;	// need check Input_Register_Clone2.bit.IN32
	}				
	return TRUE;
}

/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G103()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G103                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G103 code   (need ouput control,such as valve or delay open/close)         */
/*	Packeting G103 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G103(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	//CMD1 = 0x67  . need  control output port...
	//CMD2 bit0= 1	 =>out1
	//CMD2 bit1= 1	 =>out2
	//************************
	Uint32 si;			
	GCODE  *GCodeSIPointer;
			
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 103;//0x67

	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	GCodeSIPointer->Sub_CMD2 = Sub_CMD2;
	return 1;
}

/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G104()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G104                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G104 code   (need Fluorescence control)         */
/*	Packeting G104 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G104(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	/*
	Byte 1, Bit 0, controls Red. 0 enables, 1 disables.
	Byte 1, Bit 1, controls Green. 0 enables, 1 disables.
	Byte 1, Bit 2, controls Cyan. 0 enables, 1 disables.
	Byte 1, Bit 3, controls UV. 0 enables, 1 disables.
	Byte 1, Bit 4, controls Green/Yellow filter selection. 0 selects Yellow Filter, 1 selects Green Filter.
	Byte 1, Bit 5, controls Blue. 0 enables, 1 disables.
	Byte 1, Bit 6, controls Teal. 0 enables, 1 disables.
	*/
	//Main_CMD = 0x68(104)  . need  Fluorescence control ...
	//Sub_CMD1 bit0= 0		  =>Red 					   620~645nm	   
	//peak 7			Excitation : (nm) :  620-645, @name: Cy5/Cy5.5 AlexaFluor 660  
	
	//Sub_CMD1 bit1= 0		  =>Green			526~558nm		
	//peak5 			Excitation : (nm) :  526-558, @name: Cy3/Alexa Fluor 555  
	
	//Sub_CMD1 bit2= 0		  =>Cyan					  460~495nm   
	//peak3 			Excitation : (nm) :  460-495, @name: GFP/FITC
	
	//Sub_CMD1 bit3= 0		  =>UV							380~400nm		
	//peak 1			Excitation: (nm) :	   380-400 , @name : DAPI
	
	//Sub_CMD1 bit4= 0		  =>Yellow Filter		  560~590nm 	 
	//peak 6			Excitation : (nm) :  560-590, @name: TexRed/mCherry
	
	//Sub_CMD1 bit5= 0		  =>Blue					   426~450nm	   
	//peak 2			Excitation: (nm) :	   426-450 , @name : CFP   
	
	//Sub_CMD1 bit6= 0		  =>Teal						497~526nm		
	//peak 4			Excitation : (nm) :  497-526, @name: AlexaFluor 514 
	
	/************************
	Sub_CMD1 value set.
	0XFF- Disables All.
	0XFE- Enables Red, Disables Green,Cyan,Blue,UV,Teal.
	0XFD- Enables Green, Disables Red,Cyan,Blue,UV,Teal.
	0XFB- Enables Cyan, Disables Red,Green,Blue,UV,Teal.
	0XF7- Enables UV, Disables Red,Green,Cyan,Blue,Teal.
	0XEF- 	Selects Yellow Excitation Filter.
	0XDF- Enables Blue, Disables Red,Green,Cyan,UV,Teal.
	0XBF- Enables Teal, Disables Red,Green,Cyan,Blue,UV.
	Multi-channel(if need)
	0XDB- Enables Cyan and Blue, Disables all others.
	0XBE- Enables Red and Teal, Disables all others.
	********************************/
	/************************
	Sub_CMD1 value name match with color.
	0XFF(255)- Disables All.
	0XFE(254)- Cy5/Cy5.5 AlexaFluor 660
	0XFD(253)- ECy3/Alexa Fluor 555 
	0XFB(251)- GFP/FITC
	0XF7(247)- DAPI
	0XEF(239)- TexRed/mCherry
	0XDF(223)- CFP
	0XBF(191)- AlexaFluor 514 
	********************************/	

	
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 104;//0x68

	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	GCodeSIPointer->Sub_CMD2 = 0;
	return 1;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G105()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G105                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G105 code   (send new camera scan startpoint coordinate to DSP)         */
/*	Packeting G105 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G105(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	//Main_CMD = 0x69  need  send new coordinate to DSP...
	//Sub_CMD1 is open sign
	//Sub_CMD1 = 0x01 send new startpoint coordinate to DSP...
	//Sub_CMD1 = 0x02 send new offset coordinate to DSP...
	//************************
	Uint32 si;			
	GCODE  *GCodeSIPointer;

	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 105;//0x69

	GCodeSIPointer->Sub_CMD1 = 0;//set to 
	//Sub_CMD1 = 0x01 send new startpoint coordinate to DSP...
	//GCodeSIPointer->Sub_CMD1 = 1;
	
	//Sub_CMD1 = 0x02 send new offset coordinate to DSP...
	//GCodeSIPointer->Sub_CMD1 = 2;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	
	GCodeSIPointer->Sub_CMD2 = 0;

	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;

	//Sub_CMD1 = 0x01 send new startpoint coordinate to DSP...
	//Sub_CMD1 = 0x02 send new offset coordinate to DSP...
	#if 0
	GCodeSIPointer->Sub_CMD1 = 1;//for test
	#endif
	#if 0
	GCodeSIPointer->Sub_CMD1 = 2;//for test
	#endif

	if(GCodeSIPointer->Sub_CMD1 == 1)
	{
		GCodeSIPointer->EndPoint.Axis17_Clone_Optical_X = System_Clone.PositionCoordinate1.Axis17_Clone_Optical_X;
		GCodeSIPointer->EndPoint.Axis18_Clone_Optical_Y = System_Clone.PositionCoordinate1.Axis18_Clone_Optical_Y;
	}
	else if(GCodeSIPointer->Sub_CMD1 == 2)
	{
		GCodeSIPointer->EndPoint.Axis17_Clone_Optical_X = System_Clone.OffsetCoordinate.Axis17_Clone_Optical_X;
		GCodeSIPointer->EndPoint.Axis18_Clone_Optical_Y = System_Clone.OffsetCoordinate.Axis18_Clone_Optical_Y;
	}
		
	return 1;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G106()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G106                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G106 code   (LED votage control)         */
/*	Packeting G106 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G106(DACONVERT  Wave_Para)
{		
	//Main_CMD = 0x6a  LED output votage control...
	//Sub_CMD1 select LED channel
	//Sub_CMD1 = 0x02 LED flash light
	//Sub_CMD1 = 0x03 LED 2 
	//Sub_CMD1 = 0x04 LED 3 	

	//Sub_CMD2 votage out value(0.01)Max.=500.
	//Sub_CMD2 = 500 (5.0V)
	//Sub_CMD2 = 234 (2.34V)
	
	//************************
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 106;//0x6a

	GCodeSIPointer->Sub_CMD1 = 0;//set to 
	GCodeSIPointer->Sub_CMD1 = Wave_Para.VotageChannel;
	//Sub_CMD1 = 0x02 LED flash light
	//GCodeSIPointer->Sub_CMD1 = 2;
	
	//Sub_CMD1 = 0x03 LED 2 
	//GCodeSIPointer->Sub_CMD1 = 3;

	//Sub_CMD1 = 0x04 LED 3 	
	//GCodeSIPointer->Sub_CMD1 = 4;


	//Sub_CMD2 votage out value(0.01)Max.=500.
	//Sub_CMD2 = 500 (5.0V)
	//Sub_CMD2 = 234 (2.34V)	
	//GCodeSIPointer->Sub_CMD2 = 345;//for test
	GCodeSIPointer->Sub_CMD2 = Wave_Para.VotageValue;

	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;

	#if 0
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = AutoSign;
	GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A = Voltage_Max;
	GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans = Voltage_Start;
	GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans = CycleNum;
	GCodeSIPointer->EndPoint.Axis5 = m_StepValue;
	
	GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R = PWM_HighCount;
	GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y = PWM_LowCount;
	GCodeSIPointer->EndPoint.Axis8_PlateEmpty_Destination_X = PWM_Phase;
	#endif
// 	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = Wave_Para.AutoSign;
// 	GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A = Wave_Para.Voltage_Max;
// 	GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans = Wave_Para.Voltage_Start;
// 	GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans = Wave_Para.CycleNum;
// 	GCodeSIPointer->EndPoint.Axis5 = 0;
// 	GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R = Wave_Para.PWMHighCount;
// 	GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y = Wave_Para.PWMLowCount;
// 	GCodeSIPointer->EndPoint.Axis8_PlateEmpty_Destination_X = Wave_Para.PWMPhase;
// 	GCodeSIPointer->EndPoint.Axis9_Plate_Transfer_Z = Wave_Para.RiseTime;
// 	GCodeSIPointer->EndPoint.Axis10_Plate_Transfer_A = Wave_Para.FallTime;
// 	GCodeSIPointer->EndPoint.Axis11_PlateWaste_Source_X = Wave_Para.HighKeepTime;
// 	GCodeSIPointer->EndPoint.Axis12_PlateWaste_Source_Y = Wave_Para.LowKeepTime;


	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = Wave_Para.AutoSign;
	GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A = Wave_Para.Voltage_Max;
	GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans = Wave_Para.Voltage_Start;
	GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans = Wave_Para.CycleNum;
	GCodeSIPointer->EndPoint.Axis5 = Wave_Para.VotageChannel_Sync;
	GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R = Wave_Para.PWMHighCount;
	GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y = Wave_Para.PWMLowCount;
	GCodeSIPointer->EndPoint.Axis8_PlateEmpty_Destination_X = Wave_Para.PWMPhase;
	GCodeSIPointer->EndPoint.Axis9_Plate_Transfer_Z = Wave_Para.RiseTime;
	GCodeSIPointer->EndPoint.Axis10_Plate_Transfer_A = Wave_Para.FallTime;
	GCodeSIPointer->EndPoint.Axis11_PlateWaste_Source_X = Wave_Para.HighKeepTime;
	GCodeSIPointer->EndPoint.Axis12_PlateWaste_Source_Y = Wave_Para.LowKeepTime;

	GCodeSIPointer->EndPoint.Axis13 = Wave_Para.StartOffsetTime;
	GCodeSIPointer->EndPoint.Axis14 = Wave_Para.EndOffsetTime;
	//多传两项
	GCodeSIPointer->EndPoint.Axis15_Clone_Linear_X = Wave_Para.Voltage_Max2;
	GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y = Wave_Para.Voltage_Start2;
	GCodeSIPointer->EndPoint.Axis17_Clone_Optical_X = Wave_Para.WaveSelect;
	return 1;
}

/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G107()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G107                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G107 code   (Reset LED flash time)                                         */
/*	Packeting G107 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G107(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 107;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;//(200~2000,uint :0.1us)
	GCodeSIPointer->Sub_CMD2 = Sub_CMD2;//(200~2000,uint :0.1us)
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}

/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G108()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G108                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G108 code   (load /uload tips)                                           */
/*	Packeting G108 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G108(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	//CMD1 bit0= 1 0x01	 =>load channel 1 tip;0 uload tips
	//CMD1 bit1= 1 0x02	 =>load channel 2 tip;0 uload tips
	//CMD1 bit2= 1 0x04	 =>load channel 3 tip;0 uload tips
	//CMD1 bit3= 1 0x08	 =>load channel 4 tip;0 uload tips	
	//if(CMD1 == 0x0f),then load channel 1~4 tips. 
	
	//reserved for total 8 channel
	//CMD1 bit4= 1 0x10	 =>load channel 5 tip;0 uload tips
	//CMD1 bit5= 1 0x20	 =>load channel 6 tip;0 uload tips	
	//CMD1 bit6= 1 0x40	 =>load channel 7 tip;0 uload tips
	//CMD1 bit7= 1 0x80	 =>load channel 8 tip;0 uload tips

	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 108;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	GCodeSIPointer->Sub_CMD2 = 0;				//reserved
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G109()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G109                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G109 code    (liquid bump liquid displacement )                                         */
/*	Packeting G109 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G109(Uint32 Sub_CMD1,Uint32 Sub_CMD2)
{	
	//CMD1 bit0= 1 0x01  =>Pumping channel 1 liquid;0 no change
	//CMD1 bit1= 1 0x02  =>Pumping channel 2 liquid;0 no change
	//CMD1 bit2= 1 0x04  =>Pumping channel 3 liquid;0 no change
	//CMD1 bit3= 1 0x08  =>Pumping channel 4 liquid;0 no change	
	//reserved for total 8 channel
	//CMD1 bit4= 1 0x10  =>Pumping channel 5 liquid;0 no change
	//CMD1 bit5= 1 0x20  =>Pumping channel 6 liquid;0 no change	
	//CMD1 bit6= 1 0x40  =>Pumping channel 7 liquid;0 no change
	//CMD1 bit7= 1 0x80  =>Pumping channel 8 liquid;0 no change

	//Feedrate and EndPoint match to every channel
	
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 109;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	GCodeSIPointer->Sub_CMD2 = 0;		//reserved		
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}



/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G110()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G110                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G110 code   (Centrifuge control)                                           */
/*	Packeting G110 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G110(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32 HoldTime)
{	
	//CMD1=1 Centrifuge position mode; CMD2= Centrifuge posiotion angle(unit ud:90000=>90 degree)

	//CMD1=2 Centrifuge rotate mode;CMD2= Centrifuge Rotational speed(unit RPM :2000 RPM);
	//HoldTime = Centrifuge time(unit:s   2=2s;120=2min;1800=30min)


	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 110;
	if((Sub_CMD1 != 1)&&(Sub_CMD1 != 2))
	{
		//MessageBox(_T("Need set Centrifuge mode"));
		return FALSE;
	}
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	GCodeSIPointer->Sub_CMD2 = Sub_CMD2;
	
	if(GCodeSIPointer->Sub_CMD1 == 2)
	{
		GCodeSIPointer->HoldTime = HoldTime;
	}
	else
	{
		GCodeSIPointer->HoldTime = 0;
	}
		
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G111()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G111                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G111 code   (Vibration heating)                                         */
/*	Packeting G111 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G111(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32 HoldTime)
{	
	//CMD1 Vibration station select: if(CMD1 == 1) station 1; if(CMD1 == 2) station 2;
	//CMD2 :(bit1=1)= Vibration; (bit2=1)= heating; 
	//HoldTime = Vibration and heating time(unit:s   2=2s;120=2min;1800=30min)

	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 111;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	GCodeSIPointer->Sub_CMD2 = Sub_CMD2;
	GCodeSIPointer->HoldTime = HoldTime;


	//Feedrate match to Vibration speed uint(RPM)		
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}



/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G112()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G112                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G112 code   (Electromagnetic pulse oscillation)                                         */
/*	Packeting G112 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G112(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32 HoldTime)
{	
	//CMD1 Electromagnetic pulse oscillation station select: if(CMD1 == 1) station 1; if(CMD1 == 2) station 2;
	//CMD2 :reserved; 
	//HoldTime = Electromagnetic pulse oscillation time;(unit:s	2=2s;120=2min;1800=30min);if(CMD1 == 0),then no pulse oscillation

	Uint32 si;			
	GCODE  *GCodeSIPointer;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 112;
	GCodeSIPointer->Sub_CMD1 = Sub_CMD1;
	GCodeSIPointer->Sub_CMD2 = Sub_CMD2;
	GCodeSIPointer->HoldTime = HoldTime;	

	//Feedrate match to speed		
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
				
	return TRUE;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G113()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_G113                                     */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  G113 code   (PWM control)         */
/*	Packeting G113 code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_G113(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint16 PWM_HighCount,Uint16 PWM_LowCount,Uint16 PWM_Phase)
{	//if SubCMD1=1,then AOTF_Enable; if SubCMD2=1,then Pockels_Enable
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	Uint32 AOTF_Enable,Pockels_Enable;
	
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 113;//113
	AOTF_Enable = Sub_CMD1;
	Pockels_Enable = Sub_CMD2;
	GCodeSIPointer->Sub_CMD1 = AOTF_Enable;
	GCodeSIPointer->Sub_CMD2 = Pockels_Enable;

	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;

	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = PWM_HighCount;
	GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A = PWM_LowCount;
	GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans = PWM_Phase;
	GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans = 0;
	GCodeSIPointer->EndPoint.Axis5 = 0;
	return 1;
}


Uint16 DSPCodePacket_Code_G114(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32	HoldTime)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	GCodeSIPointer->Main_CMD = 114;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD1 = 0;
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;
	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	
	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->HoldTime = HoldTime;
	return TRUE;
}

INT40AXIS DSPCodePacket_Code_SetEndPoint(void)
{	//read from PC database
	INT40AXIS	EndPoint;//uint :um
	
	EndPoint.Axis1_Clone_Linear_Z = 20000;
	EndPoint.Axis2_Clone_Linear_A = 20000;
	EndPoint.Axis3_Clone_Optical_LensTrans = 30000;
	EndPoint.Axis4_Clone_Optical_FilterTrans = 30000;
	EndPoint.Axis5 = 20000;
	EndPoint.Axis6_PlateEmpty_Destination_Z1_R = 30000;
	EndPoint.Axis7_PlateEmpty_Destination_Y = 30000;
	EndPoint.Axis8_PlateEmpty_Destination_X = 30000;
	EndPoint.Axis9_Plate_Transfer_Z = 20000;
	EndPoint.Axis10_Plate_Transfer_A = 20000; 
	EndPoint.Axis11_PlateWaste_Source_X = 30000;
	EndPoint.Axis12_PlateWaste_Source_Y = 30000;
	EndPoint.Axis13 = 20000;
	EndPoint.Axis14 = 20000;
	EndPoint.Axis15_Clone_Linear_X = 600000;
	EndPoint.Axis16_Clone_Linear_Y = 600000;
	EndPoint.Axis17_Clone_Optical_X = 30000;
	EndPoint.Axis18_Clone_Optical_Y = 30000;
	EndPoint.Axis19_Clone_Optical_LensUpDown = 20000;
	EndPoint.Axis20_PlateEmpty_Destination_Z2_L = 30000;	
	EndPoint.Axis21_Plate_Transfer_X = 20000;
	EndPoint.Axis22_Plate_Transfer_Y = 20000;
	EndPoint.Axis23 = 20000;	
	return EndPoint;
}




/***************************************************************************/
/*  Function name: DSPCodePacket_Code_Empty()                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_Empty                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  Empty code  				         */
/*	Packeting Empty code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_Empty(void)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;

	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 0x64;//G100
	GCodeSIPointer->Sub_CMD1 = 0xff;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	
	return TRUE;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_Feedrate(GCODE  *psGCodeSIPointer,float64 Feedrate)                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_Feedrate                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  Feedrate  				         */
/*	Packeting Empty code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_Feedrate(GCODE  *psGCodeSIPointer,float64 Feedrate)
{	
	psGCodeSIPointer->Feedrate.Axis1_Clone_Linear_Z = 3;
	psGCodeSIPointer->Feedrate.Axis2_Clone_Linear_A = Feedrate;
	psGCodeSIPointer->Feedrate.Axis3_Clone_Optical_LensTrans = 10;
	psGCodeSIPointer->Feedrate.Axis4_Clone_Optical_FilterTrans = 10;
	psGCodeSIPointer->Feedrate.Axis5 = 10;
	psGCodeSIPointer->Feedrate.Axis6_PlateEmpty_Destination_Z1_R = 80;
	psGCodeSIPointer->Feedrate.Axis7_PlateEmpty_Destination_Y = Feedrate;
	psGCodeSIPointer->Feedrate.Axis8_PlateEmpty_Destination_X = 80;
	psGCodeSIPointer->Feedrate.Axis9_Plate_Transfer_Z = Feedrate;
	psGCodeSIPointer->Feedrate.Axis10_Plate_Transfer_A = Feedrate;
	psGCodeSIPointer->Feedrate.Axis11_PlateWaste_Source_X = 80;
	psGCodeSIPointer->Feedrate.Axis12_PlateWaste_Source_Y = Feedrate;
	psGCodeSIPointer->Feedrate.Axis13 = Feedrate;
	psGCodeSIPointer->Feedrate.Axis14 = Feedrate;
	psGCodeSIPointer->Feedrate.Axis15_Clone_Linear_X = Feedrate;
	psGCodeSIPointer->Feedrate.Axis16_Clone_Linear_Y = Feedrate;
	psGCodeSIPointer->Feedrate.Axis17_Clone_Optical_X = Feedrate;
	psGCodeSIPointer->Feedrate.Axis18_Clone_Optical_Y = Feedrate;
	psGCodeSIPointer->Feedrate.Axis19_Clone_Optical_LensUpDown = 3;
	psGCodeSIPointer->Feedrate.Axis20_PlateEmpty_Destination_Z2_L = Feedrate;
	psGCodeSIPointer->Feedrate.Axis21_Plate_Transfer_X = Feedrate;
	psGCodeSIPointer->Feedrate.Axis22_Plate_Transfer_Y = Feedrate;
	psGCodeSIPointer->Feedrate.Axis23 = Feedrate;
	psGCodeSIPointer->Feedrate.Axis24 = Feedrate;
	psGCodeSIPointer->Feedrate.Axis25 = Feedrate;	
		
	return TRUE;
}


/***************************************************************************/
/*  Function name: DSPCodePacket_Code_StartPoint(float64 Feedrate)                                   */
/*  Argument:NO     	                                    */
/*  Return value:TRUE/
/*  Function: DSPCodePacket_Code_StartPoint                                           */
/*    Please refer to the following document for more detials.        */                                   
/*								*/
/*  StartPoint  				         */
/*	Packeting Empty code ,PC -> DSP		*/
/***************************************************************************/
Uint16 DSPCodePacket_Code_StartPoint(GCODE  *psGCodeSIPointer)
{		
	psGCodeSIPointer->StartPoint.Axis1_Clone_Linear_Z = (MACH_COORDINATE_Clone.Axis1_Clone_Linear_Z*1000.0);
	psGCodeSIPointer->StartPoint.Axis2_Clone_Linear_A = (MACH_COORDINATE_Clone.Axis2_Clone_Linear_A*1000.0);
	psGCodeSIPointer->StartPoint.Axis3_Clone_Optical_LensTrans = (MACH_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans*1000.0);
	psGCodeSIPointer->StartPoint.Axis4_Clone_Optical_FilterTrans = (MACH_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans*1000.0);		
	psGCodeSIPointer->StartPoint.Axis5 = (MACH_COORDINATE_Clone.Axis5*1000.0);		
	psGCodeSIPointer->StartPoint.Axis6_PlateEmpty_Destination_Z1_R = (MACH_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R*1000.0);		
	psGCodeSIPointer->StartPoint.Axis7_PlateEmpty_Destination_Y = (MACH_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y*1000.0);
	psGCodeSIPointer->StartPoint.Axis8_PlateEmpty_Destination_X = (MACH_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X*1000.0);
	psGCodeSIPointer->StartPoint.Axis9_Plate_Transfer_Z = (MACH_COORDINATE_Clone.Axis9_Plate_Transfer_Z*1000.0);
	psGCodeSIPointer->StartPoint.Axis10_Plate_Transfer_A = (MACH_COORDINATE_Clone.Axis10_Plate_Transfer_A*1000.0);		
	psGCodeSIPointer->StartPoint.Axis11_PlateWaste_Source_X = (MACH_COORDINATE_Clone.Axis11_PlateWaste_Source_X*1000.0);
	psGCodeSIPointer->StartPoint.Axis12_PlateWaste_Source_Y = (MACH_COORDINATE_Clone.Axis12_PlateWaste_Source_Y*1000.0);
	psGCodeSIPointer->StartPoint.Axis13 = (MACH_COORDINATE_Clone.Axis13*1000.0);
	psGCodeSIPointer->StartPoint.Axis14 = (MACH_COORDINATE_Clone.Axis14*1000.0);		
	psGCodeSIPointer->StartPoint.Axis15_Clone_Linear_X = (MACH_COORDINATE_Clone.Axis15_Clone_Linear_X*1000.0);		
	psGCodeSIPointer->StartPoint.Axis16_Clone_Linear_Y = (MACH_COORDINATE_Clone.Axis16_Clone_Linear_Y*1000.0);		
	psGCodeSIPointer->StartPoint.Axis17_Clone_Optical_X = (MACH_COORDINATE_Clone.Axis17_Clone_Optical_X*1000.0);
	psGCodeSIPointer->StartPoint.Axis18_Clone_Optical_Y = (MACH_COORDINATE_Clone.Axis18_Clone_Optical_Y*1000.0);
	psGCodeSIPointer->StartPoint.Axis19_Clone_Optical_LensUpDown = (MACH_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown*1000.0);
	psGCodeSIPointer->StartPoint.Axis20_PlateEmpty_Destination_Z2_L = (MACH_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L*1000.0);		
	psGCodeSIPointer->StartPoint.Axis21_Plate_Transfer_X = (MACH_COORDINATE_Clone.Axis21_Plate_Transfer_X*1000.0);
	psGCodeSIPointer->StartPoint.Axis22_Plate_Transfer_Y = (MACH_COORDINATE_Clone.Axis22_Plate_Transfer_Y*1000.0);
	psGCodeSIPointer->StartPoint.Axis23 = (MACH_COORDINATE_Clone.Axis23*1000.0);
	psGCodeSIPointer->StartPoint.Axis24 = (MACH_COORDINATE_Clone.Axis24*1000.0);		
	psGCodeSIPointer->StartPoint.Axis25 = (MACH_COORDINATE_Clone.Axis25*1000.0);		
	
	return TRUE;
}


/* ���´���ʾ����װ��������*/

/***************************************************************************/
/*  Function name: DSPCodePacket_SyncStartPoint()                                   */
/*  Argument:NO     	                                    */
/*  Return value:		0:Packeting not complete ;1:Packeting Complete  */
/* ʾ����װ�˶���ͬ���˶����??*/
/*  Function: Packeting SyncStartPoint code                                          */
/***************************************************************************/
Uint16 DSPCodePacket_SyncStartPoint(void)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;

	OverallSign_Clone.FeedrateOverride = 10;
	OverallSign_Clone.NeedSendCount =0;
	OverallSign_Clone.STDSendCount = 0;
	OverallSign_Clone.NCSign= STDCODERUN;
	OverallSign_Clone.F = System_Clone.G0Speed;

	// NeedSendCount = 1;
	//G0 move to Safe Coordinate
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 0;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD2 = 0;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,System_Clone.G0Speed);

	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;

	if(GCodeSIPointer->SendCount == 1)
	{
		DSPCodePacket_Code_StartPoint(GCodeSIPointer);
	}
	else
	{
		GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;
	}	

	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = System_Clone.SafeCoordinate.Axis1_Clone_Linear_Z;
	GCodeSIPointer->EndPoint.Axis19_Clone_Optical_LensUpDown = System_Clone.SafeCoordinate.Axis19_Clone_Optical_LensUpDown;

	// NeedSendCount = 2;
	//aixs X,Y G1 move to Position Coordinate
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 0;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD2 = 0;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,System_Clone.G0Speed);
	
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;
	GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;	
	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->EndPoint.Axis15_Clone_Linear_X = System_Clone.PositionCoordinate1.Axis15_Clone_Linear_X;
	GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y = System_Clone.PositionCoordinate1.Axis16_Clone_Linear_Y;

	GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans = System_Clone.PositionCoordinate1.Axis3_Clone_Optical_LensTrans;
	GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans = System_Clone.PositionCoordinate1.Axis4_Clone_Optical_FilterTrans;	
	GCodeSIPointer->EndPoint.Axis17_Clone_Optical_X = System_Clone.PositionCoordinate1.Axis17_Clone_Optical_X;
	GCodeSIPointer->EndPoint.Axis18_Clone_Optical_Y = System_Clone.PositionCoordinate1.Axis18_Clone_Optical_Y;

	// NeedSendCount = 3;
	//aixs Z G1 move to Offset Coordinate
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 0;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD2 = 0;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,System_Clone.G0Speed);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;
	GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;	
	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = System_Clone.PositionCoordinate1.Axis1_Clone_Linear_Z+System_Clone.OffsetCoordinate.Axis1_Clone_Linear_Z;
	GCodeSIPointer->EndPoint.Axis19_Clone_Optical_LensUpDown = System_Clone.PositionCoordinate1.Axis19_Clone_Optical_LensUpDown;

	// NeedSendCount = 4;
	//aixs Z G1 move to Position Coordinate
	OverallSign_Clone.NeedSendCount++;
	si = (OverallSign_Clone.NeedSendCount-1) % STDGCODE_MOD;
	GCodeSIPointer = &GCodeBuffer[si];	
	
	GCodeSIPointer->Main_CMD = 1;
	GCodeSIPointer->Sub_CMD1 = 0;
	GCodeSIPointer->Sub_CMD2 = 0;
	
	DSPCodePacket_Code_Feedrate(GCodeSIPointer,System_Clone.G1Speed);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;
	GCodeSIPointer->StartPoint = GCodeBuffer[si-1].EndPoint;	
	GCodeSIPointer->EndPoint = GCodeSIPointer->StartPoint;
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = System_Clone.PositionCoordinate1.Axis1_Clone_Linear_Z;
	
	return 1;
}

/***************************************************************************/
/*  Function name: DSPCodePacket_CycleRun()                                   */
/*  Argument:NO     	                                    */
/*  ʾ����ѭ�����У����ڻ����ϻ����ȶ��Բ���  */
/*  Return value:		0:Packeting not complete ;1:Packeting Complete  */
/*  Function: Packeting CycleRun code                                          */
/***************************************************************************/
Uint16 DSPCodePacket_CycleRun(void)
{	
	Uint32 si;			
	GCODE  *GCodeSIPointer;

	OverallSign_Clone.FeedrateOverride = 10;
	OverallSign_Clone.NeedSendCount =0;
	OverallSign_Clone.STDSendCount = 0;
	OverallSign_Clone.NCSign= STDCODERUN;
	System_Clone.G1Speed = 50;
	OverallSign_Clone.F = System_Clone.G0Speed;

	//G Code Number 1 :Z axis go to safecoordinate(high pisition)
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = System_Clone.SafeCoordinate.Axis1_Clone_Linear_Z;//safe coordinate...

	//G Code Number 2 :X,Y axis go to safecoordinate(if need)
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis15_Clone_Linear_X = 150000;//read from database
	GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y = 200000;//read from database

	// G Code Number 3 :Linear_X and Linear_Y go to shelf 
	DSPCodePacket_Code_G4(1000);//hold on 1000 us	

	// G Code Number 4:Z axis go to clip plate coordinate...waite for PC NFC check
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = -100000;//data read from database
 
 	// G Code Number 5:Y axis go to clip plate  coordinate...
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y = 100000;//data read from database

	// G Code Number 6:Z axis down 20mm to clip plate  coordinate...
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = 120000;//data read from database

	// G Code Number 7	 //clip plate close
	Output_Register_Clone.bit.SIGN4 = 1; //other bit no change
	DSPCodePacket_Code_G103(Output_Register_Clone.all,Output_Register_Clone2.all);

	// G Code Number 8	 Z axis go to offset coordinate
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = -100000;//data read from database

	//G Code Number 9 :Y axis back to safecoordinate
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y = System_Clone.SafeCoordinate.Axis16_Clone_Linear_Y;

	//G Code Number 10 :Z axis go to safecoordinate(high pisition)
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = System_Clone.SafeCoordinate.Axis1_Clone_Linear_Z;

	//G Code Number 11 :X,Y axis go to position coordinate
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis15_Clone_Linear_X = 500000;//plate 96 well 1 position;data read from database
	GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y = 300000;//plate 96 well 1 position;data read from database

	//G Code Number 12 :Z axis go to plate 96 well 1 position offset coordinate
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = -300000;//plate 96 well 1 position;data read from database

	//G Code Number 13 :Z axis go to plate 96 well 1 position coordinate
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = -330000;//data read from database

	// G Code Number 14	 //clip plate release
	Output_Register_Clone.bit.SIGN4 = 0; //other bit no change
	DSPCodePacket_Code_G103(Output_Register_Clone.all,Output_Register_Clone2.all);

	//G Code Number 15 :Z axis go to plate 96 well 1 position offset coordinate
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = -300000;//plate 96 well 1 position;data read from database	

	//G Code Number 16 :Z axis go to safecoordinate(high pisition)
	GCodeSIPointer = &GCodeBuffer[DSPCodePacket_Code_G1()];
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = System_Clone.SafeCoordinate.Axis1_Clone_Linear_Z;
	SetEvent(event_DualCore_Clone_send);
	return 1;
}

void DIAControl_DualCore_Clone_init(void)
{
	OverallSign_Clone.ABSORG_M_Coordinate.Axis1_Clone_Linear_Z = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis2_Clone_Linear_A = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis3_Clone_Optical_LensTrans = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis4_Clone_Optical_FilterTrans = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis5 = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis6_PlateEmpty_Destination_Z1_R = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis7_PlateEmpty_Destination_Y = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis8_PlateEmpty_Destination_X = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis9_Plate_Transfer_Z = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis10_Plate_Transfer_A = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis11_PlateWaste_Source_X = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis12_PlateWaste_Source_Y = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis13 = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis14 = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis15_Clone_Linear_X = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis16_Clone_Linear_Y = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis17_Clone_Optical_X = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis18_Clone_Optical_Y = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis19_Clone_Optical_LensUpDown = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis20_PlateEmpty_Destination_Z2_L = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis21_Plate_Transfer_X = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis22_Plate_Transfer_Y = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis23 = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis24 = 0;
	OverallSign_Clone.ABSORG_M_Coordinate.Axis25 = 0;

	ABS_COORDINATE_Clone.Axis1_Clone_Linear_Z = 0;
	ABS_COORDINATE_Clone.Axis2_Clone_Linear_A = 0;
	ABS_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans = 0;
	ABS_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans = 0;
	ABS_COORDINATE_Clone.Axis5 = 0;
	ABS_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R = 0;
	ABS_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y = 0;
	ABS_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X = 0;
	ABS_COORDINATE_Clone.Axis9_Plate_Transfer_Z = 0;
	ABS_COORDINATE_Clone.Axis10_Plate_Transfer_A = 0;
	ABS_COORDINATE_Clone.Axis11_PlateWaste_Source_X = 0;
	ABS_COORDINATE_Clone.Axis12_PlateWaste_Source_Y = 0;
	ABS_COORDINATE_Clone.Axis13 = 0;
	ABS_COORDINATE_Clone.Axis14 = 0;
	ABS_COORDINATE_Clone.Axis15_Clone_Linear_X = 0;
	ABS_COORDINATE_Clone.Axis16_Clone_Linear_Y = 0;
	ABS_COORDINATE_Clone.Axis17_Clone_Optical_X = 0;
	ABS_COORDINATE_Clone.Axis18_Clone_Optical_Y = 0;
	ABS_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown = 0;
	ABS_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L = 0;
	ABS_COORDINATE_Clone.Axis21_Plate_Transfer_X = 0;
	ABS_COORDINATE_Clone.Axis22_Plate_Transfer_Y = 0;
	ABS_COORDINATE_Clone.Axis23 = 0;
	ABS_COORDINATE_Clone.Axis24 = 0;
	ABS_COORDINATE_Clone.Axis25 = 0;

	MACH_COORDINATE_Clone.Axis1_Clone_Linear_Z = 0;
	MACH_COORDINATE_Clone.Axis2_Clone_Linear_A = 0;
	MACH_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans = 0;
	MACH_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans = 0;
	MACH_COORDINATE_Clone.Axis5 = 0;
	MACH_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R = 0;
	MACH_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y = 0;
	MACH_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X = 0;
	MACH_COORDINATE_Clone.Axis9_Plate_Transfer_Z = 0;
	MACH_COORDINATE_Clone.Axis10_Plate_Transfer_A = 0;
	MACH_COORDINATE_Clone.Axis11_PlateWaste_Source_X = 0;
	MACH_COORDINATE_Clone.Axis12_PlateWaste_Source_Y = 0;
	MACH_COORDINATE_Clone.Axis13 = 0;
	MACH_COORDINATE_Clone.Axis14 = 0;
	MACH_COORDINATE_Clone.Axis15_Clone_Linear_X = 0;
	MACH_COORDINATE_Clone.Axis16_Clone_Linear_Y = 0;
	MACH_COORDINATE_Clone.Axis17_Clone_Optical_X = 0;
	MACH_COORDINATE_Clone.Axis18_Clone_Optical_Y = 0;
	MACH_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown = 0;
	MACH_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L = 0;
	MACH_COORDINATE_Clone.Axis21_Plate_Transfer_X = 0;
	MACH_COORDINATE_Clone.Axis22_Plate_Transfer_Y = 0;
	MACH_COORDINATE_Clone.Axis23 = 0;
	MACH_COORDINATE_Clone.Axis24 = 0;
	MACH_COORDINATE_Clone.Axis25 = 0;

	for (int i = 0; i < 700; i++)
	{
		TxBuf[i] = 0;
		DualCore_RxBuf[i] = 0;
/*		DualCore_ResendTxBuf[i] = 0;*/
	}

	OverallSign_Clone.sendcount = 0;
/*	OverallSign_Clone.receivecount = 0;*/
	//OverallSign_Clone
	// TODO:  在此添加额外的初始化
	Error_main_Clone.MainErrorSign = false;

	OverallSign_Clone.m_ReciveSuccess = false;
	OverallSign_Clone.PC_DSP_COM_OK = false;
	OverallSign_Clone.m_ReciveSuccessCounter = 0;
	OverallSign_Clone.m_ReciveLostCounter = 0;
	OverallSign_Clone.NCSign = NOCODERUN;
	OverallSign_Clone.NCSign_bak = NOCODERUN;
	OverallSign_Clone.CanSendCodeSign_B3 = false;
	OverallSign_Clone.CanSendCodeSign_B4 = false;
	OverallSign_Clone.LastCodeOverSign = false;
/*	DualCore_SetLastCodeOverSign(OverallSign_Clone.LastCodeOverSign);*/
	OverallSign_Clone.GetPositionSign = false;
	OverallSign_Clone.NeedGetPositionSign = false;
	OverallSign_Clone.EveryAxisCoinValid = false;
	OverallSign_Clone.FeedrateOverride = 0;
	OverallSign_Clone.ServoOn = 0;
	OverallSign_Clone.STDSendCount = 0;
	OverallSign_Clone.SPCSendCount = 0;
	OverallSign_Clone.STDReceiveDSPRunCount = 0;
	OverallSign_Clone.SPCReceiveDSPRunCount = 0;
	OverallSign_Clone.STDReceiveDSPSendCount = 0;
	OverallSign_Clone.SPCReceiveDSPSendCount = 0;
	OverallSign_Clone.JOGAxisSelect = 0;
	OverallSign_Clone.NeedParameterWriteSign = 0;
	OverallSign_Clone.ParameterWriteCompleteSign = 0;
/*	OverallSign_Clone.NeedP2PRunSign = false;*/
	OverallSign_Clone.m_OperationMode = RUN;
	OverallSign_Clone.m_OperationMode_bak = RUN;
	OverallSign_Clone.Initial_STDSign = true;
	OverallSign_Clone.Initial_SPCSign = true;

	//Position_PlateTransfering.Object2SelectNumber = 0;
	//Position_PlateTransfering.Object1SelectNumber = 0;
	//Position_PlateTransfering.Object1SelectNumber_bak = -1;

	Homing_Clone.SearchRefSign = false;
	Homing_Clone.Findzero = 0;

	Homing_Clone.NeedFindAxisSign.Axis1_Clone_Linear_Z = false;
	Homing_Clone.NeedFindAxisSign.Axis2_Clone_Linear_A = false;
	Homing_Clone.NeedFindAxisSign.Axis3_Clone_Optical_LensTrans = false;
	Homing_Clone.NeedFindAxisSign.Axis4_Clone_Optical_FilterTrans = false;
	Homing_Clone.NeedFindAxisSign.Axis5 = false;
	Homing_Clone.NeedFindAxisSign.Axis6_PlateEmpty_Destination_Z1_R = false;
	Homing_Clone.NeedFindAxisSign.Axis7_PlateEmpty_Destination_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis8_PlateEmpty_Destination_X = false;
	Homing_Clone.NeedFindAxisSign.Axis9_Plate_Transfer_Z = false;
	Homing_Clone.NeedFindAxisSign.Axis10_Plate_Transfer_A = false;

	Homing_Clone.NeedFindAxisSign.Axis11_PlateWaste_Source_X = false;
	Homing_Clone.NeedFindAxisSign.Axis12_PlateWaste_Source_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis13 = false;
	Homing_Clone.NeedFindAxisSign.Axis14 = false;
	Homing_Clone.NeedFindAxisSign.Axis15_Clone_Linear_X = false;
	Homing_Clone.NeedFindAxisSign.Axis16_Clone_Linear_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis17_Clone_Optical_X = false;
	Homing_Clone.NeedFindAxisSign.Axis18_Clone_Optical_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis19_Clone_Optical_LensUpDown = false;
	Homing_Clone.NeedFindAxisSign.Axis20_PlateEmpty_Destination_Z2_L = false;

	Homing_Clone.NeedFindAxisSign.Axis21_Plate_Transfer_X = false;
	Homing_Clone.NeedFindAxisSign.Axis22_Plate_Transfer_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis23 = false;
	Homing_Clone.NeedFindAxisSign.Axis24 = false;
	Homing_Clone.NeedFindAxisSign.Axis25 = false;

	Homing_Clone.FindRefSign.Axis1_Clone_Linear_Z = false;
	Homing_Clone.FindRefSign.Axis2_Clone_Linear_A = false;
	Homing_Clone.FindRefSign.Axis3_Clone_Optical_LensTrans = false;
	Homing_Clone.FindRefSign.Axis4_Clone_Optical_FilterTrans = false;
	Homing_Clone.FindRefSign.Axis5 = false;
	Homing_Clone.FindRefSign.Axis6_PlateEmpty_Destination_Z1_R = false;
	Homing_Clone.FindRefSign.Axis7_PlateEmpty_Destination_Y = false;
	Homing_Clone.FindRefSign.Axis8_PlateEmpty_Destination_X = false;
	Homing_Clone.FindRefSign.Axis9_Plate_Transfer_Z = false;
	Homing_Clone.FindRefSign.Axis10_Plate_Transfer_A = false;

	Homing_Clone.FindRefSign.Axis11_PlateWaste_Source_X = false;
	Homing_Clone.FindRefSign.Axis12_PlateWaste_Source_Y = false;
	Homing_Clone.FindRefSign.Axis13 = false;
	Homing_Clone.FindRefSign.Axis14 = false;
	Homing_Clone.FindRefSign.Axis15_Clone_Linear_X = false;
	Homing_Clone.FindRefSign.Axis16_Clone_Linear_Y = false;
	Homing_Clone.FindRefSign.Axis17_Clone_Optical_X = false;
	Homing_Clone.FindRefSign.Axis18_Clone_Optical_Y = false;
	Homing_Clone.FindRefSign.Axis19_Clone_Optical_LensUpDown = false;
	Homing_Clone.FindRefSign.Axis20_PlateEmpty_Destination_Z2_L = false;

	Homing_Clone.FindRefSign.Axis21_Plate_Transfer_X = false;
	Homing_Clone.FindRefSign.Axis22_Plate_Transfer_Y = false;
	Homing_Clone.FindRefSign.Axis23 = false;
	Homing_Clone.FindRefSign.Axis24 = false;
	Homing_Clone.FindRefSign.Axis25 = false;
// 
// 	FPGA_HardLimitPOS_Register_Clone_Back.all = 0;
// 	FPGA_HardLimitNEG_Register_Clone_Back.all = 0;

	FPGA_HardLimitPOS_Register_Clone.all = 0;
	FPGA_HardLimitNEG_Register_Clone.all = 0;
// 	FPGA_SoftLimitPOS_Register_Clone.all = 0;
// 	FPGA_SoftLimitNEG_Register_Clone.all = 0;
	FPGA_Home_Register_Clone.all = 0;
	FPGA_Coin_Register_Clone.all = 0;
	FPGA_ServoAlarm_Register_Clone.all = 0;
	FPGA_EncoderAlarm_Register_Clone.all = 0;
	Input_Register_Clone.all = 0;
	Input_Register_Clone2.all = 0;
	Output_Register_Clone.all = 0;
	Output_Register_Clone2.all = 0;
	MainCommand_Register_Clone.all = 0;
	MainCommand_Register_Clone2.all = 0;
	MainStatus_Register_Clone.all = 0;
	MainStatus_Register_Clone2.all = 0;

// 	m_servo_status = false;//servo off
// 	m_MotorRun_POSDown = false;//判断正转按键是否按下=false;
// 	m_MotorRun_NEGDown = false;//判断反转按键是否按下=false;

	//m_lastXAxisvalue = 0;
	//m_lastYAxisvalue = 0;
	//m_lastZAxisvalue = 0;
	// m_lastAAxisvalue = 0;




	event_DualCore_Clone_send = CreateEvent(NULL, TRUE, FALSE, NULL);

	thread_DualCore_Clone_send = CreateThread(NULL, 0, UDP_DualCore_Clone_SendThreadProc, NULL, 0, NULL);

}
/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b0()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: DSP_com_BackCMD_b0                                           */
/*	PC 	Main	 Command B0 */
/*	Common data packet ,always connect ,com. PC -> DSP		*/
/***************************************************************************/
int DualCore_DSP_com_MainCMD_b0(void) //1:run
{
	EnterCriticalSection(&dualcore_udp_lock);
	unsigned int data;
	unsigned int temp;
	unsigned int length;
	Int32  *SendUUint32Pointer1;
	int ret;

	length = SystemPrameterPackageLength_B0;	//32

	//SendUUint32Pointer1 = (unsigned int *)TxBuf;
	SendUUint32Pointer1 = DualCore_TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length;
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Dual_Core;				//站点地址
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb0;				//
	*SendUUint32Pointer1++ = 0x01;				//

	OverallSign_Clone.ServoOn = true;//for test ,always servo on...
	*SendUUint32Pointer1++ = OverallSign_Clone.ServoOn;// 1

	*SendUUint32Pointer1++ = OverallSign_Clone.NCSign;// 2

	if (OverallSign_Clone.RapidFeedrateOverride == 0)
	{
		temp = 0;
	}
	else
	{
		temp = 10;
	}
	*SendUUint32Pointer1++ = temp; // 3

	if (OverallSign_Clone.FeedrateOverride == 0)
	{
		temp = 0;
	}
	else
	{
		temp = 10;
	}
	*SendUUint32Pointer1++ = temp; // 4

	//for test LED Flash//5,6,7,8
#if 0
	if (CounterForLED++ > 10)
	{

		*SendUUint32Pointer1++ = 1;// LEDFlashCtr1 (1:ctrl LED1 Flash)
		*SendUUint32Pointer1++ = 1;// LEDFlashCtr2 (1:ctrl LED2 Flash)
		CounterForLED = 0;
		*SendUUint32Pointer1++ = 0x500;//400um
		*SendUUint32Pointer1++ = 0x500;
	}
	else
	{

		*SendUUint32Pointer1++ = 0;// LEDFlashCtr1 (1:ctrl LED1 Flash)
		*SendUUint32Pointer1++ = 0;// LEDFlashCtr2 (1:ctrl LED2 Flash)
		*SendUUint32Pointer1++ = 0;
		*SendUUint32Pointer1++ = 0;
	}
#else if	
	*SendUUint32Pointer1++ = 0;// LEDFlashCtr1 (1:ctrl LED1 Flash)
	*SendUUint32Pointer1++ = 0;// LEDFlashCtr2 (1:ctrl LED2 Flash)
	*SendUUint32Pointer1++ = 0x500;	//LEDFlashTime1 unit:um; 0xffff=always on
	*SendUUint32Pointer1++ = 0x500;	//LEDFlashTime2 unit:um; 0xffff=always on
#endif

	*SendUUint32Pointer1++ = (OverallSign_Clone.JOGAxisSelect & 0x1FFFFFF);// 25 JOGAxis 

	temp = 0;
	//printf("OverallSign_Clone.CanSendCodeSign_B4=%d,m_MotorRun_POSDown=%d,OverallSign_Clone.JOGAxisSelect=%d\n", OverallSign_Clone.CanSendCodeSign_B4, m_MotorRun_POSDown, OverallSign_Clone.JOGAxisSelect);
	if (OverallSign_Clone.CanSendCodeSign_B4 == true)
	{
// 		if (m_MotorRun_POSDown == true)
// 		{
// 			temp = 1;//正转
// 			m_MotorRun_POSDown = false;
// 		}
// 		if (m_MotorRun_NEGDown == true)
// 		{
// 			temp = 2;//反转
// 			m_MotorRun_NEGDown = false;
// 		}
		temp = 1; 
	}
	*SendUUint32Pointer1++ = temp; // 10

	*SendUUint32Pointer1++ = (m_JOGDistance * 1000); // 11

	if (OverallSign_Clone.m_OperationMode == JOG)
	{
		temp = m_JogSpeed;//mm/s
	}
	else
	{
		temp = m_speedinfo_Set;
	}
	*SendUUint32Pointer1++ = (temp); 	// 12

	temp = 0;
	if (OverallSign_Clone.NeedGetPositionSign == true)
	{
		OverallSign_Clone.Initial_STDSign = true;
		OverallSign_Clone.Initial_SPCSign = true;
		OverallSign_Clone.NeedGetPositionSign = false;
	}
	if (OverallSign_Clone.Initial_STDSign == true)
	{//bit0
		temp |= 0x0001;
	}

	if (OverallSign_Clone.Initial_SPCSign == true)
	{//bit1
		temp |= 0x0002;
	}

	if ((OverallSign_Clone.Initial_STDSign == true) || (OverallSign_Clone.Initial_SPCSign == true))
	{//bit2
		if ((OverallSign_Clone.GetPositionSign == 0)
			&& (OverallSign_Clone.STDReceiveDSPRunCount == 0)
			&& (OverallSign_Clone.SPCReceiveDSPRunCount == 0)
			&& (OverallSign_Clone.STDReceiveDSPSendCount == 0)
			&& (OverallSign_Clone.SPCReceiveDSPSendCount == 0)
			)
		{
			OverallSign_Clone.NeedGetPositionSign = false;
		}
		OverallSign_Clone.Initial_STDSign = false;
		OverallSign_Clone.Initial_SPCSign = false;
// 		OverallSign_Clone.STDSendCount = 0;
// 		OverallSign_Clone.SPCSendCount = 0;
		temp |= 0x0004;
	}
	*SendUUint32Pointer1++ = temp;// 13

	temp = 0;
	//bit0
	if (OverallSign_Clone.CameraScanSign == true)
	{
		temp |= 0x0001;			// 1:CameraScan
	}
	//bit1
	if (OverallSign_Clone.CameraStaticCaptureSign == true)
	{
		temp |= 0x0002;			// 1:CameraStaticCapture=camera stepper
	}
	//bit2
	if (OverallSign_Clone.MicroEScanSign == true)
	{
		temp |= 0x0004;			// 1:MicroEScan
	}
	//bit3
	if (OverallSign_Clone.CameraSoftTriggerLedFlashSign == true)
	{
		temp |= 0x0008;			// 1:CameraSoftTriggerLedFlashSign
	}
	*SendUUint32Pointer1++ = temp;// 14

	temp = 0;
	//if((Homing_Clone.Findzero == 0)&&(OverallSign_Clone.m_OperationMode==REFERENCE))
	if (OverallSign_Clone.m_OperationMode == REFERENCE)
	{
		if (Homing_Clone.NeedFindAxisSign.Axis1_Clone_Linear_Z)
		{
			temp |= 0x01;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis2_Clone_Linear_A)
		{
			temp |= 0x02;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis3_Clone_Optical_LensTrans)
		{
			temp |= 0x04;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis4_Clone_Optical_FilterTrans)
		{
			temp |= 0x08;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis5)
		{
			temp |= 0x010;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis6_PlateEmpty_Destination_Z1_R)
		{
			temp |= 0x020;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis7_PlateEmpty_Destination_Y)
		{
			temp |= 0x040;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis8_PlateEmpty_Destination_X)
		{
			temp |= 0x080;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis9_Plate_Transfer_Z)
		{
			temp |= 0x0100;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis10_Plate_Transfer_A)
		{
			temp |= 0x0200;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis11_PlateWaste_Source_X)
		{
			temp |= 0x0400;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis12_PlateWaste_Source_Y)
		{
			temp |= 0x0800;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis13)
		{
			temp |= 0x01000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis14)
		{
			temp |= 0x02000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis15_Clone_Linear_X)
		{
			temp |= 0x04000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis16_Clone_Linear_Y)
		{
			temp |= 0x08000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis17_Clone_Optical_X)
		{
			temp |= 0x00010000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis18_Clone_Optical_Y)
		{
			temp |= 0x00020000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis19_Clone_Optical_LensUpDown)
		{
			temp |= 0x00040000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis20_PlateEmpty_Destination_Z2_L)
		{
			temp |= 0x00080000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis21_Plate_Transfer_X)
		{
			temp |= 0x00100000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis22_Plate_Transfer_Y)
		{
			temp |= 0x00200000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis23)
		{
			temp |= 0x00400000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis24)
		{
			temp |= 0x00800000;
		}
		if (Homing_Clone.NeedFindAxisSign.Axis25)
		{
			temp |= 0x01000000;
		}

		if (Homing_Clone.SearchRefSign == true)
		{
			temp |= 0x80000000;
		}
	}
	*SendUUint32Pointer1++ = temp; // 15

	*SendUUint32Pointer1++ = 0x0000000;//16 FunctionSelectionSwitch = 0;//for test all;FunctionSelectionSwitch = 1;//for test Signal
	*SendUUint32Pointer1++ = 0x10;//17//for test
	*SendUUint32Pointer1++ = 0x11;//18//for test

	*SendUUint32Pointer1++ = 0x12;//19//for test
	*SendUUint32Pointer1++ = 0x13;//20//for test
	*SendUUint32Pointer1++ = 0x14;//21//for test
	*SendUUint32Pointer1++ = 0x15;//22//for test
	*SendUUint32Pointer1++ = 0x16;//23//for test
	*SendUUint32Pointer1++ = 0x17;//24//for test
	*SendUUint32Pointer1++ = 0x18;//25//for test
	*SendUUint32Pointer1++ = 0x19;//26//for test

	data = crc32((unsigned int*)DualCore_TxBuf, length);//length=26-1+7=32	
	*SendUUint32Pointer1++ = data;
	*SendUUint32Pointer1++ = SendUDPSequence_B0++;


	*SendUUint32Pointer1 = 0xbb;

	Int32 send_buffer[700];
	for (int i = 0; i <= (length + 3); i++)
	{
		send_buffer[i] = htonl(DualCore_TxBuf[i]);
		DualCore_ResendTxBuf[i] = send_buffer[i];
	}

	DualCore_ResendLength = length;

	GetLocalTime(&st_udpdualcore_resend);
	st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
	UDP_DualCore_WaitRespFlag = 1;
	ResetEvent(event_DualCore_UDP_Rsend);
	udpdualcore_resend_times = 0;

	ret = UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);
	if (ret < 1)
	{
		printf("DualCore send fail\n");
		UDP_DualCore_WaitRespFlag = 0;
		LeaveCriticalSection(&dualcore_udp_lock);
		return 0;
	}
	else
	{
		GetLocalTime(&st_udpdualcore_resend);
		st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
		udpdualcore_resend_times = 0;

		ret = WaitForSingleObject(event_DualCore_UDP_Rsend, INFINITE);
		if (ret != 0)
		{
			printf("event_DualCore_UDP_Rsend wait fail\n");
		}
		else
		{
			ResetEvent(event_DualCore_UDP_Rsend);
			if (UDP_DualCore_WaitRespFlag == 1)
			{
				printf("B0 packet no send resp\n");
				UDP_DualCore_WaitRespFlag = 0;
				LeaveCriticalSection(&dualcore_udp_lock);
				return 0;
			}
		}
	}

	if (SendUDPSequence_B0 > MAX_SENDUDPSEQUENCE)
		SendUDPSequence_B0 = 0;
	LeaveCriticalSection(&dualcore_udp_lock);
	return 1;
}
/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b1()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: DSP_com_MainCMD_b1                                           */
/*	PC 	Main	 Command B1 */
/*	reserved data packet ,always connect ,com. PC -> DSP		*/
/***************************************************************************/
int DualCore_DSP_com_MainCMD_b1(void) //reserved
{
	EnterCriticalSection(&dualcore_udp_lock);
	unsigned int data;
	int temp = 0;
	unsigned short length;
	Int32  *SendUUint32Pointer1;
	int ret;

	OverallSign_Clone.NeedParameterWriteSign = 1;
	printf("DualCore_DSP_com_MainCMD_b1 OverallSign_Clone.NeedParameterWriteSign = 1\n");
	length = SystemPrameterPackageLength_B1;//492	
	SendUUint32Pointer1 = DualCore_TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length;
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Dual_Core;				//站点地址
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb1;				//
	*SendUUint32Pointer1++ = 0x01;				//


	*SendUUint32Pointer1++ = System_Clone.Tsample;	//
	*SendUUint32Pointer1++ = System_Clone.SlaveMAX;//
	*SendUUint32Pointer1++ = System_Clone.TrackRunOutRangeSQR;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.acc_SET.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.dec_SET.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.VeerDeltaV;//
	*SendUUint32Pointer1++ = System_Clone.NicetyVeerDeltaV;//
	*SendUUint32Pointer1++ = System_Clone.VeerDeltaT;//
	*SendUUint32Pointer1++ = System_Clone.NicetyVeerDeltaT;//
	*SendUUint32Pointer1++ = System_Clone.LinearAxisMinUnit;//
	*SendUUint32Pointer1++ = System_Clone.LinearAxisOutUnitEQU;//
	*SendUUint32Pointer1++ = System_Clone.G0Speed;//
	*SendUUint32Pointer1++ = System_Clone.G0Speed_2;//
	*SendUUint32Pointer1++ = System_Clone.G1Speed;//
	*SendUUint32Pointer1++ = System_Clone.G1Speed_2;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeed;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeedBack;//
	*SendUUint32Pointer1++ = System_Clone.SRefBack;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeed_2;//
	*SendUUint32Pointer1++ = System_Clone.SRefSpeedBack_2;//
	*SendUUint32Pointer1++ = System_Clone.SRefBack_2;//

	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis1_Clone_Linear_Z;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis2_Clone_Linear_A;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis3_Clone_Optical_LensTrans;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis4_Clone_Optical_FilterTrans;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis5;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis6_PlateEmpty_Destination_Z1_R;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis7_PlateEmpty_Destination_Y;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis8_PlateEmpty_Destination_X;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis9_Plate_Transfer_Z;//�
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.REFStopVariable.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.RefDir.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.EncoderCheckChoose.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis11_PlateWaste_Source_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis12_PlateWaste_Source_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis13;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis14;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis15_Clone_Linear_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis16_Clone_Linear_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis17_Clone_Optical_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis18_Clone_Optical_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis19_Clone_Optical_LensUpDown;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis20_PlateEmpty_Destination_Z2_L;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis21_Plate_Transfer_X;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis22_Plate_Transfer_Y;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis23;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis24;// 
	*SendUUint32Pointer1++ = System_Clone.MotorChangeDir.Axis25;// 

	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.EncoderRDDir.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.AxisResolution.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.CoordORG.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.OffsetCoordinate.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.SafeCoordinate.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.BackCoordinate.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis1_Clone_Linear_Z;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis2_Clone_Linear_A;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis3_Clone_Optical_LensTrans;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis4_Clone_Optical_FilterTrans;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis5;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis6_PlateEmpty_Destination_Z1_R;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis7_PlateEmpty_Destination_Y;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis8_PlateEmpty_Destination_X;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis9_Plate_Transfer_Z;// �
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate1.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis1_Clone_Linear_Z;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis2_Clone_Linear_A;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis3_Clone_Optical_LensTrans;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis4_Clone_Optical_FilterTrans;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis5;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis6_PlateEmpty_Destination_Z1_R;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis7_PlateEmpty_Destination_Y;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis8_PlateEmpty_Destination_X;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis9_Plate_Transfer_Z;// 
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.PositionCoordinate2.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.MAXSpeed.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.REFStopPosition.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.SLimitPos.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis1_Clone_Linear_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis2_Clone_Linear_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis3_Clone_Optical_LensTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis4_Clone_Optical_FilterTrans;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis5;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis6_PlateEmpty_Destination_Z1_R;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis7_PlateEmpty_Destination_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis8_PlateEmpty_Destination_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis9_Plate_Transfer_Z;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis10_Plate_Transfer_A;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis11_PlateWaste_Source_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis12_PlateWaste_Source_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis13;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis14;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis15_Clone_Linear_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis16_Clone_Linear_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis17_Clone_Optical_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis18_Clone_Optical_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis19_Clone_Optical_LensUpDown;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis20_PlateEmpty_Destination_Z2_L;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis21_Plate_Transfer_X;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis22_Plate_Transfer_Y;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis23;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis24;//
	*SendUUint32Pointer1++ = System_Clone.SLimitNeg.Axis25;//

	*SendUUint32Pointer1++ = System_Clone.FunctionSelect01;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect02;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect03;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect04;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect05;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect06;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect07;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect08;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect09;// 
	*SendUUint32Pointer1++ = System_Clone.FunctionSelect10;// 

	*SendUUint32Pointer1++ = System_Clone.LEDFlashTime1;
	*SendUUint32Pointer1++ = System_Clone.LEDFlashTime2;
	*SendUUint32Pointer1++ = 102;
	*SendUUint32Pointer1++ = 103;
	*SendUUint32Pointer1++ = 104;
	*SendUUint32Pointer1++ = 105;
	*SendUUint32Pointer1++ = 106;

	data = crc32((unsigned int *)DualCore_TxBuf, length);//length=495-3
	*SendUUint32Pointer1++ = data;
	*SendUUint32Pointer1++ = SendUDPSequence_B1++;


	*SendUUint32Pointer1++ = 0xbb;

	Int32 send_buffer[700];
	for (int i = 0; i <= (length + 3); i++)
	{
		send_buffer[i] = htonl(DualCore_TxBuf[i]);
		DualCore_ResendTxBuf[i] = send_buffer[i];
	}

	DualCore_ResendLength = length;

	GetLocalTime(&st_udpdualcore_resend);
	st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
	UDP_DualCore_WaitRespFlag = 2;
	ResetEvent(event_DualCore_UDP_Rsend);
	udpdualcore_resend_times = 0;

	ret = UDP_DualCoreSendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);
	if (ret < 1)
	{
		printf("DualCore send fail\n");
		UDP_DualCore_WaitRespFlag = 0;
		LeaveCriticalSection(&dualcore_udp_lock);
		return 0;
	}
	else
	{
		GetLocalTime(&st_udpdualcore_resend);
		st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
		udpdualcore_resend_times = 0;

		ret = WaitForSingleObject(event_DualCore_UDP_Rsend, INFINITE);
		if (ret != 0)
		{
			printf("event_DualCore_UDP_Rsend wait fail\n");
		}
		else
		{
			ResetEvent(event_DualCore_UDP_Rsend);
			if (UDP_DualCore_WaitRespFlag == 2)
			{
				printf("B1 packet no send resp\n");
				UDP_DualCore_WaitRespFlag = 0;
				LeaveCriticalSection(&dualcore_udp_lock);
				return 0;
			}
		}
	}

	if (SendUDPSequence_B1 > MAX_SENDUDPSEQUENCE)
		SendUDPSequence_B1 = 0;
	LeaveCriticalSection(&dualcore_udp_lock);
	return 1;
}

/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b2()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: DSP_com_MainCMD_b2                                           */
/*	PC 	Main	 Command B2 */
/*	compensation data packet ,always connect ,com. PC -> DSP		*/
/***************************************************************************/
int DualCore_DSP_com_MainCMD_b2(void) //PC->DSP分段补偿数据通讯包协议
{
	EnterCriticalSection(&dualcore_udp_lock);
	unsigned int data;
	unsigned short length;
	Int32  *SendUINT32UPointer1;
	Uint32	temp_count, MicroEDataBaseStartCnt;
	int ret;

	OverallSign_Clone.NeedMicroEDataBaseSendSign = true;

	//OverallSign_Clone.MicroEDataBaseTotalCnt = 160;//for test

	length = SystemPrameterPackageLength_B2;//210
	SendUINT32UPointer1 = DualCore_TxBuf;
	*SendUINT32UPointer1++ = 0xaa;				//START
	*SendUINT32UPointer1++ = length;
	*SendUINT32UPointer1++ = 0xdd;				//ʶ����0xdd
	*SendUINT32UPointer1++ = IP_Dual_Core;				//站点地址
	*SendUINT32UPointer1++ = 0x01;				//
	*SendUINT32UPointer1++ = 0xb2;				//
	*SendUINT32UPointer1++ = 0x01;				//

	MicroEDataBaseStartCnt = OverallSign_Clone.MicroEDataBaseStartCnt;
	OverallSign_Clone.MicroEDataBaseEndCnt = OverallSign_Clone.MicroEDataBaseStartCnt + (SystemPrameterPackageLength_B2 - 10);

	if (OverallSign_Clone.MicroEDataBaseEndCnt > OverallSign_Clone.MicroEDataBaseTotalCnt)
	{
		OverallSign_Clone.MicroEDataBaseEndCnt = OverallSign_Clone.MicroEDataBaseTotalCnt;
	}

	*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseStartCnt;
	*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseEndCnt;
	*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseTotalCnt;

	for (temp_count = 0; temp_count < (SystemPrameterPackageLength_B2 - 10); temp_count++)
	{
		if (OverallSign_Clone.MicroEDataBaseStartCnt < OverallSign_Clone.MicroEDataBaseTotalCnt)
		{
			//#if MicroESendDataSign
			//*SendUINT32UPointer1++ = MonitorBuffer1[OverallSign_Clone.MicroEDataBaseStartCnt++].Z-MonitorBuffer1[0].Z+System_Clone_Optical.OffsetCoordinate.Z;
			//*SendUINT32UPointer1++ = MonitorBuffer1[OverallSign_Clone.MicroEDataBaseStartCnt++].Axis19_Clone_Optical_LensUpDown;
			//#else
			//for test
			//*SendUINT32UPointer1++ = OverallSign_Clone.MicroEDataBaseStartCnt;			
			//#endif

			//OverallSign_Clone.MicroEDataBaseStartCnt++;

			*SendUINT32UPointer1++ = MonitorBuffer1[OverallSign_Clone.MicroEDataBaseStartCnt++].Axis1_Clone_Linear_Z;
		}
		else
		{
			*SendUINT32UPointer1++ = 0;
		}
	}
	OverallSign_Clone.MicroEDataBaseEndCnt = OverallSign_Clone.MicroEDataBaseStartCnt;

	*SendUINT32UPointer1++ = 0;

	data = crc32((unsigned int *)DualCore_TxBuf, length);
	*SendUINT32UPointer1++ = data;
	*SendUINT32UPointer1++ = SendUDPSequence_B2++;


	*SendUINT32UPointer1++ = 0xbb;//51

	Int32 send_buffer[700];
	for (int i = 0; i <= (length + 3); i++)
	{
		send_buffer[i] = htonl(DualCore_TxBuf[i]);
		DualCore_ResendTxBuf[i] = send_buffer[i];
	}

	DualCore_ResendLength = length;

	GetLocalTime(&st_udpdualcore_resend);
	st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
	UDP_DualCore_WaitRespFlag = 3;
	ResetEvent(event_DualCore_UDP_Rsend);
	udpdualcore_resend_times = 0;

	ret = UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);
	if (ret < 1)
	{
		printf("DualCore send fail\n");
		UDP_DualCore_WaitRespFlag = 0;
		LeaveCriticalSection(&dualcore_udp_lock);
		return 0;
	}
	else
	{
		GetLocalTime(&st_udpdualcore_resend);
		st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
		udpdualcore_resend_times = 0;

		ret = WaitForSingleObject(event_DualCore_UDP_Rsend, INFINITE);
		if (ret != 0)
		{
			printf("event_DualCore_UDP_Rsend wait fail\n");
		}
		else
		{
			ResetEvent(event_DualCore_UDP_Rsend);
			if (UDP_DualCore_WaitRespFlag == 3)
			{
				printf("B2 packet no send resp\n");
				UDP_DualCore_WaitRespFlag = 0;
				LeaveCriticalSection(&dualcore_udp_lock);
				return 0;
			}
		}
	}

	if (SendUDPSequence_B2 > MAX_SENDUDPSEQUENCE)
		SendUDPSequence_B2 = 0;
	LeaveCriticalSection(&dualcore_udp_lock);

	return 1;
}
/***************************************************************************/
/*  Function name: DSP_com_MainCMD_b3()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: DSP_com_MainCMD_b3                                           */
/*	PC 	Main	 Command B3 */
/*	STD batch Code packet ,only connect if have STD batch Code,com. PC -> DSP		*/
/***************************************************************************/
int DualCore_DSP_com_MainCMD_b3(void) //PC->DSP自动模式代码组数据通讯包协议
{
	EnterCriticalSection(&dualcore_udp_lock);
	Int32 data;
	Uint32 temp;
	Int16 length;
	Int32  *SendUUint32Pointer1;
	GCODE  *GCodeSIPointer;
	Uint32 si;
	int ret;

	length = 70;

	SendUUint32Pointer1 = DualCore_TxBuf;
	*SendUUint32Pointer1++ = 0xaa;				//START
	*SendUUint32Pointer1++ = length;
	*SendUUint32Pointer1++ = 0xdd;				//ʶ����0xdd
	*SendUUint32Pointer1++ = IP_Dual_Core;				//绔欑偣鍦板潃
	*SendUUint32Pointer1++ = 0x01;				//
	*SendUUint32Pointer1++ = 0xb3;				//
	*SendUUint32Pointer1++ = 0x01;				//

	si = (OverallSign_Clone.STDSendCount - 1) % DUALCORE_STDGCODE_MOD;
	GCodeSIPointer = &DualCore_GCodeBuffer[si];
	//OverallSign_Clone.STDSendCount++;//for test B3 packet
	*SendUUint32Pointer1++ = OverallSign_Clone.STDSendCount;
	*SendUUint32Pointer1++ = GCodeSIPointer->Main_CMD;
	*SendUUint32Pointer1++ = GCodeSIPointer->Sub_CMD1;
	*SendUUint32Pointer1++ = GCodeSIPointer->Sub_CMD2;

	if (1)
	{
		//#ifdef DEBUGPROCESSCOMM
		printf("STDSendCount=%d,Main_CMD=%d,Sub_CMD1=0x%x,Sub_CMD2=0x%x\n", OverallSign_Clone.STDSendCount, GCodeSIPointer->Main_CMD, GCodeSIPointer->Sub_CMD1, GCodeSIPointer->Sub_CMD2);
		//#else
		printf("#######################Start################################\n");
		printf("STDSendCount=%d,Main_CMD=%d,Sub_CMD1=0x%x,Sub_CMD2=0x%x\n", OverallSign_Clone.STDSendCount, GCodeSIPointer->Main_CMD, GCodeSIPointer->Sub_CMD1, GCodeSIPointer->Sub_CMD2);
		printf("send robot coord:\n");
		printf("X=%d,Y=%d,Z=%d,A=%d,B=0,C=0\n", GCodeSIPointer->EndPoint.Axis21_Plate_Transfer_X, GCodeSIPointer->EndPoint.Axis22_Plate_Transfer_Y, GCodeSIPointer->EndPoint.Axis9_Plate_Transfer_Z, GCodeSIPointer->EndPoint.Axis10_Plate_Transfer_A);
		printf("VEl:X=%d,Y=%d,Z=%d,A=%d,B=0,C=0\n", GCodeSIPointer->Feedrate.Axis21_Plate_Transfer_X, GCodeSIPointer->Feedrate.Axis22_Plate_Transfer_Y, GCodeSIPointer->Feedrate.Axis9_Plate_Transfer_Z, GCodeSIPointer->Feedrate.Axis10_Plate_Transfer_A);
		printf("send source coord:\n");
		printf("X=%d,Y=%d,Z=0,A=0,B=0,C=0\n", GCodeSIPointer->EndPoint.Axis11_PlateWaste_Source_X, GCodeSIPointer->EndPoint.Axis12_PlateWaste_Source_Y);
		printf("VEl:X=%d,Y=%d,Z=0,A=0,B=0,C=0\n", GCodeSIPointer->Feedrate.Axis11_PlateWaste_Source_X, GCodeSIPointer->Feedrate.Axis12_PlateWaste_Source_Y);
		printf("send destination coord:\n");
		printf("X=%d,Y=%d,Z=%d,A=%d,B=0,C=0\n", GCodeSIPointer->EndPoint.Axis8_PlateEmpty_Destination_X, GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y, GCodeSIPointer->EndPoint.Axis20_PlateEmpty_Destination_Z2_L, GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R);
		printf("send linear coord:\n");
		printf("X=%d,Y=%d,Z=%d,A=%d,B=0,C=0\n", GCodeSIPointer->EndPoint.Axis15_Clone_Linear_X, GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y, GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z, GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A);
		printf("VEl:X=%d,Y=%d,Z=%d,A=%d,B=0,C=0\n", GCodeSIPointer->Feedrate.Axis15_Clone_Linear_X, GCodeSIPointer->Feedrate.Axis16_Clone_Linear_Y, GCodeSIPointer->Feedrate.Axis1_Clone_Linear_Z, GCodeSIPointer->Feedrate.Axis2_Clone_Linear_A);
		printf("send optical coord:\n");
		printf("X=%d,Y=%d,Z=%d,A=%d,B=%d,C=0\n", GCodeSIPointer->EndPoint.Axis17_Clone_Optical_X, GCodeSIPointer->EndPoint.Axis18_Clone_Optical_Y, GCodeSIPointer->EndPoint.Axis19_Clone_Optical_LensUpDown, GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans, GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans);
		printf("Vel X=%d,Y=%d,Z=%d,A=%d,B=%d,C=0\n", GCodeSIPointer->Feedrate.Axis17_Clone_Optical_X, GCodeSIPointer->Feedrate.Axis18_Clone_Optical_Y, GCodeSIPointer->Feedrate.Axis19_Clone_Optical_LensUpDown, GCodeSIPointer->Feedrate.Axis3_Clone_Optical_LensTrans, GCodeSIPointer->Feedrate.Axis4_Clone_Optical_FilterTrans);
		printf("************************End*******************************\n");
		//#endif	
	}

	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis5;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis8_PlateEmpty_Destination_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis9_Plate_Transfer_Z;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis10_Plate_Transfer_A;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis11_PlateWaste_Source_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis12_PlateWaste_Source_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis13;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis14;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis15_Clone_Linear_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis16_Clone_Linear_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis17_Clone_Optical_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis18_Clone_Optical_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis19_Clone_Optical_LensUpDown;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis20_PlateEmpty_Destination_Z2_L;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis21_Plate_Transfer_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis22_Plate_Transfer_Y;
	//GCodeSIPointer->EndPoint.Axis23 = 123456;//for test
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis23;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis24;
	*SendUUint32Pointer1++ = GCodeSIPointer->EndPoint.Axis25;

	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis1_Clone_Linear_Z;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis2_Clone_Linear_A;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis3_Clone_Optical_LensTrans;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis4_Clone_Optical_FilterTrans;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis5;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis6_PlateEmpty_Destination_Z1_R;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis7_PlateEmpty_Destination_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis8_PlateEmpty_Destination_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis9_Plate_Transfer_Z;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis10_Plate_Transfer_A;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis11_PlateWaste_Source_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis12_PlateWaste_Source_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis13;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis14;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis15_Clone_Linear_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis16_Clone_Linear_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis17_Clone_Optical_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis18_Clone_Optical_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis19_Clone_Optical_LensUpDown;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis20_PlateEmpty_Destination_Z2_L;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis21_Plate_Transfer_X;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis22_Plate_Transfer_Y;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis23;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis24;
	*SendUUint32Pointer1++ = GCodeSIPointer->Feedrate.Axis25;

	//GCodeSIPointer->HoldTime = 334455;//for test
	*SendUUint32Pointer1++ = GCodeSIPointer->HoldTime;
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved
	*SendUUint32Pointer1++ = 0;//reserved	
	*SendUUint32Pointer1++ = 0;//reserved 
	*SendUUint32Pointer1++ = 0;//reserved 
	*SendUUint32Pointer1++ = 0;//reserved 
	*SendUUint32Pointer1++ = 0;//reserved 	

	data = crc32((unsigned int *)DualCore_TxBuf, length);//length=64-1+7=70
	*SendUUint32Pointer1++ = data;
	*SendUUint32Pointer1++ = SendUDPSequence_B3++;


	*SendUUint32Pointer1 = 0xbb;

	Int32 send_buffer[700];
	for (int i = 0; i <= (length + 3); i++)
	{
		send_buffer[i] = htonl(DualCore_TxBuf[i]);
		DualCore_ResendTxBuf[i] = send_buffer[i];
	}

	DualCore_ResendLength = length;

	GetLocalTime(&st_udpdualcore_resend);
	st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
	UDP_DualCore_WaitRespFlag = 4;
	ResetEvent(event_DualCore_UDP_Rsend);
	udpdualcore_resend_times = 0;

	ret = UDP_SendData((char*)send_buffer, (length + 4) * 4, IP_Dual_Core);
	if (ret < 1)
	{
		printf("DualCore send fail\n");
		UDP_DualCore_WaitRespFlag = 0;
		LeaveCriticalSection(&dualcore_udp_lock);
		return 0;
	}
	else
	{
		GetLocalTime(&st_udpdualcore_resend);
		st_udpdualcore_lastms = st_udpdualcore_resend.wSecond * 1000 + st_udpdualcore_resend.wMilliseconds;
		udpdualcore_resend_times = 0;

		ret = WaitForSingleObject(event_DualCore_UDP_Rsend, INFINITE);
		if (ret != 0)
		{
			printf("event_DualCore_UDP_Rsend wait fail\n");
		}
		else
		{
			ResetEvent(event_DualCore_UDP_Rsend);
			if (UDP_DualCore_WaitRespFlag == 4)
			{
				printf("B3 packet no send resp\n");
				UDP_DualCore_WaitRespFlag = 0;
				LeaveCriticalSection(&dualcore_udp_lock);
				return 0;
			}
		}
	}

	if (SendUDPSequence_B3 > MAX_SENDUDPSEQUENCE)
		SendUDPSequence_B3 = 0;
	LeaveCriticalSection(&dualcore_udp_lock);
	return 1;
}
int DualCore_Clone_SendB3Packet(void)
{
	//发送B3包开始
	if (Error_main_Clone.MainErrorSign == true)
	{
		OverallSign_Clone.packsendflag = 0;//
		OverallSign_Clone.packsendfinishflag = 0;
		ResetEvent(event_DualCore_Clone_send);
		printf("Error_main_Clone.MainErrorSign == true\n");
		return 0;
	}

	if (OverallSign_Clone.NCSign != STDCODERUN)
	{
		OverallSign_Clone.packsendflag = 0;//
		OverallSign_Clone.packsendfinishflag = 0;
		ResetEvent(event_DualCore_Clone_send);
		printf("OverallSign_Clone.NCSign != STDCODERUN,OverallSign_Clone.NCSign=%d\n", OverallSign_Clone.NCSign);
		return 0;
	}
	if (OverallSign_Clone.NCSign_bak != OverallSign_Clone.NCSign)
	{
		Sleep(100);
		if (OverallSign_Clone.NCSign_bak != OverallSign_Clone.NCSign)
		{
			OverallSign_Clone.packsendflag = 0;//
			OverallSign_Clone.packsendfinishflag = 0;
			ResetEvent(event_DualCore_Clone_send);
			printf("OverallSign_Clone.NCSign_bak != OverallSign_Clone.NCSign,NCSign_bak=%d,NCSign=%d\n", OverallSign_Clone.NCSign_bak, OverallSign_Clone.NCSign);
			return 0;
		}
	}
	if (OverallSign_Clone.CanSendCodeSign_B3 == 0)
	{
		Sleep(100);
		if (OverallSign_Clone.CanSendCodeSign_B3 == 0)
		{
			//OverallSign_Clone.packsendflag = 0;//
			//OverallSign_Clone.packsendfinishflag = 0;
			//ResetEvent(event_DualCore_Clone_send);
			printf("OverallSign_Clone.CanSendCodeSign_B3 == 0\n");
			Sleep(100);
			return 0;
		}
	}
	if (OverallSign_Clone.GetPositionSign == false)
	{
		OverallSign_Clone.packsendflag = 0;//
		OverallSign_Clone.packsendfinishflag = 0;
		ResetEvent(event_DualCore_Clone_send);
		printf("OverallSign_Clone.GetPositionSign == false\n");
		return 0;
	}

	if ((OverallSign_Clone.STDReceiveDSPSendCount < OverallSign_Clone.NeedSendCount)
		&& (OverallSign_Clone.STDSendCount < OverallSign_Clone.NeedSendCount))
	{
		if (OverallSign_Clone.STDReceiveDSPSendCount == OverallSign_Clone.STDSendCount)
		{
			OverallSign_Clone.STDSendCount++;
		}
		else
		{
			Sleep(100);
			if (OverallSign_Clone.STDReceiveDSPSendCount == OverallSign_Clone.STDSendCount)
			{
				OverallSign_Clone.STDSendCount++;
			}
		}
		DualCore_DSP_com_MainCMD_b3();
	}
	else
	{
		if ((OverallSign_Clone.packsendfinishflag & 0x00000004) == 0x00000004)
		{
			if (OverallSign_Clone.STDReceiveDSPSendCount == OverallSign_Clone.STDSendCount)
			{
				OverallSign_Clone.packsendflag = 0;//
				//OverallSign_PlateTransfering.packsendfinishflag=0;
				ResetEvent(event_DualCore_Clone_send);
				printf("resetevent OverallSign_Clone.STDSendCount=%d\n", OverallSign_Clone.STDSendCount);
			}
			else
			{
				if (OverallSign_Clone.STDSendCount > 0)
					DualCore_DSP_com_MainCMD_b3();
				Sleep(100);
			}

		}
	}

	return 1;
}
/***************************************************************************/
/*  Function name: OnBnClickedReset()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
/*  Function: OnBnClickedReset                                           */
/*    Please refer to the following document for more detials.        */
/*								*/
/***************************************************************************/
void OnBnClickedReset()
{
	// TODO:  在此添加额外的初始化

	LensCompensationSign = 0;
	Error_main_Clone.MainErrorSign = false;

	OverallSign_Clone.m_ReciveSuccess = false;
	OverallSign_Clone.m_ReciveSuccessCounter = 0;
	OverallSign_Clone.NCSign = NOCODERUN;
	OverallSign_Clone.NCSign_bak = NOCODERUN;
	OverallSign_Clone.NeedSendCount = 0;
	OverallSign_Clone.CanSendCodeSign_B3 = false;
	OverallSign_Clone.CanSendCodeSign_B4 = false;
	OverallSign_Clone.LastCodeOverSign = false;
	OverallSign_Clone.GetPositionSign = false;
	OverallSign_Clone.NeedGetPositionSign = false;
	OverallSign_Clone.EveryAxisCoinValid = false;
	OverallSign_Clone.FeedrateOverride = 0;
	OverallSign_Clone.STDSendCount = 0;
	OverallSign_Clone.SPCSendCount = 0;
	OverallSign_Clone.STDReceiveDSPRunCount = 0;
	OverallSign_Clone.SPCReceiveDSPRunCount = 0;
	OverallSign_Clone.STDReceiveDSPSendCount = 0;
	OverallSign_Clone.SPCReceiveDSPSendCount = 0;
	OverallSign_Clone.JOGAxisSelect = 0;
	OverallSign_Clone.m_OperationMode = RUN;
	OverallSign_Clone.Initial_STDSign = true;
	OverallSign_Clone.Initial_SPCSign = true;

	OverallSign_Clone.CameraScanSign = false;
	OverallSign_Clone.MicroEScanSign = false;
	OverallSign_Clone.CameraStaticCaptureSign = false;
	OverallSign_Clone.CameraSoftTriggerLedFlashSign = false;

	Homing_Clone.NeedFindAxisSign.Axis1_Clone_Linear_Z = false;
	Homing_Clone.NeedFindAxisSign.Axis2_Clone_Linear_A = false;
	Homing_Clone.NeedFindAxisSign.Axis3_Clone_Optical_LensTrans = false;
	Homing_Clone.NeedFindAxisSign.Axis4_Clone_Optical_FilterTrans = false;
	Homing_Clone.NeedFindAxisSign.Axis5 = false;
	Homing_Clone.NeedFindAxisSign.Axis6_PlateEmpty_Destination_Z1_R = false;
	Homing_Clone.NeedFindAxisSign.Axis7_PlateEmpty_Destination_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis8_PlateEmpty_Destination_X = false;
	Homing_Clone.NeedFindAxisSign.Axis9_Plate_Transfer_Z = false;
	Homing_Clone.NeedFindAxisSign.Axis10_Plate_Transfer_A = false;
	Homing_Clone.NeedFindAxisSign.Axis11_PlateWaste_Source_X = false;
	Homing_Clone.NeedFindAxisSign.Axis12_PlateWaste_Source_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis13 = false;
	Homing_Clone.NeedFindAxisSign.Axis14 = false;
	Homing_Clone.NeedFindAxisSign.Axis15_Clone_Linear_X = false;
	Homing_Clone.NeedFindAxisSign.Axis16_Clone_Linear_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis17_Clone_Optical_X = false;
	Homing_Clone.NeedFindAxisSign.Axis18_Clone_Optical_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis19_Clone_Optical_LensUpDown = false;
	Homing_Clone.NeedFindAxisSign.Axis20_PlateEmpty_Destination_Z2_L = false;
	Homing_Clone.NeedFindAxisSign.Axis21_Plate_Transfer_X = false;
	Homing_Clone.NeedFindAxisSign.Axis22_Plate_Transfer_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis23 = false;
	Homing_Clone.NeedFindAxisSign.Axis24 = false;
	Homing_Clone.NeedFindAxisSign.Axis25 = false;

	FPGA_Coin_Register_Clone.all = 0;
	FPGA_ServoAlarm_Register_Clone.all = 0;
	FPGA_EncoderAlarm_Register_Clone.all = 0;
	FPGA_HardLimitPOS_Register_Clone.all = 0;
	FPGA_HardLimitNEG_Register_Clone.all = 0;
	SoftLimitPOS_Register_Clone.all = 0;
	SoftLimitNEG_Register_Clone.all = 0;
	TrackRunOver_Register_Clone.all = 0;
	InterpolationOver_Register_Clone.all = 0;
	CompensationDataCheck_Register_Clone.all = 0;

	//System_Clone.FunctionSelect07 = 1;   //下位打印B1，B2,B3,
	//System_Clone.FunctionSelect08 = 1;//下位打印B0


	DSP_com_MainCMD_50();
	DSP_com_MainCMD_51();
	DSP_com_MainCMD_52();
	DSP_com_MainCMD_b1();
}
Uint16 CheckAllAxisInStaticState_Clone(void)
{
	if ((OverallSign_Clone.STDSendCount > 0) &&
		(OverallSign_Clone.NeedSendCount == OverallSign_Clone.STDSendCount) &&
		(OverallSign_Clone.STDSendCount == OverallSign_Clone.STDReceiveDSPRunCount) &&
		(OverallSign_Clone.STDSendCount == OverallSign_Clone.STDReceiveDSPSendCount) &&
		(OverallSign_Clone.LastCodeOverSign == 1) &&
		(OverallSign_Clone.EveryAxisCoinValid == 1)
		)
	{
		return true;
	}
	else
	{
		return false;
	}

}

void DIAControl_DualCore_Clone_Reset()
{
	// TODO:  在此添加额外的初始化
	int ntmer;
	Error_main_Clone.MainErrorSign = false;

	OverallSign_Clone.m_ReciveSuccess = false;
	OverallSign_Clone.m_ReciveSuccessCounter = 0;
	OverallSign_Clone.m_ReciveLostCounter = 0;
	OverallSign_Clone.NCSign = NOCODERUN;
	OverallSign_Clone.NCSign_bak = NOCODERUN;
	OverallSign_Clone.NeedSendCount = 0;
	OverallSign_Clone.CanSendCodeSign_B3 = false;
	OverallSign_Clone.CanSendCodeSign_B4 = false;
	OverallSign_Clone.LastCodeOverSign = false;
	OverallSign_Clone.CameraScanSign = false;
	//DualCore_SetLastCodeOverSign(OverallSign_Clone.LastCodeOverSign);
	OverallSign_Clone.GetPositionSign = false;
	OverallSign_Clone.NeedGetPositionSign = false;
	OverallSign_Clone.EveryAxisCoinValid = false;
	OverallSign_Clone.FeedrateOverride = 0;
	OverallSign_Clone.STDSendCount = 0;
	OverallSign_Clone.SPCSendCount = 0;
	OverallSign_Clone.STDReceiveDSPRunCount = 0;
	OverallSign_Clone.SPCReceiveDSPRunCount = 0;
	OverallSign_Clone.STDReceiveDSPSendCount = 0;
	OverallSign_Clone.SPCReceiveDSPSendCount = 0;
	OverallSign_Clone.JOGAxisSelect = 0;
	OverallSign_Clone.NeedParameterWriteSign = 0;
	OverallSign_Clone.ParameterWriteCompleteSign = 0;
	OverallSign_Clone.NeedP2PRunSign = false;
	OverallSign_Clone.m_OperationMode = RUN;
	OverallSign_Clone.Initial_STDSign = true;
	OverallSign_Clone.Initial_SPCSign = true;

	Homing_Clone.NeedFindAxisSign.Axis1_Clone_Linear_Z = false;
	Homing_Clone.NeedFindAxisSign.Axis2_Clone_Linear_A = false;
	Homing_Clone.NeedFindAxisSign.Axis3_Clone_Optical_LensTrans = false;
	Homing_Clone.NeedFindAxisSign.Axis4_Clone_Optical_FilterTrans = false;
	Homing_Clone.NeedFindAxisSign.Axis5 = false;
	Homing_Clone.NeedFindAxisSign.Axis6_PlateEmpty_Destination_Z1_R = false;
	Homing_Clone.NeedFindAxisSign.Axis7_PlateEmpty_Destination_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis8_PlateEmpty_Destination_X = false;
	Homing_Clone.NeedFindAxisSign.Axis9_Plate_Transfer_Z = false;
	Homing_Clone.NeedFindAxisSign.Axis10_Plate_Transfer_A = false;

	Homing_Clone.NeedFindAxisSign.Axis11_PlateWaste_Source_X = false;
	Homing_Clone.NeedFindAxisSign.Axis12_PlateWaste_Source_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis13 = false;
	Homing_Clone.NeedFindAxisSign.Axis14 = false;
	Homing_Clone.NeedFindAxisSign.Axis15_Clone_Linear_X = false;
	Homing_Clone.NeedFindAxisSign.Axis16_Clone_Linear_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis17_Clone_Optical_X = false;
	Homing_Clone.NeedFindAxisSign.Axis18_Clone_Optical_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis19_Clone_Optical_LensUpDown = false;
	Homing_Clone.NeedFindAxisSign.Axis20_PlateEmpty_Destination_Z2_L = false;

	Homing_Clone.NeedFindAxisSign.Axis21_Plate_Transfer_X = false;
	Homing_Clone.NeedFindAxisSign.Axis22_Plate_Transfer_Y = false;
	Homing_Clone.NeedFindAxisSign.Axis23 = false;
	Homing_Clone.NeedFindAxisSign.Axis24 = false;
	Homing_Clone.NeedFindAxisSign.Axis25 = false;

// 	MovePause_Flag = 0;
// 	m_MotorRun_POSDown = false;//判断正转按键是否按下=false;
// 	m_MotorRun_NEGDown = false;//判断反转按键是否按下=false;
	OverallSign_Clone.packsendflag |= 0x00000001;
	SetEvent(event_DualCore_Clone_send);
	/*
	Clone_Linear_DSP_com_MainCMD_50();
	Clone_Linear_DSP_com_MainCMD_51();
	Clone_Linear_DSP_com_MainCMD_52();
	*/
	if (DualCore_DSP_com_MainCMD_b1() < 1)
	{
		printf("Send DualCore_DSP_com_MainCMD_b1 fail\n");
	}
	ntmer = 0;
	while (1)
	{
		Sleep(500);
		if ((OverallSign_Clone.NeedParameterWriteSign == 1) && (OverallSign_Clone.ParameterWriteCompleteSign == 0))
		{
			//PlateTransfering_DSP_com_MainCMD_50();
			//PlateTransfering_DSP_com_MainCMD_51();
			//PlateTransfering_DSP_com_MainCMD_52();
			printf("OverallSign_Clone.NeedParameterWriteSign=%d,OverallSign_Clone.ParameterWriteCompleteSign=%d\n", OverallSign_Clone.NeedParameterWriteSign, OverallSign_Clone.ParameterWriteCompleteSign);
			if (DualCore_DSP_com_MainCMD_b1() < 1)
			{
				printf("DualCore_Clone 写参数失败\n");
				//return ;
			}
			ntmer++;
			if (ntmer > 3)
			{
				//DualCore_Clone_Stop();
				printf("DualCore_Clone 参数写入未完成\n");
			}
		}
		else
		{
			//if ((OverallSign_Clone.NeedParameterWriteSign == 0) && (OverallSign_Clone.ParameterWriteCompleteSign == 1))
			//{
			break;
			//}
			//else
			//{
			//	printf("Waiting DualCore_Clone 参数写入完成\n");
			//}
		}
	}

	Error_main_Clone.Resetlimit = 1;
	if (Homing_Clone.Findzero == 0)
	{
		//Send_NoticeWarningInfo_To_Queue("还未找到参考点\n", -1, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("还未找到参考点\n", 0, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("还未找到参考点\n", 1, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("还未找到参考点\n", 2, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("还未找到参考点\n", 3, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("还未找到参考点\n", 4, 0, 1);
	}
	else
	{
		//Send_NoticeWarningInfo_To_Queue("已找到参考点\n", -1, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 0, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 1, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 2, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 3, 0, 1);
// 		Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 4, 0, 1);
	}
}
static DWORD WINAPI UDP_DualCore_Clone_SendThreadProc(void* pCtrl)
{
	DWORD ret;
	int ncount;

	while (1)
	{
// 		if (OverallSign_Clone.PC_DSP_COM_OK)
// 		{
// 			//DualCore_DSP_com_MainCMD_b0();
// 			DSP_com_MainCMD_b0();
// 			Sleep(10);
// 			//SetEvent(event_DualCore_Clone_send);
// 			//		printf("DualCore_Clone b0 trigger event\n");
// 		}
		ret = WaitForSingleObject(event_DualCore_Clone_send, 5000);

		if (ret == 0)
		{
			
			if (1)
			{

// 				if (Error_main_Clone.MainErrorSign == true)
// 				{
// 					return -1;
// 				}
// 				if (OverallSign_Clone.NCSign != STDCODERUN)
// 				{
// 					return -1;
// 				}
#if 0
				if ((OverallSign_Clone.STDReceiveDSPSendCount < OverallSign_Clone.NeedSendCount)
					//&&(OverallSign_Clone.STDSendCount < OverallSign_Clone.NeedSendCount)
					)
				{
					if (OverallSign_Clone.STDReceiveDSPSendCount == OverallSign_Clone.STDSendCount)
					{
						OverallSign_Clone.STDSendCount++;
					}
					DSP_com_MainCMD_b3();
				}
#endif

				//SetTimer(nIDEvent, 20, NULL);
	//			SetEvent(event_DualCore_Clone_send);
			}
			// 				if (OverallSign_Clone.CanSendCodeSign_B3 == 0)
			// 				{
			// 					return -1;
			// 				}
			// 				if (OverallSign_Clone.GetPositionSign == false)
			// 				{
			// 					return -1;
			// 				}
			
			/*		ResetEvent(event_DualCore_Clone_send);*/
			//KillTimer(nIDEvent);			
			//SetTimer(nIDEvent, 20, NULL);

			if ((OverallSign_Clone.packsendflag & 0x00000004) == 0x00000004)
			{
				if (OverallSign_Clone.PC_DSP_COM_OK)
				{
					//DualCore_Clone_SendB3Packet();
					if ((OverallSign_Clone.STDReceiveDSPSendCount < OverallSign_Clone.NeedSendCount)
						//&&(OverallSign_Clone.STDSendCount < OverallSign_Clone.NeedSendCount)
						)
					{
						if (OverallSign_Clone.STDReceiveDSPSendCount == OverallSign_Clone.STDSendCount)
						{
							OverallSign_Clone.STDSendCount++;
						}
						DSP_com_MainCMD_b3();
						OverallSign_Clone.packsendflag = 0x00000001;
						printf("DualCore_Clone b3 trigger event\n");
					}
				}
				else
				{
					printf("IP_DualCore_Clone can not communicate\n");
				}
				SetEvent(event_DualCore_Clone_send);
			}			
			if ((OverallSign_Clone.packsendflag & 0x00000002) == 0x00000002)
			{
				if (OverallSign_Clone.PC_DSP_COM_OK)
				{
					if (Error_main_Clone.MainErrorSign == true)
					{
						OverallSign_Clone.packsendflag = 0;//
						ResetEvent(event_DualCore_Clone_send);
						continue;
					}
					if (OverallSign_Clone.MicroEScanSign == 1)
					{
						Sleep(5);
						continue;
					}

					
					if (MonitorBuffer1Cnt == 0)
					{
						Sleep(5);
						continue;
					}
					
					if (OverallSign_Clone.MicroEDataBaseStartCnt_DSP < OverallSign_Clone.MicroEDataBaseStartCnt)
					{
						ncount++;
						if (ncount > 1)
						{
							OverallSign_Clone.MicroEDataBaseStartCnt = OverallSign_Clone.MicroEDataBaseStartCnt_DSP;
						}
						else
						{
							Sleep(100);
						}
						Sleep(10);
					}
					if ((OverallSign_Clone.MicroEDataBaseStartCnt < OverallSign_Clone.MicroEDataBaseTotalCnt))
					{
						ncount = 0;
						DSP_com_MainCMD_b2();
						//	printf("Clone_Optical b2 trigger event\n");
					}
					if (OverallSign_Clone.MicroEDataBaseStartCnt_DSP != 0)
					{
						printf("Get some thing!!\n");
					}
					if ((OverallSign_Clone.MicroEDataBaseStartCnt_DSP == OverallSign_Clone.MicroEDataBaseTotalCnt))
					{
						printf("Receive down!!\n");
						OverallSign_Clone.MicroEDataBaseStartCnt_DSP = 0; //下次发送做准备
						OverallSign_Clone.packsendflag = 0;//
						ResetEvent(event_DualCore_Clone_send);
					}

				}
				else
				{
					printf("IP_Clone_Optical can not communicate\n");
				}
			}
			else
			{
				if (OverallSign_Clone.m_OperationMode != REFERENCE)
				{
					ResetEvent(event_DualCore_Clone_send);
				}
				else
				{
					if (Homing_Clone.Findzero == 0)
					{
						//DIAControl_DualCore_Clone_Servoon();
						//DualCore_Clone_HomingRun();
						Sleep(1);
					}
					else
					{
						ResetEvent(event_DualCore_Clone_send);
						DIAControl_DualCore_Clone_Reset();
						//printf("ResetEvent(event_Clone_Optical_send)\n");
					}
				}
			}

			//printf("trigger event\n");
		}
		else if (ret == 258)
		{
// 			if (Homing_Clone.Findzero == 1)
// 			{
// 				if (((OverallSign_Clone.NeedSendCount > OverallSign_Clone.STDSendCount) && (OverallSign_Clone.packsendflag > 0)) || ((OverallSign_Clone.packsendflag & 0x00000001) == 0x00000001))
// 				{
 					SetEvent(event_DualCore_Clone_send);
// 					printf("+++ timeout SetEvent event_DualCore_Clone_send +++\n");
// 				}
// 			}
			//if(OverallSign_Clone.PC_DSP_COM_OK)
			//DualCore_DSP_com_MainCMD_b0();
			//if (OverallSign_PlateTransfering.LastCodeOverSign == 1)
			//{
			//}
		}

	}
}


int DualCore_DSP_com_BackCMD_b0(void)
{
	unsigned int temp, temp1;
	double xvalue, yvalue, zvalue;
	CString str, str1;
	int *UUint32Pointer1;

	short i;
	int data32;

	UUint32Pointer1 = &DualCore_RxBuf[7];

	//版本日期返回 
	str.Empty();
	temp = *UUint32Pointer1++;
	str.Format(_T("%2d"), temp);
	str1 = _T("") + str + _T("年");

	temp = *UUint32Pointer1++;
	str.Format(_T("%2d"), temp);
	str1 = str1 + str + _T("月");

	temp = *UUint32Pointer1++;
	str.Format(_T("%2d"), temp);
	str1 = str1 + str + _T("日");
	str = str1;
	//GetDlgItem(IDC_Clone_DSP_Time)->SetWindowText(str);

	str.Empty();
	temp = *UUint32Pointer1++;
	str.Format(_T("%d"), temp);
	str = _T("V01.00.0") + str;
	//GetDlgItem(IDC_Clone_DSP_Version)->SetWindowText(str);

	temp = *UUint32Pointer1++;
	temp1 = temp & 0x03;
	if (temp1 == 1)
	{
		OverallSign_Clone.NCSign_bak = STDCODERUN;
	}
	else if (temp1 == 2)
	{
		OverallSign_Clone.NCSign_bak = SPCCODERUN;
	}
	else
	{
		OverallSign_Clone.NCSign_bak = NOCODERUN;
	}

	OverallSign_Clone.STDReceiveDSPRunCount = *UUint32Pointer1++;// 1
	OverallSign_Clone.SPCReceiveDSPRunCount = *UUint32Pointer1++;// 2
	if ((OverallSign_Clone.m_OperationMode == JOG) || (OverallSign_Clone.m_OperationMode == REFERENCE))
	{
		OverallSign_Clone.SPCSendCount = OverallSign_Clone.SPCReceiveDSPRunCount;
		OverallSign_Clone.SPCReceiveDSPSendCount = OverallSign_Clone.SPCReceiveDSPRunCount;
	}

	OverallSign_Clone.STDReceiveDSPSendCount = *UUint32Pointer1++;// 3
	OverallSign_Clone.SPCReceiveDSPSendCount = *UUint32Pointer1++;// 4
	OverallSign_Clone.MicroEDataBaseStartCnt_DSP = *UUint32Pointer1++;//10	

	UUint32Pointer1++;//reserved 11
	UUint32Pointer1++;//Feedrate.Axis1 12
	UUint32Pointer1++;//13
	UUint32Pointer1++;//14
	UUint32Pointer1++;//15
	UUint32Pointer1++;//16
	UUint32Pointer1++;//17
	UUint32Pointer1++;//18
	UUint32Pointer1++;//19
	UUint32Pointer1++;//20
	UUint32Pointer1++;//21

	UUint32Pointer1++;//22
	UUint32Pointer1++;//23
	UUint32Pointer1++;//24
	UUint32Pointer1++;//25
	UUint32Pointer1++;//26
	UUint32Pointer1++;//27
	UUint32Pointer1++;//28
	UUint32Pointer1++;//29
	UUint32Pointer1++;//30
	UUint32Pointer1++;//31

	UUint32Pointer1++;//32
	UUint32Pointer1++;//33
	UUint32Pointer1++;//34
	UUint32Pointer1++;//35
	UUint32Pointer1++;//36

	ABS_COORDINATE_Clone.Axis1_Clone_Linear_Z = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis2_Clone_Linear_A = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis5 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis9_Plate_Transfer_Z = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis10_Plate_Transfer_A = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate

	ABS_COORDINATE_Clone.Axis11_PlateWaste_Source_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis12_PlateWaste_Source_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis13 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis14 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis15_Clone_Linear_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis16_Clone_Linear_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis17_Clone_Optical_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis18_Clone_Optical_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate

	ABS_COORDINATE_Clone.Axis21_Plate_Transfer_X = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis22_Plate_Transfer_Y = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis23 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis24 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate
	ABS_COORDINATE_Clone.Axis25 = (*UUint32Pointer1++) / 1000.0f;//axis ABS coordinate

	//各轴机床坐标
	MACH_COORDINATE_Clone.Axis1_Clone_Linear_Z = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis2_Clone_Linear_A = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis5 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis9_Plate_Transfer_Z = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis10_Plate_Transfer_A = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate

	MACH_COORDINATE_Clone.Axis11_PlateWaste_Source_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis12_PlateWaste_Source_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis13 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis14 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis15_Clone_Linear_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis16_Clone_Linear_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis17_Clone_Optical_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis18_Clone_Optical_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate

	MACH_COORDINATE_Clone.Axis21_Plate_Transfer_X = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis22_Plate_Transfer_Y = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis23 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis24 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate
	MACH_COORDINATE_Clone.Axis25 = (*UUint32Pointer1++) / 1000.0f;//axis machine coordinate

// 	MACH_COORDINATE_PlateTransfering.Z = MACH_COORDINATE_Clone.Axis9_Plate_Transfer_Z;
// 	MACH_COORDINATE_PlateTransfering.A = MACH_COORDINATE_Clone.Axis10_Plate_Transfer_A;
// 	MACH_COORDINATE_PlateTransfering.X = MACH_COORDINATE_Clone.Axis21_Plate_Transfer_X;
// 	MACH_COORDINATE_PlateTransfering.Y = MACH_COORDINATE_Clone.Axis22_Plate_Transfer_Y;
// 	MACH_COORDINATE_PlateTransfering.B = 0;
// 	MACH_COORDINATE_PlateTransfering.C = 0;
// 
// 	MACH_COORDINATE_PlateWaste_Source.Z = 0;
// 	MACH_COORDINATE_PlateWaste_Source.A = 0;
// 	MACH_COORDINATE_PlateWaste_Source.X = MACH_COORDINATE_Clone.Axis11_PlateWaste_Source_X;
// 	MACH_COORDINATE_PlateWaste_Source.Y = MACH_COORDINATE_Clone.Axis12_PlateWaste_Source_Y;
// 	MACH_COORDINATE_PlateWaste_Source.B = 0;
// 	MACH_COORDINATE_PlateWaste_Source.C = 0;
// 
// 	MACH_COORDINATE_PlateEmpty_Destination.Z = MACH_COORDINATE_Clone.Axis20_PlateEmpty_Destination_Z2_L;
// 	MACH_COORDINATE_PlateEmpty_Destination.A = MACH_COORDINATE_Clone.Axis6_PlateEmpty_Destination_Z1_R;
// 	MACH_COORDINATE_PlateEmpty_Destination.X = MACH_COORDINATE_Clone.Axis8_PlateEmpty_Destination_X;
// 	MACH_COORDINATE_PlateEmpty_Destination.Y = MACH_COORDINATE_Clone.Axis7_PlateEmpty_Destination_Y;
// 	MACH_COORDINATE_PlateEmpty_Destination.B = 0;
// 	MACH_COORDINATE_PlateEmpty_Destination.C = 0;
// 
// 	MACH_COORDINATE_Clone_Linear.Z = MACH_COORDINATE_Clone.Axis1_Clone_Linear_Z;
// 	MACH_COORDINATE_Clone_Linear.A = MACH_COORDINATE_Clone.Axis2_Clone_Linear_A;
// 	MACH_COORDINATE_Clone_Linear.X = MACH_COORDINATE_Clone.Axis15_Clone_Linear_X;
// 	MACH_COORDINATE_Clone_Linear.Y = MACH_COORDINATE_Clone.Axis16_Clone_Linear_Y;
// 	MACH_COORDINATE_Clone_Linear.B = 0;
// 	MACH_COORDINATE_Clone_Linear.C = 0;
// 
// 	MACH_COORDINATE_Clone_Optical.Z = MACH_COORDINATE_Clone.Axis19_Clone_Optical_LensUpDown;
// 	MACH_COORDINATE_Clone_Optical.A = MACH_COORDINATE_Clone.Axis3_Clone_Optical_LensTrans;
// 	MACH_COORDINATE_Clone_Optical.B = MACH_COORDINATE_Clone.Axis4_Clone_Optical_FilterTrans;
// 	MACH_COORDINATE_Clone_Optical.X = MACH_COORDINATE_Clone.Axis17_Clone_Optical_X;
// 	MACH_COORDINATE_Clone_Optical.Y = MACH_COORDINATE_Clone.Axis18_Clone_Optical_Y;
// 	MACH_COORDINATE_Clone_Optical.C = 0;
	//added 20150408 for test COM. time
	SYSTEMTIME ComTime;
	GetLocalTime(&ComTime);
	DWORD mCurrentTick = ComTime.wMilliseconds;
	DWORD mDelta = mCurrentTick - m_lastReadTime;

	//TRACE("NowValue:%f , Elapsed time : %d milliseconds    = %d - %d\r",ABS_COORDINATE_Clone.Axis1,mDelta,mCurrentTick , m_lastReadTime);
	m_lastReadTime = ComTime.wMilliseconds;

	if ((*UUint32Pointer1) == 0x01)
	{
		OverallSign_Clone.CanSendCodeSign_B4 = true;
	}
	else
	{
		OverallSign_Clone.CanSendCodeSign_B4 = false;
	}
	UUint32Pointer1++;

	if ((*UUint32Pointer1) == 0x01)
	{
		OverallSign_Clone.CanSendCodeSign_B3 = true;
	}
	else
	{
		OverallSign_Clone.CanSendCodeSign_B3 = false;
	}
	UUint32Pointer1++;


	str_Information_Clone.Empty();
	if ((*UUint32Pointer1) == 0x01)
	{
		if (Error_main_Clone.MainErrorSign == 0)
			//Send_NoticeWarningInfo_To_Queue("DSP main error\n", -1, 0, 1);
		Error_main_Clone.MainErrorSign = 1;
		str_Information_Clone = str_Information_Clone + _T("DSP main error");

	}
	else
	{
		Error_main_Clone.MainErrorSign = 0;
	}
	UUint32Pointer1++;

	if ((*UUint32Pointer1) == 0x01)
	{	//获取稳定坐标 
		OverallSign_Clone.GetPositionSign = true;
		//GetDlgItem(IDC_Clone_GetPosition)->SetWindowText(_T("1"));
	}
	else
	{	//未获取稳定坐标
		OverallSign_Clone.GetPositionSign = false;
		//GetDlgItem(IDC_Clone_GetPosition)->SetWindowText(_T("0"));
	}
	UUint32Pointer1++;

	if ((*UUint32Pointer1) == 0x01)
	{	//最后一条代码走完标志  1－走完   
		OverallSign_Clone.LastCodeOverSign = true;
		//DualCore_SetLastCodeOverSign(OverallSign_Clone.LastCodeOverSign);
		//GetDlgItem(IDC_Clone_CodeOver)->SetWindowText(_T("1"));
	}
	else
	{	//最后一条代码走完标志  1－走完   
		OverallSign_Clone.LastCodeOverSign = false;
		//DualCore_SetLastCodeOverSign(OverallSign_Clone.LastCodeOverSign);
		//GetDlgItem(IDC_Clone_CodeOver)->SetWindowText(_T("0"));
	}
	UUint32Pointer1++;

	temp = *UUint32Pointer1++;
	if (temp == 0x01)
	{
		OverallSign_Clone.ParameterWriteCompleteSign = 1;
		if (OverallSign_Clone.NeedParameterWriteSign == 1)
		{
			OverallSign_Clone.NeedParameterWriteSign = 0;
		}
	}
	else
	{
		OverallSign_Clone.ParameterWriteCompleteSign = 0;
	}

	//use 32bit to receive positive hard limit,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	FPGA_HardLimitPOS_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	if (FPGA_HardLimitPOS_Register_Clone.all == 0)
	{
		Error_main_Clone.HardlimitPosFlag = 0;
	}
	else
	{
		if (Error_main_Clone.HardlimitPosFlag == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("正硬限位\n", -1, 0, 1);
			printf("HardlimitPos=0x%x\n", temp);
		}
		Error_main_Clone.HardlimitPosFlag = 1;
	}

	//GetDlgItem(IDC_Clone_Limit_HPOS)->SetWindowText(str);

	//use 32bit to receive negative hard limit,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	FPGA_HardLimitNEG_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	if (FPGA_HardLimitNEG_Register_Clone.all == 0)
	{
		Error_main_Clone.HardlimitNegFlag = 0;
	}
	else
	{
		if (Error_main_Clone.HardlimitNegFlag == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("负硬限位\n", -1, 0, 1);
			printf("HardlimitNEG=0x%x\n", temp);
		}
		Error_main_Clone.HardlimitNegFlag = 1;
	}
	//GetDlgItem(IDC_Clone_Limit_HNEG)->SetWindowText(str);

	//use 32bit to receive position complete signal,Max. aixs=32 axis
	temp = 0xF87FFFFF & (*UUint32Pointer1++);
	FPGA_Coin_Register_Clone.all = temp;
	OverallSign_Clone.EveryAxisCoinValid = (temp & 0x80000000) ? (1) : (0);
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_COIN)->SetWindowText(str);	

	//use 32bit to receive Home complete signal,Max. aixs=32 axis
	temp = *UUint32Pointer1++;
	FPGA_Home_Register_Clone.all = temp;
	//Homing_Clone.Findzero = (temp & 0x80000000)?(1):(0);
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_FindRef)->SetWindowText(str);	
	if ((temp & 0x80000000) == 0x80000000)
	{
		if (Homing_Clone.Findzero == 0)
		{
// 			Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 0, 0, 1);
// 			Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 1, 0, 1);
// 			Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 2, 0, 1);
// 			Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 3, 0, 1);
// 			Send_NoticeWarningInfo_To_Queue("已找到参考点\n", 4, 0, 1);
			printf("已找到参考点\n");
			str_Information_Clone = str_Information_Clone + _T("已找到参考点 ");
		}
		Homing_Clone.Findzero = 1;
	}
	else
	{
		Homing_Clone.Findzero = 0;
		//str_Information_Clone=str_Information_Clone+_T("无参考点 ");
	}

	//use 32bit to receive other alarm,Max. =32
	temp = *UUint32Pointer1++;
	temp1 = temp & 0x01;
	if (temp1 == 0x01)
	{
		if (Error_main_Clone.InsideRAMCheckError == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("DSP芯片内部RAM检验出错\n", -1, 0, 1);
			str_Information_Clone = str_Information_Clone + _T("DSP芯片内部RAM检验出错 ");
		}
		Error_main_Clone.InsideRAMCheckError = 1;		//BIT0  DSP芯片内部RAM检验出错报警  		1: 报警有效
	}
	else
	{
		Error_main_Clone.InsideRAMCheckError = 0;
	}

	temp1 = temp & 0x02;
	if (temp1 == 0x02)
	{
		if (Error_main_Clone.OutsideRAMCheckError == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("DSP芯片外部RAM检验出错\n", -1, 0, 1);
			str_Information_Clone = str_Information_Clone + _T("DSP芯片外部RAM检验出错 ");
		}
		Error_main_Clone.OutsideRAMCheckError = 1;	//BIT1  DSP芯片外部RAM检验出错报警  		1: 报警有效

	}
	else
	{
		Error_main_Clone.OutsideRAMCheckError = 0;
	}
	temp1 = temp & 0x0004;
	if (temp1 == 0x0004)
	{
		if (Error_main_Clone.NCCodeBreak == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("发送代码不连续\n", -1, 0, 1);
			str_Information_Clone = str_Information_Clone + _T("发送代码不连续 ");
		}
		Error_main_Clone.NCCodeBreak = 1;			//BIT2  PC向DSP发送代码不连续报警1: 报警有效

	}
	else
	{
		Error_main_Clone.NCCodeBreak = 0;
	}

	temp1 = temp & 0x0008;
	if (temp1 == 0x0008)
	{
		if (Error_main_Clone.TrackRunOutError == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("轨迹超程报警\n", -1, 0, 1);
			str_Information_Clone = str_Information_Clone + _T("轨迹超程报警 ");
		}
		Error_main_Clone.TrackRunOutError = 1;				//BIT3  DSP轨迹超程报警  			1: 报警有效

	}
	else
	{
		Error_main_Clone.TrackRunOutError = 0;
	}
	temp1 = temp & 0x0010;
	if (temp1 == 0x0010)
	{
		if (Error_main_Clone.InterpolationOver == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("插补量过大报警\n", -1, 0, 1);
			str_Information_Clone = str_Information_Clone + _T("插补量过大报警 ");
		}
		Error_main_Clone.InterpolationOver = 1; 		//BIT4  DSP插补量过大报警  		1: 报警有效

	}
	else
	{
		Error_main_Clone.InterpolationOver = 0;
	}
	//use 32bit to receive alarm signal,Max. aixs=32 axis
	//temp=0x7FFFFF-*UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	FPGA_ServoAlarm_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_ServoAlarm)->SetWindowText(str);	
	if (temp & 0x1FFFFFF)
	{
		if (Error_main_Clone.ServoAlarmFlag == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("轴伺服报警\n", -1, 0, 1);
			str_Information_Clone = str_Information_Clone + _T("轴伺服报警 ");
			printf("ServoAlarm=0x%x\n", temp);
		}
		Error_main_Clone.ServoAlarmFlag = 1;
	}
	else
	{
		Error_main_Clone.ServoAlarmFlag = 0;
	}

	temp = *UUint32Pointer1++;
	FPGA_EncoderAlarm_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_EncoderAlarm)->SetWindowText(str);	

	if (temp & 0x1FFFFFF)
	{
		if (Error_main_Clone.EncoderErrorFlag == 0)
		{
			//Send_NoticeWarningInfo_To_Queue("轴编码器报警\n", -1, 0, 1);
			str_Information_Clone = str_Information_Clone + _T("轴编码器报警 ");
			printf("EncoderError=0x%x\n", temp);
		}
		Error_main_Clone.EncoderErrorFlag = 1;
	}
	else
	{
		Error_main_Clone.EncoderErrorFlag = 0;
	}

	//GPIN1
	temp = 0xFFFFFFFF - (*UUint32Pointer1++);
	//FPGA_In_Register_Clone.all = temp;
	Input_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_GPIN)->SetWindowText(str);		
	////m_Control_MainInterface.GetDlgItem(IDC_Clone_GPIOIN1)->SetWindowText(str);
	//GPIN2
	temp = 0xFFFFFFFF - (*UUint32Pointer1++);
	//FPGA_In_Register_Clone2.all = temp;
	Input_Register_Clone2.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_GPIN2)->SetWindowText(str);	
	////m_Control_MainInterface.GetDlgItem(IDC_Clone_GPIOIN2)->SetWindowText(str);

	//GPOU1
	temp = 0xFFFFFFFF - *UUint32Pointer1++;
	//Output_Register_Clone.all = temp;	
	//str.Format(_T("0x%x"),temp);
	//GetDlgItem(IDC_Clone_GPOUT)->SetWindowText(str);	
	//GPOU2
	temp = *UUint32Pointer1++;
	//Output_Register_Clone2.all = temp;	
	//str.Format(_T("0x%x"),temp);
	//GetDlgItem(IDC_Clone_GPOUT2)->SetWindowText(str);

	//MainCmd1
	temp = *UUint32Pointer1++;
	MainCommand_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainCMD)->SetWindowText(str);		
	//MainCmd2
	temp = *UUint32Pointer1++;
	MainCommand_Register_Clone2.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainCMD2)->SetWindowText(str);

	//MainStatus1
	temp = *UUint32Pointer1++;
	MainStatus_Register_Clone.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainStatus)->SetWindowText(str);
	//MainStatus2
	temp = *UUint32Pointer1++;
	MainStatus_Register_Clone2.all = temp;
	str.Format(_T("0x%x"), temp);
	//GetDlgItem(IDC_Clone_MainStatus2)->SetWindowText(str);


	temp = *UUint32Pointer1++;//reserved
	FPGA_SoftLimitPOS_Register_Clone.all = temp;
	if (temp > 0)
	{
		if (Error_main_Clone.SoftlimitPosFlag == 0)
		{
			printf("SoftlimitPos=0x%x\n", temp);
			//Send_NoticeWarningInfo_To_Queue("正软限位\n", -1, 0, 1);
		}
		Error_main_Clone.SoftlimitPosFlag = 1;
	}
	else
	{
		Error_main_Clone.SoftlimitPosFlag = 0;
	}
	temp = *UUint32Pointer1++;//reserved
	FPGA_SoftLimitNEG_Register_Clone.all = temp;
	if (temp > 0)
	{
		if (Error_main_Clone.SoftlimitNegFlag == 0)
		{
			printf("SoftlimitNeg=0x%x\n", temp);
			//Send_NoticeWarningInfo_To_Queue("负软限位\n", -1, 0, 1);
		}
		Error_main_Clone.SoftlimitNegFlag = 1;
	}
	else
	{
		Error_main_Clone.SoftlimitNegFlag = 0;
	}

//	DualCore_Clone_CheckLimit();

	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;

	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;
	temp = *UUint32Pointer1++;

	RecvUDPSequence_B5 = *UUint32Pointer1++;
	RecvUDPSequence_B4 = *UUint32Pointer1++;
	RecvUDPSequence_B3 = *UUint32Pointer1++;
	RecvUDPSequence_B2 = *UUint32Pointer1++;
	RecvUDPSequence_B1 = *UUint32Pointer1++;
	RecvUDPSequence_B0 = *UUint32Pointer1++;

	if (UDP_DualCore_WaitRespFlag == 1)
	{
		if (RecvUDPSequence_B0 == (SendUDPSequence_B0 - 1))
		{
			UDP_DualCore_WaitRespFlag = 0;
			SetEvent(event_DualCore_UDP_Rsend);
		}
	}
	else if (UDP_DualCore_WaitRespFlag == 2)
	{
		if (RecvUDPSequence_B1 == (SendUDPSequence_B1 - 1))
		{
			UDP_DualCore_WaitRespFlag = 0;
			SetEvent(event_DualCore_UDP_Rsend);
		}
	}
	else if (UDP_DualCore_WaitRespFlag == 3)
	{
		if (RecvUDPSequence_B2 == (SendUDPSequence_B2 - 1))
		{
			UDP_DualCore_WaitRespFlag = 0;
			SetEvent(event_DualCore_UDP_Rsend);
		}
	}
	else if (UDP_DualCore_WaitRespFlag == 4)
	{
		if (RecvUDPSequence_B3 == (SendUDPSequence_B3 - 1))
		{
			UDP_DualCore_WaitRespFlag = 0;
			SetEvent(event_DualCore_UDP_Rsend);
		}
	}
	else if (UDP_DualCore_WaitRespFlag == 5)
	{
		if (RecvUDPSequence_B4 == (SendUDPSequence_B4 - 1))
		{
			UDP_DualCore_WaitRespFlag = 0;
			SetEvent(event_DualCore_UDP_Rsend);
		}
	}
	else if (UDP_DualCore_WaitRespFlag == 6)
	{
		if (RecvUDPSequence_B5 == (SendUDPSequence_B5 - 1))
		{
			UDP_DualCore_WaitRespFlag = 0;
			SetEvent(event_DualCore_UDP_Rsend);
		}
	}
	return 1;
}

void Udp_DualCore_init(void)
{
	DIAControl_DualCore_Clone_init();
	UDP_DualCore_WaitRespFlag = 0;
	event_DualCore_UDP_Rsend = CreateEvent(NULL, TRUE, FALSE, NULL);
	ResetEvent(event_DualCore_UDP_Rsend);
	InitializeCriticalSectionAndSpinCount(&dualcore_udp_lock, 4000);
}