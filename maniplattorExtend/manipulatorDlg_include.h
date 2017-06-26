//file***************************************************************************************************
//                                                                                                       
//                                                                                                       
//        manipulatorDlg_include.h for  Master                  
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

#ifndef __manipulatorDlg_include_h_
#define __manipulatorDlg_include_h_




//#include "afxwin.h"
#include "Common.h"
//#include "afxdialogex.h"


// #include "..//MotionControl//Clone//DIAControl_Clone.h"
// #include "..//MotionControl//UDP_Rerro.h"
// #include "..//MotionControl//UDP_Export.h"

#include "MotionControl/UDP_Export.h"
#include "MotionControl/UDP_Rerro.h"
#include "MotionControl/Clone/DIAControl_Clone.h"
#include <stdio.h>
#include "math.h"

//extern DIAControl_Clone m_Control_Clone;

extern HOMING			Homing_Clone;	
extern OVERALLSIGN		OverallSign_Clone;		// 总体标志
extern SYSTEM			System_Clone;			//系统参数结构对象
extern ERROR_Main			Error_main_Clone;			//报警结构对象
//extern LINEARCOMPENSATION LinearCompensation;


extern DOUBLEJOINTS ABS_COORDINATE_Clone; //缁濆鍧愭爣
extern DOUBLEJOINTS MACH_COORDINATE_Clone; //缂栫爜鍣ㄥ潗鏍?

extern union MAINSTATUSSIGN_REG FPGA_Coin_Register_Clone;
extern union MAINSTATUSSIGN_REG FPGA_Home_Register_Clone;
extern union DUALCORE_MAINSTATUSSIGN_REG FPGA_ServoAlarm_Register_Clone;
extern union DUALCORE_MAINSTATUSSIGN_REG FPGA_EncoderAlarm_Register_Clone;
extern union DUALCORE_MAINSTATUSSIGN_REG FPGA_HardLimitPOS_Register_Clone;
extern union DUALCORE_MAINSTATUSSIGN_REG FPGA_HardLimitNEG_Register_Clone;
extern union MAINSTATUSSIGN_REG SoftLimitPOS_Register_Clone;
extern union MAINSTATUSSIGN_REG SoftLimitNEG_Register_Clone;
extern union MAINSTATUSSIGN_REG TrackRunOver_Register_Clone;
extern union MAINSTATUSSIGN_REG InterpolationOver_Register_Clone;
extern union MAINSTATUSSIGN_REG CompensationDataCheck_Register_Clone;

extern union MAINCOMMANDSIGN_REG Input_Register_Clone;
extern union MAINCOMMANDSIGN_REG Input_Register_Clone2;
extern union MAINCOMMANDSIGN_REG Output_Register_Clone;
extern union MAINCOMMANDSIGN_REG Output_Register_Clone2;
extern union DUALCORE_MAINSTATUSSIGN_REG MainCommand_Register_Clone;
extern union DUALCORE_MAINSTATUSSIGN_REG MainCommand_Register_Clone2;
extern union DUALCORE_MAINSTATUSSIGN_REG MainStatus_Register_Clone;
extern union DUALCORE_MAINSTATUSSIGN_REG MainStatus_Register_Clone2;

extern Uint32	MonitorBuffer1Cnt/*,MonitorBuffer2Cnt,MonitorBuffer3Cnt,MonitorBuffer4Cnt*/;

extern CString str_Information_Clone;
#endif

