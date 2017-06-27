//file***************************************************************************************************
//                                                                                                       
//                                                                                                       
//        DIAControl_Clone.h for  Master                  
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

#pragma once

void ini_sys_para_Clone(void);

	
// DIAControl_Clone ¶Ô»°¿ò




		//pointer for all
		//int *UUint32Pointer1,*UUint32Pointer2;
		//int *Uint32Pointer1,*Uint32Pointer2;
		//int ParaReadBuf[700];
		//int ParaWriteBuf[700];

		void ParameterSave_Write(void);
		void ParameterSave_Read(void);
		void ParaLinearCompensationBuf_ReadAndSend(UINT8AXIS AxisID);
		int ParaCheckLinearCompensation(Uint16 AxisCount,Int32 Linear_Buf[PARABUF_LINEARSUB],Int32 Laser_Buf[PARABUF_LINEARSUB]);
		
		int DSP_com_MainCMD_b0();		
		int DSP_com_MainCMD_b1();
		int DSP_com_MainCMD_b2();
		int DSP_com_MainCMD_b3();
		int DSP_com_MainCMD_b4();
		int DSP_com_MainCMD_b5();
		
		int DSP_com_MainCMD_50();
		int DSP_com_MainCMD_51(); 
		int DSP_com_MainCMD_52(); 
		int DSP_com_MainCMD_53(); 
		int DSP_com_MainCMD_54(); 
		int DSP_com_MainCMD_55(); 
		int DSP_com_MainCMD_56(); 	
		int DSP_com_BackCMD_b0();
		Uint16 DSPCodePacket_CycleRun(void);
		Uint16 CheckAllAxisInStaticState_Clone(void);
		void OnBnClickedReset();
		void DSP_PacketOut_Linear(void);	

		Uint32 DSPCodePacket_Code_G0(void);
		Uint32 DSPCodePacket_Code_G1(void);
		Uint16 DSPCodePacket_Code_G4(Uint32	HoldTime);
		Uint16 DSPCodePacket_Code_G100(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G101(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G102(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G103(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G104(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G105(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G106(DACONVERT  Wave_Para);
		Uint16 DSPCodePacket_Code_G107(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G108(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G109(Uint32 Sub_CMD1,Uint32 Sub_CMD2);
		Uint16 DSPCodePacket_Code_G110(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32 HoldTime);
		Uint16 DSPCodePacket_Code_G111(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32 HoldTime);
		Uint16 DSPCodePacket_Code_G112(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32 HoldTime);
		Uint16 DSPCodePacket_Code_G113(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint16 PWM_HighCount,Uint16 PWM_LowCount,Uint16 PWM_Phase);
		Uint16 DSPCodePacket_Code_G114(Uint32 Sub_CMD1,Uint32 Sub_CMD2,Uint32 HoldTime);

		Uint16 DSPCodePacket_Code_Empty(void);
		
		INT40AXIS DSPCodePacket_Code_SetEndPoint(void);

		Uint16 DSPCodePacket_Code_Feedrate(GCODE  *psGCodeSIPointer,float64 Feedrate);
		Uint16 DSPCodePacket_Code_StartPoint(GCODE  *psGCodeSIPointer);
		
		unsigned short crc16(unsigned char *data, unsigned short length);
		unsigned int crc32(unsigned int *data, unsigned short length);
		void send1();
		void JOGModle_DSP(void);

		void EnableWindowAllTrueOrFalse(Uint16 TrueOrFalse);
		void EnableWindowPositionGroupTrueOrFalse(Uint16 TrueOrFalse);
		Uint16 AllStatusCheck();

		void Udp_DualCore_init(void);
		void DIAControl_DualCore_Clone_init(void);
		int DualCore_DSP_com_BackCMD_b0(void);
		void ini_sys_para_Clone(void);
		void ParameterSave_Read();
		void DualCore_UDP_ResendDeal(void);
		void DualCore_Clone_Stop(void);
		void DIAControl_DualCore_Clone_Reset();
		//INT40AXIS MonitorBuffer1[10000],MonitorBuffer2[10000];

// 		extern DOUBLEJOINTS ABS_COORDINATE_Clone_Linear; //
		// 		extern DOUBLEJOINTS MACH_COORDINATE_Clone_Linear; //
		// 
		// 		extern DOUBLEJOINTS ABS_COORDINATE_Clone_Optical; //¾ø¶Ô×ø±ê
		// 		extern DOUBLEJOINTS MACH_COORDINATE_Clone_Optical; //±àÂëÆ÷×ø±ê
		// 
		// 		extern DOUBLEJOINTS ABS_COORDINATE_PlateEmpty_Destination; //¾ø¶Ô×ø±ê
		// 		extern DOUBLEJOINTS MACH_COORDINATE_PlateEmpty_Destination; //±àÂëÆ÷×ø±ê
		// 
		// 		extern DOUBLEJOINTS ABS_COORDINATE_PlateWaste_Source; //¾ø¶Ô×ø±ê
		// 		extern DOUBLEJOINTS MACH_COORDINATE_PlateWaste_Source; //±àÂëÆ÷×ø±ê
		// 
		// 		extern DOUBLEJOINTS ABS_COORDINATE_PlateTransfering; //¾ø¶Ô×ø±ê
		// 		extern DOUBLEJOINTS MACH_COORDINATE_PlateTransfering; //±àÂëÆ÷×ø±ê