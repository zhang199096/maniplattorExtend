//file***************************************************************************************************
//                                                                                                       
//                                                                                                       
//        DIAserial.h for  Master                  
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
//*******************************************************************************************************

#pragma once

#define IP_Dual_Core 20
#define IP_Clone     186
#define RESERVE       0
#define RUN           1
#define REFERENCE     2
#define JOG           3
#define SIMULATION    4
#define IDLE          5
#define STOP          6
#define OTHER         7
#define SERVOCONTROL  8
#define MOTION_Machine 9
#define MOTION_ABS	  10
#define DSPPRAMETER_WRITE         11
#define PCPRAMETER_WRITE         12
#define INCUBATOR_POSITION		13
#define PLATEMPTY_DESTINATION_POSITION		13
#define PLATEWASTE_SOURCE_POSITION		13
#define CLONE_LINEAR_POSITION		13

//#define 	STDGCODE_MOD			10	//Code B3 pactet Max buffer cnt.
#define     DUALCORE_STDGCODE_MOD   10000
#define 	SPCGCODE_MOD			11		//Code B4 pactet Max buffer cnt.
#define	DSPGCODEPACKET_MOD	40		//Code for Test pactet Max buffer cnt.

#define NOCODERUN		0
#define STDCODERUN		1
#define SPCCODERUN		2

#define  	RealTimeRun				0				// ʵʱ����
#define  	SimulateRun				1				// ��������

#define		OPEN					1
#define		CLOSE					0

#define		ThreeAxis			0				//����
#define		FourAxis			1				//����
#define		FiveAxis			2				//����

#define		TRUE					1
#define		OK						1
#define		COMPLETE				1	
#define		FALSE					0
#define		NOT_COMPLETE				0

#define  	REF_XYZ			0	//����ϵΪ3 ����
#define  	REF_XYZA		1	//����ϵΪ3+A ����
#define  	REF_XYZB		2	//����ϵΪ3+B ����
#define  	REF_XYZC		3	//����ϵΪ3+C ����

#define	StepZero		0		//����0
#define	StepOne			1		//����1
#define	StepTwo			2		//����2
#define	StepThree		3		//����3

#define	ImageDataBuff_MOD		1000	

#define 	STDGCODE_MOD			40000
#define 	SPCGCODE_MOD			11
#define	DSPGCODEPACKET_MOD	40

#define PARABUF_LINEARSUB	300
#define Linear_DeltaValue 1000

#define CameraModle_Software 0
#define CameraModle_External 1


typedef unsigned char    Uint8;
typedef unsigned short   Uint16;
typedef unsigned int     Uint32;
typedef unsigned long    Uint40;
typedef char             Int8;
typedef short            Int16;
typedef int              Int32;
typedef long             Int40;
typedef float           float32;
typedef double	     	float64;


typedef struct
{
	Int32 Axis1_Clone_Linear_Z;
	Int32 Axis2_Clone_Linear_A;
	Int32 Axis3_Clone_Optical_LensTrans;
	Int32 Axis4_Clone_Optical_FilterTrans;
	Int32 Axis5;
	Int32 Axis6_PlateEmpty_Destination_Z1_R;	
	Int32 Axis7_PlateEmpty_Destination_Y;
	Int32 Axis8_PlateEmpty_Destination_X;
	Int32 Axis9_Plate_Transfer_Z;
	Int32 Axis10_Plate_Transfer_A;
	Int32 Axis11_PlateWaste_Source_X;
	Int32 Axis12_PlateWaste_Source_Y;
	Int32 Axis13;
	Int32 Axis14;
	Int32 Axis15_Clone_Linear_X;
	Int32 Axis16_Clone_Linear_Y;	
	Int32 Axis17_Clone_Optical_X;
	Int32 Axis18_Clone_Optical_Y;
	Int32 Axis19_Clone_Optical_LensUpDown;
	Int32 Axis20_PlateEmpty_Destination_Z2_L;
	Int32 Axis21_Plate_Transfer_X;
	Int32 Axis22_Plate_Transfer_Y;
	Int32 Axis23;
	Int32 Axis24;
	Int32 Axis25;
}INT40AXIS;     

typedef struct
{
	Uint32 Axis1_Clone_Linear_Z;
	Uint32 Axis2_Clone_Linear_A;
	Uint32 Axis3_Clone_Optical_LensTrans;
	Uint32 Axis4_Clone_Optical_FilterTrans;
	Uint32 Axis5;
	Uint32 Axis6_PlateEmpty_Destination_Z1_R;	
	Uint32 Axis7_PlateEmpty_Destination_Y;
	Uint32 Axis8_PlateEmpty_Destination_X;
	Uint32 Axis9_Plate_Transfer_Z;
	Uint32 Axis10_Plate_Transfer_A;
	Uint32 Axis11_PlateWaste_Source_X;
	Uint32 Axis12_PlateWaste_Source_Y;
	Uint32 Axis13;
	Uint32 Axis14;
	Uint32 Axis15_Clone_Linear_X;
	Uint32 Axis16_Clone_Linear_Y;	
	Uint32 Axis17_Clone_Optical_X;
	Uint32 Axis18_Clone_Optical_Y;
	Uint32 Axis19_Clone_Optical_LensUpDown;
	Uint32 Axis20_PlateEmpty_Destination_Z2_L;
	Uint32 Axis21_Plate_Transfer_X;
	Uint32 Axis22_Plate_Transfer_Y;
	Uint32 Axis23;
	Uint32 Axis24;
	Uint32 Axis25;
}UINT40AXIS;     

typedef struct
{
	Uint16 Axis1_Clone_Linear_Z;
	Uint16 Axis2_Clone_Linear_A;
	Uint16 Axis3_Clone_Optical_LensTrans;
	Uint16 Axis4_Clone_Optical_FilterTrans;
	Uint16 Axis5;
	Uint16 Axis6_PlateEmpty_Destination_Z1_R;	
	Uint16 Axis7_PlateEmpty_Destination_Y;
	Uint16 Axis8_PlateEmpty_Destination_X;
	Uint16 Axis9_Plate_Transfer_Z;
	Uint16 Axis10_Plate_Transfer_A;
	Uint16 Axis11_PlateWaste_Source_X;
	Uint16 Axis12_PlateWaste_Source_Y;
	Uint16 Axis13;
	Uint16 Axis14;
	Uint16 Axis15_Clone_Linear_X;
	Uint16 Axis16_Clone_Linear_Y;	
	Uint16 Axis17_Clone_Optical_X;
	Uint16 Axis18_Clone_Optical_Y;
	Uint16 Axis19_Clone_Optical_LensUpDown;
	Uint16 Axis20_PlateEmpty_Destination_Z2_L;
	Uint16 Axis21_Plate_Transfer_X;
	Uint16 Axis22_Plate_Transfer_Y;
	Uint16 Axis23;
	Uint16 Axis24;
	Uint16 Axis25;
}UINT16AXIS;     

typedef struct
{
	Int16 Axis1_Clone_Linear_Z;
	Int16 Axis2_Clone_Linear_A;
	Int16 Axis3_Clone_Optical_LensTrans;
	Int16 Axis4_Clone_Optical_FilterTrans;
	Int16 Axis5;
	Int16 Axis6_PlateEmpty_Destination_Z1_R;	
	Int16 Axis7_PlateEmpty_Destination_Y;
	Int16 Axis8_PlateEmpty_Destination_X;
	Int16 Axis9_Plate_Transfer_Z;
	Int16 Axis10_Plate_Transfer_A;
	Int16 Axis11_PlateWaste_Source_X;
	Int16 Axis12_PlateWaste_Source_Y;
	Int16 Axis13;
	Int16 Axis14;
	Int16 Axis15_Clone_Linear_X;
	Int16 Axis16_Clone_Linear_Y;	
	Int16 Axis17_Clone_Optical_X;
	Int16 Axis18_Clone_Optical_Y;
	Int16 Axis19_Clone_Optical_LensUpDown;
	Int16 Axis20_PlateEmpty_Destination_Z2_L;
	Int16 Axis21_Plate_Transfer_X;
	Int16 Axis22_Plate_Transfer_Y;
	Int16 Axis23;
	Int16 Axis24;
	Int16 Axis25;
}INT16AXIS;     

typedef struct
{
	Uint8 Axis1_Clone_Linear_Z;
	Uint8 Axis2_Clone_Linear_A;
	Uint8 Axis3_Clone_Optical_LensTrans;
	Uint8 Axis4_Clone_Optical_FilterTrans;
	Uint8 Axis5;
	Uint8 Axis6_PlateEmpty_Destination_Z1_R;	
	Uint8 Axis7_PlateEmpty_Destination_Y;
	Uint8 Axis8_PlateEmpty_Destination_X;
	Uint8 Axis9_Plate_Transfer_Z;
	Uint8 Axis10_Plate_Transfer_A;
	Uint8 Axis11_PlateWaste_Source_X;
	Uint8 Axis12_PlateWaste_Source_Y;
	Uint8 Axis13;
	Uint8 Axis14;
	Uint8 Axis15_Clone_Linear_X;
	Uint8 Axis16_Clone_Linear_Y;	
	Uint8 Axis17_Clone_Optical_X;
	Uint8 Axis18_Clone_Optical_Y;
	Uint8 Axis19_Clone_Optical_LensUpDown;
	Uint8 Axis20_PlateEmpty_Destination_Z2_L;
	Uint8 Axis21_Plate_Transfer_X;
	Uint8 Axis22_Plate_Transfer_Y;
	Uint8 Axis23;
	Uint8 Axis24;
	Uint8 Axis25;
}UINT8AXIS;   

typedef struct
{
	float64 Axis1_Clone_Linear_Z;
	float64 Axis2_Clone_Linear_A;
	float64 Axis3_Clone_Optical_LensTrans;
	float64 Axis4_Clone_Optical_FilterTrans;
	float64 Axis5;
	float64 Axis6_PlateEmpty_Destination_Z1_R;	
	float64 Axis7_PlateEmpty_Destination_Y;
	float64 Axis8_PlateEmpty_Destination_X;
	float64 Axis9_Plate_Transfer_Z;
	float64 Axis10_Plate_Transfer_A;
	float64 Axis11_PlateWaste_Source_X;
	float64 Axis12_PlateWaste_Source_Y;
	float64 Axis13;
	float64 Axis14;
	float64 Axis15_Clone_Linear_X;
	float64 Axis16_Clone_Linear_Y;	
	float64 Axis17_Clone_Optical_X;
	float64 Axis18_Clone_Optical_Y;
	float64 Axis19_Clone_Optical_LensUpDown;
	float64 Axis20_PlateEmpty_Destination_Z2_L;
	float64 Axis21_Plate_Transfer_X;
	float64 Axis22_Plate_Transfer_Y;
	float64 Axis23;
	float64 Axis24;
	float64 Axis25;
}float64AXIS;     

typedef struct
{
	Uint16		SearchRefSign;			// Ҫ�������ο����־
	Uint16		RefStep;				//�ο�����������
	UINT16AXIS	NeedFindAxisSign;		// ��Ҫ������ı�־
	UINT16AXIS	FindRefSign;			// ���ҵ�ĳ��ο����־
	Uint16		PreSearchRefSign;		// �����ο���ǰ��ʼ����־
	Uint16      Findzero;               //Ѱ�ҵ�����־added by mendy 2011/6/20
	Uint16 		getzero;				//���ڼ�¼�������added by mendy 2011/6/21
}HOMING;


typedef struct
{
	float Axis1_Clone_Linear_Z;
	float Axis2_Clone_Linear_A;
	float Axis3_Clone_Optical_LensTrans;
	float Axis4_Clone_Optical_FilterTrans;
	float Axis5;
	float Axis6_PlateEmpty_Destination_Z1_R;	
	float Axis7_PlateEmpty_Destination_Y;
	float Axis8_PlateEmpty_Destination_X;
	float Axis9_Plate_Transfer_Z;
	float Axis10_Plate_Transfer_A;
	float Axis11_PlateWaste_Source_X;
	float Axis12_PlateWaste_Source_Y;
	float Axis13;
	float Axis14;
	float Axis15_Clone_Linear_X;
	float Axis16_Clone_Linear_Y;	
	float Axis17_Clone_Optical_X;
	float Axis18_Clone_Optical_Y;
	float Axis19_Clone_Optical_LensUpDown;
	float Axis20_PlateEmpty_Destination_Z2_L;
	float Axis21_Plate_Transfer_X;
	float Axis22_Plate_Transfer_Y;
	float Axis23;
	float Axis24;
	float Axis25;
}DOUBLEJOINTS; 

typedef struct
{
	Int32 Axis1_Clone_Linear_Z;
	Int32 Axis2_Clone_Linear_A;
	Int32 Axis3_Clone_Optical_LensTrans;
	Int32 Axis4_Clone_Optical_FilterTrans;
	Int32 Axis5;
	Int32 Axis6_PlateEmpty_Destination_Z1_R;	
	Int32 Axis7_PlateEmpty_Destination_Y;
	Int32 Axis8_PlateEmpty_Destination_X;
	Int32 Axis9_Plate_Transfer_Z;
	Int32 Axis10_Plate_Transfer_A;
	Int32 Axis11_PlateWaste_Source_X;
	Int32 Axis12_PlateWaste_Source_Y;
	Int32 Axis13;
	Int32 Axis14;
	Int32 Axis15_Clone_Linear_X;
	Int32 Axis16_Clone_Linear_Y;	
	Int32 Axis17_Clone_Optical_X;
	Int32 Axis18_Clone_Optical_Y;
	Int32 Axis19_Clone_Optical_LensUpDown;
	Int32 Axis20_PlateEmpty_Destination_Z2_L;
	Int32 Axis21_Plate_Transfer_X;
	Int32 Axis22_Plate_Transfer_Y;
	Int32 Axis23;
	Int32 Axis24;
	Int32 Axis25;
}INTJOINTS; 

typedef struct
{	
	float64		Tsample;							
	float64		Tsample1;				
	Uint8		SlaveMAX;				
	float64		TrackRunOutRangeSQR;	
	UINT40AXIS	acc_SET;			
	UINT40AXIS	dec_SET;			
	float64 VeerDeltaV;				
	float64 NicetyVeerDeltaV;			
	float64 VeerDeltaT;				
	float64 NicetyVeerDeltaT;			
	Uint32 LinearAxisMinUnit;			
	Uint32 LinearAxisOutUnitEQU;		

	float64 G0Speed;						
	float64 G0Speed_2;					
	float64 G1Speed;						
	float64 G1Speed_2;					

	float64 SRefSpeed;					
	float64 SRefSpeedBack;				
	float64 SRefBack;						
	float64 SRefSpeed_2;					
	float64 SRefSpeedBack_2;				
	float64 SRefBack_2;					

	UINT16AXIS REFStopVariable;				
	UINT16AXIS	RefDir;					
	UINT16AXIS	EncoderCheckChoose;		

	INT40AXIS	MotorChangeDir;
	INT40AXIS	EncoderRDDir;
	INT40AXIS	AxisResolution;
	INT40AXIS	CoordORG;
	INT40AXIS 	SafeCoordinate;
	INT40AXIS 	BackCoordinate;
	INT40AXIS 	OffsetCoordinate;
	float64AXIS	MAXSpeed;
	float64AXIS	REFStopPosition;
	INT40AXIS	SLimitPos;
	INT40AXIS 	SLimitNeg;
			
	INT40AXIS PositionCoordinate1; 
	INT40AXIS PositionCoordinate2; 

	Uint32 FunctionSelect01;				
	Uint32 FunctionSelect02;				
	Uint32 FunctionSelect03;				
	Uint32 FunctionSelect04;				
	Uint32 FunctionSelect05;				
	Uint32 FunctionSelect06;				
	Uint32 FunctionSelect07;				
	Uint32 FunctionSelect08;				
	Uint32 FunctionSelect09;				
	Uint32 FunctionSelect10;				

	Uint32 LEDFlashTime1;
	Uint32 LEDFlashTime2;
}SYSTEM;	//ϵͳ�����ṹ��

typedef struct
{
	Uint32 		SendCount;			//DSP��ǰ����ԭ����λ��ָ�룬���ظ�ARM������ʾ��������;
	Uint32 		Main_CMD,Sub_CMD1,Sub_CMD2;			//������,ȷ��G��������
	INT40AXIS	StartPoint;			 //���. ��λ����������С���뵥λ
	INT40AXIS	EndPoint;			 //�յ�.  ��λ����������С���뵥λ
	float64AXIS	Feedrate;			//�����ٶ�,��λ:mm/s
	Int16 		AxisMotionStyle;	//0��A4�᲻���У�1��A4�����С�3:���μ��ߵ�����IV�ᵥ�����У�4:IV�ᵥ��SPC	
	Uint32	HoldTime;		//GO4 Hold Time
}GCODE;


typedef struct
{
	Uint16 				NCSign;  	// 0:ͨ��״̬  1:��׼���룻2:�������-JOG HANDLE REF��INI=0
	Uint16 				NCSign_bak;  	// 0:ͨ��״̬  1:��׼���룻2:�������-JOG HANDLE REF��INI=0

	//�ٶȿ�����ز���
	Uint16				EveryAxisCoinValid;			// ����COIN ����Ч��־
	Uint16				EveryAxisOverValid;			// ����OVER ����Ч��־
	Uint8				FeedrateOverride;			//������λ	[0-20]
	Uint8				RapidFeedrateOverride;			//������λ	[0-20]
	Uint16				XYZ_Over;               			// 1--��ɵ�ǰG����XYZ������                                   
	float64    				F;						//G�����趨�����ٶ�  (mm/s)

	// ���������ز���
	INT40AXIS			ABS_Coordinate;				// ��ǰ��������ֵunit:��С���뵥λ
	INT40AXIS			ABSORG_M_Coordinate;		// ��ǰ��������ԭ��Ļ�������ֵunit:��С���뵥λ
	Uint16				LastCodeOverSign;			//���һ�����������־  1������
	UINT16AXIS			SearchRefMoveSign;			//�ο�����ҷ���1������2������

	//��ر�־λ	
	Uint8				Initial_STDSign;				//��־�������г�ʼ����־
	Uint8				Initial_SPCSign;				//�ֶ����㶯���г�ʼ����־
	Uint16 				GetPositionSign;				//1--�ѻ���ȶ���������ֵ��־
	Uint16 				NeedGetPositionSign;				//1--�����ȶ���������ֵ��־

	Uint8 				NeedParameterWriteSign;		//����д���־
	Uint8 				ParameterWriteCompleteSign;//����д����ɱ�־

	Uint8 				NeedMicroEDataBaseSendSign;		
	Uint8 				MicroEDataBaseSendCompleteSign;

	//�˶����״̬
	Uint8				ServoOn;
	int 					m_OperationMode;
	int 					m_OperationMode_bak;
	Uint8 				JOGAxisSelect;

	//ͨ�����״̬
	bool 				PC_DSP_COM_OK;//false:ֹͣ; true:����
	bool 				m_ReciveSuccess;
	int 					m_ReciveSuccessCounter;
	int 					m_ReciveLostCounter;
	int					sendcount;

	Uint8				CanSendCodeSign_B3;	//��λ�˿��Է��ʹ����־
	Uint8				CanSendCodeSign_B4;
	Uint32 				NeedSendCount;		//ARM��Ҫ���ʹ�������
	Uint32				STDSendCount;		//ARM/PC���ͱ�־����λ��ָ��
	Uint32				SPCSendCount;		//ARM/PC�����ֶ����㶯����λ��ָ��
	Uint32				LastSendCount;		//�ݴ�ARMʵ�ʷ��ʹ���λ��ָ��
	//��λ����������
	Uint32				STDReceiveDSPRunCount;//
	Uint32				SPCReceiveDSPRunCount;
	Uint32				STDReceiveDSPSendCount;
	Uint32				SPCReceiveDSPSendCount;	
	
	Uint8				CameraScanSign;
	Uint8				CameraStaticCaptureSign;
	Uint8				MicroEScanSign;
	Uint8				CameraSoftTriggerLedFlashSign;

	Uint32				MicroEDataBaseStartCnt;
	Uint32				MicroEDataBaseEndCnt;
	Uint32				MicroEDataBaseTotalCnt;
	Uint32				MicroEDataBaseStartCnt_DSP;

	Uint8 				PacketSendLinearSign;
	Uint32             packsendflag;
	Uint32				packsendfinishflag;
	Uint32			NeedP2PRunSign;
}OVERALLSIGN;
struct  DUALCORE_MAINSTATUSSIGN_BITS
{     // bits   description
	Uint32 SIGN1 : 1;
	Uint32 SIGN2 : 1;
	Uint32 SIGN3 : 1;
	Uint32 SIGN4 : 1;
	Uint32 SIGN5 : 1;
	Uint32 SIGN6 : 1;
	Uint32 SIGN7 : 1;
	Uint32 SIGN8 : 1;
	Uint32 SIGN9 : 1;
	Uint32 SIGN10 : 1;
	Uint32 SIGN11 : 1;
	Uint32 SIGN12 : 1;
	Uint32 SIGN13 : 1;
	Uint32 SIGN14 : 1;
	Uint32 SIGN15 : 1;
	Uint32 SIGN16 : 1;
	Uint32 SIGN17 : 1;
	Uint32 SIGN18 : 1;
	Uint32 SIGN19 : 1;
	Uint32 SIGN20 : 1;
	Uint32 SIGN21 : 1;
	Uint32 SIGN22 : 1;
	Uint32 SIGN23 : 1;
	Uint32 SIGN24 : 1;
	Uint32 SIGN25 : 1;
	Uint32 SIGN26 : 1;
	Uint32 SIGN27 : 1;
	Uint32 SIGN28 : 1;
	Uint32 SIGN29 : 1;
	Uint32 SIGN30 : 1;
	Uint32 SIGN31 : 1;
	Uint32 SIGN32 : 1;
};
union DUALCORE_MAINSTATUSSIGN_REG {
	Uint32         all;
	struct DUALCORE_MAINSTATUSSIGN_BITS bit;
};//���λִ����ɷ��ر�־
 typedef struct
{ 
	Uint32 VotageChannel;
	Uint32 VotageChannel_Sync;
	Uint32 VotageValue;
	Uint32 WaveSelect; //0:triangle;1:sin;2:cos;3:atan

	Uint32 AutoSign; 	// 1
	Uint32 Voltage_Max; //uint:0.01V
	Uint32 Voltage_Start; //uint:0.01V
	Uint32 Voltage_Max2; //uint:0.01V
	Uint32 Voltage_Start2; //uint:0.01V
	Uint32 CycleNum; //num

	Uint32 RiseTime; //uint:0.1ms
	Uint32 FallTime;  //uint:0.1ms
	Uint32 HighKeepTime; //uint:0.1ms
	Uint32 LowKeepTime; //uint:0.1ms

	Uint32 StartOffsetTime;//uint:0.1ms
	Uint32 EndOffsetTime;//uint:0.1ms

	Uint32 PWMHighCount; //uint:0.1us
	Uint32 PWMLowCount; //uint:0.1us
	Uint32 PWMPhase; //uint:0.1us
}DACONVERT; 
typedef struct
{
UINT16AXIS		EncoderError;		// Öá¹âÕ¤³ß±¨¾¯   1--±¨¾¯

	Uint16 		NCCodeBreak;					//ARM·¢ËÍG´úÂë²»Á¬Ðø
	Uint16 		InterpolationOver;			//²å²¹Á¿³¬5mm±¨¾¯
	Uint16 		TrackRunOutError;			//¹ì¼£Æ«Àë±¨¾¯
	Uint16 		HaveGetErrorDataSign;		//ÒÑ½ØÈ¡´íÎóÊý¾Ý±êÖ¾, when ARM ->RET
 	Uint16 		RealRunCount;				//ÒÑ½ØÈ¡´íÎóÊý¾ÝÔËÐÐÌõÊý	
	Uint16 		InsideRAMCheckError;		// ÄÚ²¿RAM Ð£Ñé´íÎó
	Uint16 		OutsideRAMCheckError;		// Íâ²¿RAM Ð£Ñé´íÎó
	Uint16 		MainErrorSign;				//×Ü´íÎó±êÖ¾
	Uint16		AlmHLimitPos;
	Uint16		AlmHLimitNeg;
	Uint16		AlmSLimitPos;
	Uint16		AlmSLimitNeg;
	UINT16AXIS 	ServoAlarm;							//ËÅ·þ±¨¾¯		1:±¨¾¯ÓÐÐ§
	Uint16 		CompDataCheckError;				// �����Բ�������У�����
	Uint16		OpticalFlashTriggerBreakError; //OpticalFlashTriggerBreak
	Uint16		CameraShutterTriggerBreakError; //CameraShutterTriggerBreak
	Uint16		LensPositionBreakError;
	Uint16 Resetlimit;
	Uint16 HardlimitPosFlag;
	Uint16 HardlimitNegFlag;
	Uint16 ServoAlarmFlag;
	Uint16 EncoderErrorFlag;
	Uint16 SoftlimitPosFlag;
	Uint16 SoftlimitNegFlag;
}ERROR_Main;					//�����ṹ��

struct  MAINCOMMANDSIGN_BITS 
{     // bits   description
	Uint32 SIGN1:1;	
	Uint32 SIGN2:1;	
	Uint32 SIGN3:1;	
	Uint32 SIGN4:1;	
	Uint32 SIGN5:1;	
	Uint32 SIGN6:1;	
	Uint32 SIGN7:1;	
	Uint32 SIGN8:1;
	Uint32 SIGN9:1;
	Uint32 SIGN10:1;
	Uint32 SIGN11:1;	
	Uint32 SIGN12:1;	
	Uint32 SIGN13:1;	
	Uint32 SIGN14:1;	
	Uint32 SIGN15:1;	
	Uint32 SIGN16:1;
	Uint32 SIGN17:1;	
	Uint32 SIGN18:1;	
	Uint32 SIGN19:1;	
	Uint32 SIGN20:1;	
	Uint32 SIGN21:1;	
	Uint32 SIGN22:1;	
	Uint32 SIGN23:1;	
	Uint32 SIGN24:1;
	Uint32 SIGN25:1;
	Uint32 SIGN26:1;
	Uint32 SIGN27:1;	
	Uint32 SIGN28:1;	
	Uint32 SIGN29:1;	
	Uint32 SIGN30:1;	
	Uint32 SIGN31:1;	
	Uint32 SIGN32:1;
};
union MAINCOMMANDSIGN_REG {
   Uint32         all;
   struct MAINCOMMANDSIGN_BITS bit;
};

struct  MAINSTATUSSIGN_BITS 
{     // bits   description
	Uint32 SIGN1:1;	
	Uint32 SIGN2:1;	
	Uint32 SIGN3:1;	
	Uint32 SIGN4:1;	
	Uint32 SIGN5:1;	
	Uint32 SIGN6:1;	
	Uint32 SIGN7:1;	
	Uint32 SIGN8:1;
	Uint32 SIGN9:1;
	Uint32 SIGN10:1;
	Uint32 SIGN11:1;	
	Uint32 SIGN12:1;	
	Uint32 SIGN13:1;	
	Uint32 SIGN14:1;	
	Uint32 SIGN15:1;	
	Uint32 SIGN16:1;
	Uint32 SIGN17:1;	
	Uint32 SIGN18:1;	
	Uint32 SIGN19:1;	
	Uint32 SIGN20:1;	
	Uint32 SIGN21:1;	
	Uint32 SIGN22:1;	
	Uint32 SIGN23:1;	
	Uint32 SIGN24:1;
	Uint32 SIGN25:1;
	Uint32 SIGN26:1;
	Uint32 SIGN27:1;	
	Uint32 SIGN28:1;	
	Uint32 SIGN29:1;	
	Uint32 SIGN30:1;	
	Uint32 SIGN31:1;	
	Uint32 SIGN32:1;
};
union MAINSTATUSSIGN_REG {
   Uint32         all;
   struct MAINSTATUSSIGN_BITS bit;
};

typedef struct
{
	Uint16 AxisID;
	UINT16AXIS AxisCount;

	INT40AXIS PARALinear_Buf[PARABUF_LINEARSUB];
	INT40AXIS PARALaser_Buf[PARABUF_LINEARSUB];

	
}LINEARCOMPENSATION;

extern HOMING					Homing_Clone;	
extern OVERALLSIGN				OverallSign_Clone;		// �����־
extern SYSTEM					System_Clone;			//ϵͳ�����ṹ����
extern ERROR_Main					Error_main_Clone;			//�����ṹ����
extern LINEARCOMPENSATION 		LinearCompensation;
extern DACONVERT					DA_Convert;

extern INT40AXIS MonitorBuffer1[2000]/*,MonitorBuffer2[1000]*/;
