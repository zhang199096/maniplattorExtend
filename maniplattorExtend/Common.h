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

#define  	RealTimeRun				0				// 实时运行
#define  	SimulateRun				1				// 仿真运行

#define		OPEN					1
#define		CLOSE					0

#define		ThreeAxis			0				//三轴
#define		FourAxis			1				//四轴
#define		FiveAxis			2				//五轴

#define		TRUE					1
#define		OK						1
#define		COMPLETE				1	
#define		FALSE					0
#define		NOT_COMPLETE				0

#define  	REF_XYZ			0	//坐标系为3 坐标
#define  	REF_XYZA		1	//坐标系为3+A 坐标
#define  	REF_XYZB		2	//坐标系为3+B 坐标
#define  	REF_XYZC		3	//坐标系为3+C 坐标

#define	StepZero		0		//步骤0
#define	StepOne			1		//步骤1
#define	StepTwo			2		//步骤2
#define	StepThree		3		//步骤3

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
	Uint16		SearchRefSign;			// 要求搜索参考点标志
	Uint16		RefStep;				//参考点搜索步骤
	UINT16AXIS	NeedFindAxisSign;		// 需要搜索轴的标志
	UINT16AXIS	FindRefSign;			// 已找到某轴参考点标志
	Uint16		PreSearchRefSign;		// 搜索参考点前初始化标志
	Uint16      Findzero;               //寻找到零点标志added by mendy 2011/6/20
	Uint16 		getzero;				//用于记录零点坐标added by mendy 2011/6/21
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
}SYSTEM;	//系统参数结构组

typedef struct
{
	Uint32 		SendCount;			//DSP当前运行原代码位置指针，返回给ARM用于显示及其它用途
	Uint32 		Main_CMD,Sub_CMD1,Sub_CMD2;			//命令字,确定G代码类型
	INT40AXIS	StartPoint;			 //起点. 单位：线性轴最小输入单位
	INT40AXIS	EndPoint;			 //终点.  单位：线性轴最小输入单位
	float64AXIS	Feedrate;			//进给速度,单位:mm/s
	Int16 		AxisMotionStyle;	//0：A4轴不运行，1：A4轴运行。3:单段加线第三步IV轴单独运行；4:IV轴单独SPC	
	Uint32	HoldTime;		//GO4 Hold Time
}GCODE;


typedef struct
{
	Uint16 				NCSign;  	// 0:通常状态  1:标准代码；2:特殊代码-JOG HANDLE REF；INI=0
	Uint16 				NCSign_bak;  	// 0:通常状态  1:标准代码；2:特殊代码-JOG HANDLE REF；INI=0

	//速度控制相关参数
	Uint16				EveryAxisCoinValid;			// 各轴COIN 都有效标志
	Uint16				EveryAxisOverValid;			// 各轴OVER 都有效标志
	Uint8				FeedrateOverride;			//进给档位	[0-20]
	Uint8				RapidFeedrateOverride;			//进给档位	[0-20]
	Uint16				XYZ_Over;               			// 1--完成当前G代码XYZ轴运行                                   
	float64    				F;						//G代码设定进给速度  (mm/s)

	// 坐标跟踪相关参数
	INT40AXIS			ABS_Coordinate;				// 当前绝对坐标值unit:最小输入单位
	INT40AXIS			ABSORG_M_Coordinate;		// 当前绝对坐标原点的机床坐标值unit:最小输入单位
	Uint16				LastCodeOverSign;			//最后一条代码走完标志  1－走完
	UINT16AXIS			SearchRefMoveSign;			//参考点查找方向。1：负向；2：正向

	//相关标志位	
	Uint8				Initial_STDSign;				//标志代码运行初始化标志
	Uint8				Initial_SPCSign;				//手动、点动运行初始化标志
	Uint16 				GetPositionSign;				//1--已获得稳定反馈坐标值标志
	Uint16 				NeedGetPositionSign;				//1--需获得稳定反馈坐标值标志

	Uint8 				NeedParameterWriteSign;		//参数写入标志
	Uint8 				ParameterWriteCompleteSign;//参数写入完成标志

	Uint8 				NeedMicroEDataBaseSendSign;		
	Uint8 				MicroEDataBaseSendCompleteSign;

	//运动相关状态
	Uint8				ServoOn;
	int 					m_OperationMode;
	int 					m_OperationMode_bak;
	Uint8 				JOGAxisSelect;

	//通信相关状态
	bool 				PC_DSP_COM_OK;//false:停止; true:运行
	bool 				m_ReciveSuccess;
	int 					m_ReciveSuccessCounter;
	int 					m_ReciveLostCounter;
	int					sendcount;

	Uint8				CanSendCodeSign_B3;	//上位端可以发送代码标志
	Uint8				CanSendCodeSign_B4;
	Uint32 				NeedSendCount;		//ARM需要发送代码条数
	Uint32				STDSendCount;		//ARM/PC发送标志代码位置指针
	Uint32				SPCSendCount;		//ARM/PC发送手动、点动代码位置指针
	Uint32				LastSendCount;		//暂存ARM实际发送代码位置指针
	//下位反馈接收条
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
};//打包位执行完成返回标志
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
UINT16AXIS		EncoderError;		// 脰谩鹿芒脮陇鲁脽卤篓戮炉   1--卤篓戮炉

	Uint16 		NCCodeBreak;					//ARM路垄脣脥G麓煤脗毛虏禄脕卢脨酶
	Uint16 		InterpolationOver;			//虏氓虏鹿脕驴鲁卢5mm卤篓戮炉
	Uint16 		TrackRunOutError;			//鹿矛录拢脝芦脌毛卤篓戮炉
	Uint16 		HaveGetErrorDataSign;		//脪脩陆脴脠隆麓铆脦贸脢媒戮脻卤锚脰戮, when ARM ->RET
 	Uint16 		RealRunCount;				//脪脩陆脴脠隆麓铆脦贸脢媒戮脻脭脣脨脨脤玫脢媒	
	Uint16 		InsideRAMCheckError;		// 脛脷虏驴RAM 脨拢脩茅麓铆脦贸
	Uint16 		OutsideRAMCheckError;		// 脥芒虏驴RAM 脨拢脩茅麓铆脦贸
	Uint16 		MainErrorSign;				//脳脺麓铆脦贸卤锚脰戮
	Uint16		AlmHLimitPos;
	Uint16		AlmHLimitNeg;
	Uint16		AlmSLimitPos;
	Uint16		AlmSLimitNeg;
	UINT16AXIS 	ServoAlarm;							//脣脜路镁卤篓戮炉		1:卤篓戮炉脫脨脨搂
	Uint16 		CompDataCheckError;				// 非线性补偿数据校验错误
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
}ERROR_Main;					//报警结构组

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
extern OVERALLSIGN				OverallSign_Clone;		// 总体标志
extern SYSTEM					System_Clone;			//系统参数结构对象
extern ERROR_Main					Error_main_Clone;			//报警结构对象
extern LINEARCOMPENSATION 		LinearCompensation;
extern DACONVERT					DA_Convert;

extern INT40AXIS MonitorBuffer1[2000]/*,MonitorBuffer2[1000]*/;
