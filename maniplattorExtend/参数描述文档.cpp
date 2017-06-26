void OnBnClickedMaininterfaceVotagecontrol()
{
	//DA Channel 1~9
	//DA Channel 1:LED1 Voltage output
	//DA Channel 2:LED2 Voltage output
	//DA Channel 3:AOTF Voltage output
	//DA Channel 4:Pockels Voltage output
	//DA Channel 5:Galvanometer1 Voltage; Triangular wave output(4~6V) Sync with AOTF or Pockels square wave
	//DA Channel 6:Galvanometer2 Voltage; Triangular wave output(4~6V)
	//DA Channel 7:Galvanometer3 Voltage; Triangular wave output(4~6V)
	//DA Channel 8:Galvanometer4 Voltage; Triangular wave output(4~6V)
	//DA Channel 9:PMT Voltage output

	GCODE  *GCodeSIPointer;
	DACONVERT  DA_Convert1;

	static Uint32 Voltage_Max = 1000;
	static Uint32 Voltage_Max_AOTF = 1000;//DC 0~10V; channel 3
	static Uint32 Voltage_Max_Pockels = 200;//DC 0~2V match 0~700V,Vmax=200.channel 4
	static Uint32 Voltage_Max_PMT = 125; //DC 0-1.25V 
	static Uint32 HighCount_Min = 9;//0.9us
	static Uint32 LowCount_Min = 5;//0.5us
	Uint32 V_Amplitude;
	
	
	UpdateData(true);
	
	DA_Convert1.VotageChannel = m_SET_VotageChannel;
	DA_Convert1.VotageValue = (DA_Convert1.VotageChannel == 9) ? (m_SET_VotageValue):(m_SET_VotageValue/2);	
	DA_Convert1.AutoSign = m_SET_AutoSign;
	DA_Convert1.Voltage_Max = m_SET_Voltage_Max;
	DA_Convert1.Voltage_Start = m_SET_Voltage_Start;
	DA_Convert1.CycleNum = m_SET_CycleNum;

	V_Amplitude = DA_Convert1.Voltage_Max - DA_Convert1.Voltage_Start;
	if(V_Amplitude < 10)
	{
		MessageBox(_T("Check Voltage_Max and Voltage_Start"));
		return;
	}
		
	DA_Convert1.RiseTime = m_SET_RiseTime;
	DA_Convert1.FallTime= m_SET_FallTime;
	DA_Convert1.HighKeepTime= m_SET_HkeepTime;
	DA_Convert1.LowKeepTime= m_SET_LkeepTime;
	
	DA_Convert1.PWMHighCount = m_SET_HighCount;
	DA_Convert1.PWMLowCount = m_SET_LowCount;	
	DA_Convert1.PWMPhase = m_SET_Phase;	

	if(DA_Convert1.VotageChannel <= 4)
	{
		if(DA_Convert1.AutoSign != 0)
		{
			MessageBox(_T("AutoSign need set to Zero if select this DA channel"));
			return;
		}
		
		if(DA_Convert1.VotageChannel == 4)//pockels DA voltage
		{	//pockels cell voltage < 200  (2.0V)
			DA_Convert1.VotageValue = (DA_Convert1.VotageValue > Voltage_Max_Pockels/2)?(Voltage_Max_Pockels/2):(DA_Convert1.VotageValue);
			DA_Convert1.VotageValue = DA_Convert1.VotageValue*0.78;//resistor flu...
		}

		if(DA_Convert1.VotageChannel == 3)//AOTF DA voltage
		{	//AOTF  voltage <(9.5V)
			DA_Convert1.VotageValue = (DA_Convert1.VotageValue > 950/2)?(950/2):(DA_Convert1.VotageValue);
			DA_Convert1.VotageValue = DA_Convert1.VotageValue*0.85;//resistor flu...
		}
		if (DA_Convert1.VotageChannel == 9) //1.25 PMT
		{//PMT voltage < 125  (1.25V)
			DA_Convert1.VotageValue = (DA_Convert1.VotageValue > Voltage_Max_PMT / 2) ? (Voltage_Max_PMT / 2) : (DA_Convert1.VotageValue);
			DA_Convert1.VotageValue = DA_Convert1.VotageValue;//resistor flu...
		}
	}
	else if(DA_Convert1.VotageChannel == 5)//AOTF
	{	//channel 5 Galvanometer sync with Out64 AOTF or Out63 Pockels
		if(DA_Convert1.AutoSign == 0x03)
		{	//bit1 and bit2 //channel 5 sync with AOTF
			if((DA_Convert1.PWMHighCount == 0)||(DA_Convert1.PWMLowCount == 0))
			{
				MessageBox(_T("need set AOTF PWM Value"));
				return;
			}
			DA_Convert1.PWMHighCount = (DA_Convert1.PWMHighCount < HighCount_Min)?(HighCount_Min):(DA_Convert1.PWMHighCount );
			DA_Convert1.PWMLowCount = (DA_Convert1.PWMLowCount < LowCount_Min)?(LowCount_Min):(DA_Convert1.PWMLowCount );
		}
		else if(DA_Convert1.AutoSign == 0x05)
		{	//bit1 and bit3 //channel 5 sync with Pockels
			if((DA_Convert1.PWMHighCount == 0)||(DA_Convert1.PWMLowCount == 0))
			{
				MessageBox(_T("need set Pockels PWM Value"));
				return;
			}
			DA_Convert1.PWMHighCount = (DA_Convert1.PWMHighCount < HighCount_Min)?(HighCount_Min):(DA_Convert1.PWMHighCount );
			DA_Convert1.PWMLowCount = (DA_Convert1.PWMLowCount < LowCount_Min)?(LowCount_Min):(DA_Convert1.PWMLowCount );
		}
		else if(DA_Convert1.AutoSign == 0x01)
		{	//bit1 //only channel 5
			DA_Convert1.AutoSign = 0x01;
			DA_Convert1.PWMHighCount = 0;
			DA_Convert1.PWMLowCount = 0;
			DA_Convert1.PWMPhase = 0;
		}
		else
		{	//no channel
			DA_Convert1.AutoSign = 0x00;
			DA_Convert1.PWMHighCount = 0;
			DA_Convert1.PWMLowCount = 0;
			DA_Convert1.PWMPhase = 0;
		}
	}

	if(DA_Convert1.AutoSign != 0x00)
	{
		if(DA_Convert1.Voltage_Max > 600)
		{
			MessageBox(_T("Voltage_Max<=600 when Triangular wave output"));
			return;
			DA_Convert1.Voltage_Max = Voltage_Max;
		}
		if(DA_Convert1.Voltage_Max < 510)
		{
			MessageBox(_T("Voltage_Max>=510 when Triangular wave output"));
			return;
			DA_Convert1.Voltage_Max = 510;
		}
		if(DA_Convert1.Voltage_Start < 10)
		{
			MessageBox(_T("Voltage_Start>=10 when Triangular wave output"));
			return;
			DA_Convert1.Voltage_Start = 400;
		}
		if(DA_Convert1.Voltage_Start > 490)
		{
			MessageBox(_T("Voltage_Start<=490 when Triangular wave output"));
			return;
			DA_Convert1.Voltage_Start = 490;
		}
		
		if(DA_Convert1.CycleNum < 10)
		{
			MessageBox(_T("CycleNum>10"));
			return;
			DA_Convert1.RiseTime = 10;
		}
		if(DA_Convert1.RiseTime < 1)
		{
			MessageBox(_T("RiseTime=1~1000"));
			return;
			DA_Convert1.RiseTime = 10;
		}
	}

	DA_Convert1.VotageValue = (DA_Convert1.VotageValue > Voltage_Max/2)?(Voltage_Max/2):(DA_Convert1.VotageValue);
	DA_Convert1.Voltage_Max = (DA_Convert1.Voltage_Max > Voltage_Max)?(Voltage_Max):(DA_Convert1.Voltage_Max);
	DA_Convert1.Voltage_Start = (DA_Convert1.Voltage_Start > Voltage_Max)?(Voltage_Max):(DA_Convert1.Voltage_Start);

	DA_Convert1.RiseTime = (DA_Convert1.RiseTime < 1)?(1):(DA_Convert1.RiseTime);
	DA_Convert1.FallTime= (DA_Convert1.FallTime < 1)?(1):(DA_Convert1.FallTime);
	DA_Convert1.HighKeepTime= (DA_Convert1.HighKeepTime < 1)?(1):(DA_Convert1.HighKeepTime);
	DA_Convert1.LowKeepTime= (DA_Convert1.LowKeepTime < 1)?(1):(DA_Convert1.LowKeepTime);

	OverallSign_Clone.NeedSendCount = OverallSign_Clone.STDReceiveDSPSendCount;
	OverallSign_Clone.STDSendCount =	OverallSign_Clone.STDSendCount;
	OverallSign_Clone.NCSign= STDCODERUN;	

	m_Control_Clone.DSPCodePacket_Code_G106(DA_Convert1);//for test voltage	
}

/***************************************************************************/
/*  Function name: DSPCodePacket_Code_G106()                                   */
/*  Argument:NO     	                                    */
/*  Return value:true/
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
	//#1 ,2 :Led #3 :AOTF #4:pockels  #5,6,7,8:振镜1，2，3，4  #9：PMT
	GCodeSIPointer->Sub_CMD1 = Wave_Para.VotageChannel; //对应相应通道 #1-9
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
	GCodeSIPointer->Sub_CMD2 = Wave_Para.VotageValue; //对应相应通道电压 （Wave_Para.AutoSign=0时有效）

	DSPCodePacket_Code_Feedrate(GCodeSIPointer,0);
	GCodeSIPointer->SendCount = OverallSign_Clone.NeedSendCount;  //对应所发G代码条数，从一递增

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
	GCodeSIPointer->EndPoint.Axis1_Clone_Linear_Z = Wave_Para.AutoSign;  //波形标志位 0：设置#通道电压 1：输出#通道波形 3：
	GCodeSIPointer->EndPoint.Axis2_Clone_Linear_A = Wave_Para.Voltage_Max; //设置电压最高值 0-1000=》0-10V
	GCodeSIPointer->EndPoint.Axis3_Clone_Optical_LensTrans = Wave_Para.Voltage_Start; //设置电压最高值 0-1000 && < Wave_Para.Voltage_Max
	GCodeSIPointer->EndPoint.Axis4_Clone_Optical_FilterTrans = Wave_Para.CycleNum;  //输出波形周期
	GCodeSIPointer->EndPoint.Axis5 = 0;
	GCodeSIPointer->EndPoint.Axis6_PlateEmpty_Destination_Z1_R = Wave_Para.PWMHighCount; //对应AOTF、pockels波形PWM 高位时间 uint :0.1us
	GCodeSIPointer->EndPoint.Axis7_PlateEmpty_Destination_Y = Wave_Para.PWMLowCount;//对应AOTF、pockels波形PWM 低位位时间 uint :0.1us （高位低位时间可以调节矩形波的占空比）
	GCodeSIPointer->EndPoint.Axis8_PlateEmpty_Destination_X = Wave_Para.PWMPhase; //对应AOTF、pockels波形PWM 相位差 uint :0.1us
	GCodeSIPointer->EndPoint.Axis9_Plate_Transfer_Z = Wave_Para.RiseTime; //对应所选#波形上升时间
	GCodeSIPointer->EndPoint.Axis10_Plate_Transfer_A = Wave_Para.FallTime;//对应所选#波形下降时间
	GCodeSIPointer->EndPoint.Axis11_PlateWaste_Source_X = Wave_Para.HighKeepTime;//对应所选#波形高位保持时间
	GCodeSIPointer->EndPoint.Axis12_PlateWaste_Source_Y = Wave_Para.LowKeepTime;//对应所选#波形低位保持时间

	return 1;
}