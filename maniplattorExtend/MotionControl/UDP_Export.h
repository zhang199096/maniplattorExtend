//file***************************************************************************************************
//                                                                                                       
//                                                                                                       
//        UDP_Export.h for  Master                  
//                                                                                                       
//                                                                                                       
//*******************************************************************************************************
//               For UDP Communication (PC<=>DSP)                                                                                        
//******* Copyright(C) 2015 GIBH ******************************************************************
//                                                                                                       
//      Ver 1.00    2016.01.05   Initial coding                                            Enjoy_Lu     
//*******************************************************************************************************


// #if defined( __cplusplus )
// extern "C" {
// #endif
// 
// #ifdef UDP_EXPORTS
// #define SDV_API extern  __declspec(dllexport)
// #else
// #define SDV_API extern  __declspec(dllimport)
// #endif
#include "..//MotionControl//UDP_Rerro.h"
	/*SDV_API*/  long __stdcall UDP_InitNet();
	/*SDV_API*/	 long __stdcall UDP_SetConnetIp(unsigned char Ip_connet );
	/*SDV_API  long __stdcall UDP_SendData(char*Buffer,unsigned char Length,unsigned char Ip_flg );*/
	/*SDV_API*/ long __stdcall UDP_SendData(char*Buffer, unsigned int Length, unsigned char Ip_flg);
	/*SDV_API */ long __stdcall SetStreamHOOK(StreamProc lpStreamFun,void*pUserData);
	long __stdcall UDP_DualCoreSendData(char*Buffer, unsigned int Length, unsigned char Ip_flg);
	void UDPSendTest(void);

// #if defined( __cplusplus )
// }
/*#endif*/
