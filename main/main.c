#include "include.h"

int main(void)
{ 
	GpioInit();
	ON_GPS_PWR();///GPS电源，LOCAL 232电源

	#ifdef ENABLE_WATCHDOG
		IwdgInit();
	#endif
	
	UsartInit(LOCAL_USART,LOCAL_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	UsartInit(GPS_USART,GPS_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	UsartInit(GPRS_USART,GPRS_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	FeedWtd();
	LongTimeDly(20000);
	FeedWtd();
	
	LocalUartFixedLenSend("system start...\r\n",StrLen("system start...\r\n",0));

	SpiFramInit();
	SysParaRead();
	SysTickInit();
	RtcInit();
	
	EXTI_DeInit();
	ExitInit();

	SysVaryInit();
	AdcInit();
	DmaInit();

	PvdInit();
	
	while(1)
	{
		FeedWtd();
		GprsMain();		//-链接外网应该使用的就是这个
		DriverMain();	///驱动
		GpsMain();	
		FeedWtd();
		SysTaskMain();	///系统任务
		LocalCommMain();
		CanMain();
		
	}
}
