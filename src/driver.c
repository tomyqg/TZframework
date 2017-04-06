#define DRIVER_GLOBAL

#include "include.h"

void GpioInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
/***	 
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	GPIO_DeInit(GPIOE);
	GPIO_DeInit(GPIOF);
***/	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD  | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	///�������
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;///GPS���õ�ؿ�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;///GPS���õ�ؿ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;///�ն�״ָ̬ʾLED�����ƽ�,�ͣ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;///GSM,LED�ƿ��ƽ�,�ߣ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;///GPS,LED�ƿ��ƽ�,�ͣ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; ///CT_LOCK,�̵�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; ///CT_GPS, GPS��Դ����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;///CT_5V,��س�����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; ///CT_LOCK_1,�̵�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;///WDIι��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;///CT_GSM��GSM�ػ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; ///CT_GSMMD,4V��Դ������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;///DIR_485��485ͨ�ŷ��򣬿�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;///CT_485��485��Դ����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	///��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;///PWR��Դ�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;///ACC�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;///RM_SHELL��ǲ���������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;///GSM,RING�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;///GPS,���߼������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
	
	RELAY_UNLOCK();///�ؼ̵�������
	RELAY_UNLOCK_1();
	OFF_GPS_LED();
	
	OFF_GPRS_PWR();///�ػ�
	HIGH_GPRS_IGT();
	UsartGprsDeInit();
	
	OFF_CAN_PWR();
}
void FeedWtd(void)
{
	sys_misc_run_struct.wdg_counter++;
	if(sys_misc_run_struct.wdg_counter >= 0xffffff)
	{
		while(1);
	}
	IWDG_ReloadCounter();
	FEED_WDT();	
}
void IwdgInit(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  IWDG_SetReload(2000);///1.6S
  IWDG_ReloadCounter();
  IWDG_Enable();
}

void DmaInit(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
   ///����DMAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    DMA_DeInit(DMA1_Channel1);		///DMA1  1ͨ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = ((u32)0x4001244C);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&sys_misc_run_struct.adc_conv_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    ///������ã�ʹ����1�������AD_Value[0]������2�������AD_Value[1]..
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    ///ѭ��ģʽ������Bufferд�����Զ��ص���ʼ��ַ��ʼ����
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    ///������ɺ�����DMAͨ��
    DMA_Cmd(DMA1_Channel1, ENABLE);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
}
void AdcInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	///����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			///ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		///����Ҫ����ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	///���������ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;			///�Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 4;			///����ͨ��ת�����г���
	ADC_Init(ADC1, &ADC_InitStructure);

///	ADC_TempSensorVrefintCmd(ENABLE);			///ADC�����¶ȴ�����ʹ��

	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_239Cycles5);

	ADC_DMACmd(ADC1, ENABLE);	/// ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
	ADC_Cmd(ADC1, ENABLE);	///����ADC

	ADC_ResetCalibration(ADC1);			///��ʼ��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1));///�ȴ���ʼ�����
	ADC_StartCalibration(ADC1);				///��ʼ��У׼
	while(ADC_GetCalibrationStatus(ADC1));		///�ȴ�У׼���
}
void SysClkConfigStop(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_HSEConfig(RCC_HSE_ON); /*HSESʹ��*/  
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); /*�ȴ�*/
	if(HSEStartUpStatus == SUCCESS) 
	{ 
		RCC_PLLCmd(ENABLE);/*ʹ��*/
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)== RESET); /*�ȴ�PLL��Ч*/      
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);/*��PLL��Ϊϵͳʱ��*/
		while(RCC_GetSYSCLKSource() != 0x08);/*�ȴ�*/
	} 
}
void RtcConfiguration(void)
{
	uint16 i = 800;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_DeInit();

	RCC_LSEConfig(RCC_LSE_ON);

	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
		if(i>0)
		{
			i--;
			FeedWtd();
			LongTimeDly(2000);
		}
		else
		{
			break;
		}
	}

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	RCC_RTCCLKCmd(ENABLE);

	RTC_WaitForSynchro();

	RTC_WaitForLastTask();

	RTC_ITConfig(RTC_IT_SEC, ENABLE);

	RTC_WaitForLastTask();

	RTC_SetPrescaler(32767);

	RTC_WaitForLastTask();
}
void RtcInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RtcConfiguration();
	RtcSetCalendarTime();

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 11;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
}
void ExitInit(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
    EXTI_ClearITPendingBit(EXTI_Line17);///�����жϽӵ���17���ⲿ�ж�   
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
    EXTI_Init(&EXTI_InitStructure);
	
	AFIO->EXTICR[1] |= 0X0400;///ѡ��E_6��Ϊ�ⲿ�ж�6������
	EXTI_ClearITPendingBit(EXTI_Line6);///E_6 PWR  
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
    EXTI_Init(&EXTI_InitStructure);
	
	AFIO->EXTICR[2] |= 0X0001;///ѡ��B_8��Ϊ�ⲿ�ж�8������
	EXTI_ClearITPendingBit(EXTI_Line8);///B_8 RING  
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
    EXTI_Init(&EXTI_InitStructure);
	
	AFIO->EXTICR[2] |= 0X0030;///ѡ��D_9��Ϊ�ⲿ�ж�9������
    EXTI_ClearITPendingBit(EXTI_Line9);///D_9,ACC 
    EXTI_InitStructure.EXTI_Line = EXTI_Line9;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
    EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
	/**
	EXTI_ClearITPendingBit(EXTI_Line13);  
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
    EXTI_Init(&EXTI_InitStructure);
  
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 13;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
	***/
}
void PvdInit(void)///��ѹ�ж�
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	///ռ�����ȼ��������ȼ��ɴ�ϵ����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_PVDLevelConfig(PWR_PVDLevel_2V9); 	/// �趨��ط�ֵ
	PWR_PVDCmd(ENABLE); /// ʹ��PVD 
	EXTI_StructInit(&EXTI_InitStructure); 
	EXTI_InitStructure.EXTI_Line = EXTI_Line16; /// PVD���ӵ��ж���16��
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; ///ʹ���ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;///��ѹ���ڷ�ֵʱ�����ж�
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; /// ʹ���ж���
	EXTI_Init(&EXTI_InitStructure); /// ��ʼ���жϿ�����
}
void RtcSetCalendarTime(void)
{
	struct tm d_t;
	time_t d_t_sec;
	
	RTC_ITConfig(RTC_IT_SEC, DISABLE);
	
	d_t.tm_year = 100 + sys_work_para_struct.date_time[0];
	d_t.tm_mon = sys_work_para_struct.date_time[1] - 1;
	d_t.tm_mday = sys_work_para_struct.date_time[2];
	d_t.tm_hour = sys_work_para_struct.date_time[3];
	d_t.tm_min = sys_work_para_struct.date_time[4];
	d_t.tm_sec = sys_work_para_struct.date_time[5];

	d_t_sec = mktime(&d_t);
	
	RTC_WaitForLastTask();
	RTC_SetCounter((u32)d_t_sec);
	RTC_WaitForLastTask();
	
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
}
void RtcGetCalendarTime(uint8 date_time[])
{
	time_t t_s;
	struct tm *d_t;
	#ifdef RTC_DEBUG
		char str_ch[256];
		uint8 str_len;
	#endif
	t_s = (time_t)RTC_GetCounter();
	d_t = localtime(&t_s);
	d_t->tm_year += 1900;
	
	date_time[0] = d_t->tm_year - 2000;
    date_time[1] =  d_t->tm_mon + 1;
    date_time[2] = d_t->tm_mday;
    date_time[3] = d_t->tm_hour;
    date_time[4] = d_t->tm_min;
    date_time[5] = d_t->tm_sec;
	
	#ifdef RTC_DEBUG
		FeedWtd();
		str_len = sprintf(str_ch,"%2d��%02d��%02d��%2dʱ%02d��%02d��\r\n",
						 date_time[0],
						 date_time[1],
						 date_time[2],
						 date_time[3],
						 date_time[4],
						 date_time[5]
						 );
		LocalUartFixedLenSend((uint8*)str_ch,str_len);
		FeedWtd();
	#endif
}
void SysTickInit(void)
{
	RCC_ClocksTypeDef RCC_ClocksStatus;
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	SysTick_Config(RCC_ClocksStatus.SYSCLK_Frequency/1000);
}
void SysTickDisable(void)
{
	SysTick->CTRL  &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}
void SysTickEnable(void)
{
	SysTick->CTRL  |= (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}
void UsartGprsDeInit(void)///Gprs�������ŷ���ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void UsartInit(USART_TypeDef* USARTx, u32 uart_bpr,uint8 data_bits,uint8 stop_bits,uint8 parity_check)///�����ʣ�����λ��,ֹͣλ����żУ��
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	switch (*(uint32_t*)&USARTx)
	{
		case LOCAL_USART_BASE:///LCD,PA.9,TX;PA.10,RX
		{
			g_local_uart_struct.rx_buf = local_uart_buf;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			
			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			break;
		}
		case GPS_USART_BASE:///GPS,PA.2,TX;PA.3,RX
		{
			g_gps_uart_struct.rx_buf = gps_uart_buf;
			
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			
			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			break;
		}
		case GPRS_USART_BASE:///GPRS,PB.10,TX;PB.11,RX
		{
			g_gprs_uart_struct.rx_buf = gprs_uart_buf;
			
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOB, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			
			NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			break;
		}
		case METER_USART_BASE:///METER,PC.10,TX;PC.11,RX
		{
			g_meter_uart_struct.rx_buf = meter_uart_buf;
			
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
 			GPIO_Init(GPIOC, &GPIO_InitStructure);
			
 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
 			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  		    GPIO_Init(GPIOC, &GPIO_InitStructure);
			
			NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = UART4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			break;
		}
		default:
		{
			goto RETURN_LAB;
		}
	}

	USART_InitStructure.USART_BaudRate = uart_bpr;
	USART_InitStructure.USART_WordLength = data_bits;
	USART_InitStructure.USART_StopBits = stop_bits;
	USART_InitStructure.USART_Parity = parity_check;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USARTx, &USART_InitStructure);
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

	USART_Cmd(USARTx, ENABLE);
RETURN_LAB:
	return;
}

void LocalDebug(uint8 data[],uint16 len,uint8 port_flag)
{
	FeedWtd();
	#ifdef SYS_PARA_INIT
		sys_misc_run_struct.local_debug_enalbe_flag = 3;
	
		if(sys_misc_run_struct.local_debug_enalbe_flag == port_flag)
		{
			LocalUartFixedLenSend(data,len);
		}
	#else
		if(sys_misc_run_struct.local_debug_enalbe_flag == port_flag)
		{
			LocalUartFixedLenSend(data,len);
		}
	#endif
	FeedWtd();
}
void LocalUartFixedLenSend(uint8 data[],uint16 len) 
{
	uint16 i;
	
	for(i=0;i<len;i++)
	{
		USART_SendData(LOCAL_USART,data[i]);
		while(USART_GetFlagStatus(LOCAL_USART,USART_FLAG_TXE) == RESET);
	}
}
void LocalUartParaInit(void)
{
	g_local_uart_struct.rx_counter = 0;
	g_local_uart_struct.rx_delay_time_counter = 0;
	g_local_uart_struct.rx_flag = INVALID_VAL_55;
}
void LocalUartIsRxDone(uint16 past_ms_val)
{
	if(g_local_uart_struct.rx_delay_time_counter > 0)
	{
		if(g_local_uart_struct.rx_delay_time_counter >= past_ms_val)
		{
			g_local_uart_struct.rx_delay_time_counter -= past_ms_val;
		}
		else
		{
			g_local_uart_struct.rx_delay_time_counter = 0;
		}
		
		if((g_local_uart_struct.rx_delay_time_counter == 0)&&(g_local_uart_struct.rx_counter > 0))
		{
			g_local_uart_struct.rx_flag = VALID_VAL_AA;	//-˵��һ֡����
		}
	}
}
uint16 GetLocalUartRxData(uint8 rx_data[])	//-ȡ�����ݱ�����
{
	uint16 rx_counter = 0;
	
	if(g_local_uart_struct.rx_flag == VALID_VAL_AA)
	{
		rx_counter = g_local_uart_struct.rx_counter;
		MemCpy(rx_data,g_local_uart_struct.rx_buf,rx_counter);
		LocalUartParaInit();
	}
	return rx_counter;
}

void GprsUartFixedLenSend(uint8 data[],uint16 len) 
{
	uint16 i;
	
	for(i=0;i<len;i++)
	{
		USART_SendData(GPRS_USART,data[i]);
		while(USART_GetFlagStatus(GPRS_USART,USART_FLAG_TXE) == RESET);
	}
}
void GprsUartParaInit(void)
{
	g_gprs_usart_struct.rx_head_point = 0;
	g_gprs_usart_struct.rx_tail_point = 0;
	g_gprs_uart_struct.rx_delay_time_counter = 0;
	g_gprs_uart_struct.rx_flag = INVALID_VAL_55;
}
void GprsUartIsRxDone(uint16 past_ms_val)
{
	uint16 rx_counter;
	
	if(g_gprs_uart_struct.rx_delay_time_counter > 0)
	{
		if(g_gprs_uart_struct.rx_delay_time_counter >= past_ms_val)
		{
			g_gprs_uart_struct.rx_delay_time_counter -= past_ms_val;
		}
		else
		{
			g_gprs_uart_struct.rx_delay_time_counter = 0;
		}
		rx_counter = (g_gprs_usart_struct.rx_head_point + GPRS_UART_BUF_LEN - g_gprs_usart_struct.rx_tail_point)%GPRS_UART_BUF_LEN;
		if((g_gprs_uart_struct.rx_delay_time_counter == 0)&&(rx_counter > 0))
		{
			g_gprs_uart_struct.rx_flag = VALID_VAL_AA;
		}
		
	}
}
uint16 GetGprsUartRxData(uint8 rx_data[])
{
	uint16 rx_counter = 0,ret_len,i,j,k;
	
	rx_counter = (g_gprs_usart_struct.rx_head_point + GPRS_UART_BUF_LEN - g_gprs_usart_struct.rx_tail_point)%GPRS_UART_BUF_LEN;
	if(gsm_misc_struct.cur_mode == AT_INIT_MODE)
	{
		for(i=0;i<rx_counter;i++)
		{
			rx_data[i] = g_gprs_uart_struct.rx_buf[(g_gprs_usart_struct.rx_tail_point+i)%GPRS_UART_BUF_LEN];
		}
		ret_len = rx_counter;
	}
	else
	{
		ret_len = 0;
		k = 0;
		while(rx_counter > 0)
		{
			if(g_gprs_uart_struct.rx_buf[g_gprs_usart_struct.rx_tail_point] == 0X7E)
			{
				rx_counter--;
				break;
			}
			else
			{
				rx_data[k++] = g_gprs_uart_struct.rx_buf[g_gprs_usart_struct.rx_tail_point];
				
				g_gprs_usart_struct.rx_tail_point = (g_gprs_usart_struct.rx_tail_point + 1)%GPRS_UART_BUF_LEN;
				rx_counter--;
			}
		}
		
		if(k >= 4)
		{
			ret_len = k;
			goto RET_LAB;
		}
		
		i = 1;
		while(rx_counter > 0)
		{
			if(g_gprs_uart_struct.rx_buf[(g_gprs_usart_struct.rx_tail_point+i)%GPRS_UART_BUF_LEN] == 0X7E)
			{
				break;
			}
			else
			{
				rx_counter--;
				i++;
			}
		}

		if(g_gprs_uart_struct.rx_buf[(g_gprs_usart_struct.rx_tail_point+i)%GPRS_UART_BUF_LEN] == 0X7E)
		{
			i++;
			
			for(j=0;j<i;j++)
			{
				rx_data[j] = g_gprs_uart_struct.rx_buf[(g_gprs_usart_struct.rx_tail_point+j)%GPRS_UART_BUF_LEN];
			}
			g_gprs_usart_struct.rx_tail_point = (g_gprs_usart_struct.rx_tail_point + i)%GPRS_UART_BUF_LEN;
			if(g_gprs_usart_struct.rx_tail_point == g_gprs_usart_struct.rx_head_point)
			{
				g_gprs_uart_struct.rx_flag = INVALID_VAL_55;
			}
			ret_len = i;
		}
		if(ret_len == 0)
		{
			g_gprs_uart_struct.rx_flag = INVALID_VAL_55;
		}
	}
RET_LAB:
	return ret_len;
}


void GpsUartParaInit(void)
{
	g_gps_uart_struct.rx_counter = 0;
	g_gps_uart_struct.rx_delay_time_counter = 0;
	g_gps_uart_struct.rx_flag = INVALID_VAL_55;
}
void GpsUartIsRxDone(uint16 past_ms_val)
{
	if(g_gps_uart_struct.rx_delay_time_counter > 0)
	{
		if(g_gps_uart_struct.rx_delay_time_counter >= past_ms_val)
		{
			g_gps_uart_struct.rx_delay_time_counter -= past_ms_val;
		}
		else
		{
			g_gps_uart_struct.rx_delay_time_counter = 0;
		}
		
		if((g_gps_uart_struct.rx_delay_time_counter == 0)&&(g_gps_uart_struct.rx_counter > 0))
		{
			g_gps_uart_struct.rx_flag = VALID_VAL_AA;
		}
	}
}
void GpsUsartIsRxDone(uint16 past_ms_val)
{
	if(g_gps_uart_struct.rx_delay_time_counter > 0)
	{
		if(g_gps_uart_struct.rx_delay_time_counter >= past_ms_val)
		{
			g_gps_uart_struct.rx_delay_time_counter -= past_ms_val;
		}
		else
		{
			g_gps_uart_struct.rx_delay_time_counter = 0;
		}
		
		if((g_gps_uart_struct.rx_delay_time_counter == 0)&&(g_gps_uart_struct.rx_counter > 0))
		{
			g_gps_uart_struct.rx_flag = VALID_VAL_AA;
		}
	}
}
uint16 GetGpsUartRxData(uint8 rx_data[])
{
	uint16 rx_counter = 0;
	
	if(g_gps_uart_struct.rx_flag == VALID_VAL_AA)
	{
		rx_counter = g_gps_uart_struct.rx_counter;
		MemCpy(rx_data,g_gps_uart_struct.rx_buf,rx_counter);
		GpsUartParaInit();
	}
	return rx_counter;
}


void MeterUartFixedLenSend(uint8 data[],uint16 len) 
{
	uint16 i;
	
	for(i=0;i<len;i++)
	{
		USART_SendData(METER_USART,data[i]);
		while(USART_GetFlagStatus(METER_USART,USART_FLAG_TXE) == RESET);
	}
}
void MeterUartParaInit(void)
{
	g_meter_uart_struct.rx_counter = 0;
	g_meter_uart_struct.rx_delay_time_counter = 0;
	g_meter_uart_struct.rx_flag = INVALID_VAL_55;
}
void MeterUartIsRxDone(uint16 past_ms_val)
{
	if(g_meter_uart_struct.rx_delay_time_counter > 0)
	{
		if(g_meter_uart_struct.rx_delay_time_counter >= past_ms_val)
		{
			g_meter_uart_struct.rx_delay_time_counter -= past_ms_val;
		}
		else
		{
			g_meter_uart_struct.rx_delay_time_counter = 0;
		}
		
		if((g_meter_uart_struct.rx_delay_time_counter == 0)&&(g_meter_uart_struct.rx_counter > 0))
		{
			g_meter_uart_struct.rx_flag = VALID_VAL_AA;
		}
		
	}
}
uint16 GetMeterUartRxData(uint8 rx_data[])
{
	uint16 rx_counter = 0;
	
	if(g_meter_uart_struct.rx_flag == VALID_VAL_AA)
	{
		rx_counter = g_meter_uart_struct.rx_counter;
		MemCpy(rx_data,g_meter_uart_struct.rx_buf,rx_counter);
		g_meter_uart_struct.rx_counter = 0x00;
		g_meter_uart_struct.rx_flag = INVALID_VAL_55;
	}
	return rx_counter;
}

void DriverMain(void)
{
	static uint16 sys_tick_pre_val=0;
	static uint16 sys_tick_cur_val=0;
	uint16 past_ms_val;
	
	sys_tick_cur_val = sys_misc_run_struct.sys_tick_ms_counter;///��ȡϵͳms������
	
	past_ms_val = (sys_tick_cur_val + (65535 - sys_tick_pre_val)) % 65535;	//-������ʱ���
	GpsUartIsRxDone(past_ms_val);
	LocalUartIsRxDone(past_ms_val);	//-����׼����ǰ�����ٶ��㹻��,ÿ�����жϵ�ʱ�乻��
	GprsUartIsRxDone(past_ms_val);		//-��ȡ������Ч��GPRS���͹����Ĵ�������
	MeterUartIsRxDone(past_ms_val);
	sys_tick_pre_val = sys_tick_cur_val;
}
