
#include "include.h"

void TermOffPwrAndPeriDeInit(void)
{	
	OFF_5V_PWR();
	OFF_GPS_PWR();
	OFF_GPS_BAT_PWR();
	OFF_WORK_LED();
	OFF_GSM_LED();
	OFF_GPS_LED();
	OFF_METER_PWR();
	SysTickDisable();
	
	RCC->AHBENR = 0x00;
	RCC->APB1ENR = 0x00000000;
	RCC->APB2ENR = 0x00000030;
}
void TermOnPwrAndPeriReInit(void)
{
	SysClkConfigStop();///停止模式唤醒后，HSE重新配置
	
	RCC->AHBENR = 0x14;
	RCC->APB1ENR = 0x18000000;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
						   RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE |
						   RCC_APB2Periph_AFIO, ENABLE);	
	ON_GPS_PWR();
	ON_GPS_BAT_PWR();
	ON_METER_PWR();
	UsartInit(LOCAL_USART,LOCAL_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	UsartInit(GPS_USART,GPS_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	UsartInit(GPRS_USART,GPRS_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	SysTickInit();
	SpiFramInit();
	AdcInit();
	DmaInit();
	SysTickEnable();
	GpsUartParaInit();
}

void TermEnterSleep(void)
{
	#ifndef SYS_PARA_INIT
	uint8 ok_ack[] = "OK";
	#endif
	uint8 i,normal_wake_flag = FALSE,break_flag = FALSE,term_lock_flag;
	uint16 tmp_acc_off_sec_counter;
	uint32 rtc_counter;

	term_lock_flag =  ((sys_work_para_struct.term_run_status_word & STATE_GPS_ANT_OFF)||
					   (sys_work_para_struct.term_run_status_word & STATE_SHELL_OFF)||
					   (sys_work_para_struct.term_run_status_word & STATE_SIMCARD_CHANGE)||
					   (sys_work_para_struct.no_simcard_falg));
		
	SysKeyDataSave();
	GsmDisLcpAndIntoSleep();
	LocalDebug("term into sleep...\r\n",StrLen("term into sleep...\r\n",0),LOCAL_TEST_DEBUG);
	TermOffPwrAndPeriDeInit();
	
	FeedWtd();
	RTC_WaitForLastTask();
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
	RTC_WaitForLastTask();
	
	pro_struct.aquire_comm_para_flag = FALSE;
	pro_struct.login_center_flag = FALSE;
	RamZero(gsm_misc_struct.gsm_mode_exe_flag,4);
	sys_work_para_struct.term_run_status_word &= ~STATE_A_GPS;
	
	gsm_misc_struct.sleep_rx_ring_falg = FALSE;
	rtc_counter = RTC_GetCounter();
	tmp_acc_off_sec_counter = sys_work_para_struct.acc_off_sec_counter;
	
	while(1)
	{	
		FeedWtd();
		RTC_WaitForLastTask();
		RTC_SetAlarm(rtc_counter++);	
		RTC_WaitForLastTask();
		PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI);
		sys_work_para_struct.acc_off_sec_counter += 1;

		if(sys_work_para_struct.acc_off_sec_counter == (sys_private_para_struct.acc_off_up_work_para_sec_timer - SYS_AHEAD_WAKEUP_SEC_TIMER - 2*SYS_TASK_SEC_TIMER))
		{
			ON_GPS_PWR();
		}
		
		if(sys_work_para_struct.acc_off_sec_counter >= (sys_private_para_struct.acc_off_up_work_para_sec_timer - SYS_AHEAD_WAKEUP_SEC_TIMER))
		{
			normal_wake_flag = TRUE;
			break_flag = TRUE;
		}
		
		if((sys_misc_run_struct.wake_up_by_exit_flag)||(!ACC_STATE()))
		{
			break_flag = TRUE;
		}
		
		if(sys_work_para_struct.term_self_reset_flag)
		{
			break_flag = TRUE;
		}
		
		if(break_flag)
		{
			sys_work_para_struct.lsnal_min_counter += (sys_work_para_struct.acc_off_sec_counter - tmp_acc_off_sec_counter) % 60;
			if(sys_work_para_struct.lsnal_min_counter >= 57600)
			{
				sys_work_para_struct.lsnal_min_counter = 57600;
			}
			
			sys_work_para_struct.lsnal_sys_comm_reset_min_counter += (sys_work_para_struct.acc_off_sec_counter - tmp_acc_off_sec_counter) % 60;
			if(term_lock_flag)
			{
				sys_work_para_struct.can_stop_heart_min_counter += (sys_work_para_struct.acc_off_sec_counter - tmp_acc_off_sec_counter) % 60;
			}

			if(sys_misc_run_struct.wake_up_by_exit_flag)
			{
				sys_misc_run_struct.wake_up_by_exit_flag = FALSE;
				sys_work_para_struct.acc_off_sec_counter = 0;
				sys_misc_run_struct.sys_cold_start_falg = TRUE;
				if(gsm_misc_struct.ring_low_counter > 0)
				{
					gsm_misc_struct.sleep_rx_ring_falg = TRUE;
				}
			
			}
			break;
		}
	}
	FeedWtd();
	TermOnPwrAndPeriReInit();
	
	if(sys_work_para_struct.term_self_reset_flag)
	{
		SysKeyDataSave();
		while(1)
		{
			NVIC_DISABLE();
		}
	}
	
	RTC_WaitForLastTask();
	RTC_ITConfig(RTC_IT_ALR,DISABLE);
	RTC_WaitForLastTask();
	FeedWtd();
	LocalDebug("term wake up...\r\n",StrLen("term wake up...\r\n",0),LOCAL_TEST_DEBUG);
	RtcGetCalendarTime(sys_work_para_struct.date_time);///获取时间
	
	sys_misc_run_struct.gps_need_datetime_check_flag = TRUE;
	
	#ifndef SYS_PARA_INIT
		if((sys_work_para_struct.date_time[3] >= 12)&&(!sys_work_para_struct.term_3_clock_reset_flag))
		{
			if(gsm_misc_struct.sleep_rx_ring_falg)
			{
				SysNoTaskDelay(5);
				GprsSendAtCmd(AT_ATH_INDEX,NULL,0,ok_ack,2);
			}
			
			if(normal_wake_flag)
			{
				LocalDebug("term 12 clock reset\r\n",StrLen("term 12 clock reset\r\n",0),LOCAL_TEST_DEBUG);
				sys_work_para_struct.term_3_clock_reset_flag = TRUE;
				TermReset();///3点复位
			}
		}
	#endif
	ftp_struct.ftp_upgrade_flag = FALSE;
	pro_struct.tx_struct.acc_on_tx_sec_counter = 0;
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		pro_struct.tx_struct.acc_on_tx_buf[i][0] = INVALID_VAL_FF;
	}
}
	
void GsmCsqCheckMonitor(uint16 past_sec)///GSM状态监测
{
	sys_misc_run_struct.gsm_csq_check_sec_counter += past_sec;
	if(sys_misc_run_struct.gsm_csq_check_sec_counter >= 1573)
	{
		sys_misc_run_struct.gsm_csq_check_sec_counter %= 1573;
		sys_misc_run_struct.gsm_csq_check_flag = TRUE;
	}
}

void TerminalHeart(uint16 past_sec)
{
	sys_misc_run_struct.up_heart_beat_sec_counter += past_sec;
	if(sys_misc_run_struct.up_heart_beat_sec_counter >= sys_private_para_struct.up_heart_beat_sec_timer)
	{
		sys_misc_run_struct.up_heart_beat_sec_counter %= sys_private_para_struct.up_heart_beat_sec_timer;
		ProUpHeartBeat();
	}
}
void AccMonitor(uint16 past_sec,uint8 time[])
{
	uint16 i,into_sleep_timer;
	
	if(!ACC_STATE())///ACC开
	{
		sys_work_para_struct.acc_on_sec_statistic_counter++;
		sys_work_para_struct.acc_on_sec_counter += past_sec;
		ON_5V_PWR();
		if(!(sys_work_para_struct.term_run_status_word & STATE_ACC_ON))
		{
			if(sys_work_para_struct.acc_on_sec_counter >= 5)
			{
				sys_misc_run_struct.again_into_sleep_flag = FALSE;
				sys_work_para_struct.acc_off_sec_counter = 0x00;
				sys_work_para_struct.term_run_status_word |= STATE_ACC_ON;
				ProUpAccOnOff(0X00);
				AccOnOffDataAppend(time,TRUE);
				LocalDebug("ACC on\r\n",StrLen("ACC on\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
		
		if(sys_private_para_struct.acc_on_up_work_para_sec_timer == 0)
		{
			sys_private_para_struct.acc_on_up_work_para_sec_timer = 120;
		}
		if(sys_work_para_struct.acc_on_sec_counter >= sys_private_para_struct.acc_on_up_work_para_sec_timer)
		{
			sys_work_para_struct.acc_on_sec_counter %= sys_private_para_struct.acc_on_up_work_para_sec_timer;
			ProUpWorkPara();
		}
	}
	else///ACC关
	{
		sys_work_para_struct.acc_off_sec_counter += past_sec;
		OFF_5V_PWR();
		if(sys_work_para_struct.term_run_status_word & STATE_ACC_ON)
		{
			if(sys_work_para_struct.acc_off_sec_counter >= 5)
			{
				sys_work_para_struct.acc_on_sec_counter = 0x00;
				sys_work_para_struct.term_run_status_word &= ~STATE_ACC_ON;
				ProUpAccOnOff(0X01);
				AccOnOffDataAppend(time,FALSE);
				LocalDebug("ACC off\r\n",StrLen("ACC off\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
		///拖车报警,按ACC开间隔上传工作参数
		
		if((sys_work_para_struct.term_run_status_word & STATE_TRAILER)&&
		   (sys_work_para_struct.acc_off_sec_counter >= sys_private_para_struct.acc_on_up_work_para_sec_timer))
		{
			sys_work_para_struct.acc_off_sec_counter = 0x00;
			ProUpWorkPara();
		}
		else///非拖车时,按ACC关间隔上传
		{
			if(sys_work_para_struct.acc_off_sec_counter >= (sys_private_para_struct.acc_off_up_work_para_sec_timer - SYS_AHEAD_WAKEUP_SEC_TIMER))
			{
				if(sys_work_para_struct.acc_off_sec_counter >= sys_private_para_struct.acc_off_up_work_para_sec_timer)
				{
					sys_work_para_struct.acc_off_sec_counter = 0;
					ProUpWorkPara();
					sys_misc_run_struct.again_into_sleep_flag = TRUE;
				}
			}
			else
			{
				if(sys_misc_run_struct.again_into_sleep_flag)
				{
					into_sleep_timer = SYS_DELAY_SLEEP_SEC_TIMER;
				}
				else
				{
					into_sleep_timer = sys_private_para_struct.acc_off_sleep_sec_timer;
				}
				 
				if(sys_work_para_struct.acc_off_sec_counter >= into_sleep_timer)
				{
					for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)///如果上行发送区有数据,则延迟5秒睡眠
					{
						if(pro_struct.tx_struct.re_tx_full_flag[i])
						{
							break;
						}
					}
					
					if(i != PRO_MAX_TX_BUF_ARRAY)
					{
						sys_work_para_struct.acc_off_sec_counter -= 2;
					}
					else
					{
						sys_misc_run_struct.again_into_sleep_flag = FALSE;
						sys_misc_run_struct.wake_up_by_exit_flag = FALSE;
						sys_misc_run_struct.term_enter_sleep_flag = TRUE;///终端进入休眠
					}
				}
			}
		}
	}
}
void PowerMonitor(uint16 past_sec)
{
	static uint16 s_pwr_down_sec_counter = 0x00;
	static uint16 s_pwr_on_sec_counter = 0x00;
	
	if(!PWR_STATE())
	{
		s_pwr_down_sec_counter += past_sec;
		if(s_pwr_down_sec_counter >= 300)///断外电，300秒报警
		{
			if(!(sys_work_para_struct.term_run_status_word & STATE_PWR_DOWN))
			{
				sys_work_para_struct.term_run_status_word |= STATE_PWR_DOWN;
				ProUpAlarm(ALRM_FLAG_PWR_DOWN);
				LocalDebug("power off alarm\r\n",StrLen("power off alarm\r\n",0),LOCAL_TEST_DEBUG);
			}
			s_pwr_down_sec_counter = 300;
		}
	}
	else
	{
		if(s_pwr_down_sec_counter > 3)///断外电3秒后,再重新上电,系统重启
		{
			LocalDebug("power on restart\r\n",StrLen("power on restart\r\n",0),LOCAL_TEST_DEBUG);
			TermReset();
		}
		s_pwr_on_sec_counter += past_sec;
		if(s_pwr_on_sec_counter >= 120)///上外电，120秒解除报警
		{
			if(sys_work_para_struct.term_run_status_word & STATE_PWR_DOWN)
			{
				sys_work_para_struct.term_run_status_word &= ~STATE_PWR_DOWN;
				ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_PWR_DOWN);
				LocalDebug("power off alarm cancel\r\n",StrLen("power off alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
	}
}
void ShellMonitor(uint16 past_sec)
{
	static uint8 shell_off_sec_counter = 0x00;
	static uint8 shell_on_sec_counter = 0x00;
	
	if(!SHELL_STATE())
	{ 
		shell_on_sec_counter = 0;
		shell_off_sec_counter += past_sec;
		if(shell_off_sec_counter >= 3)///拆盖，3秒报警
		{
			if(!(sys_work_para_struct.term_run_status_word & STATE_SHELL_OFF))
			{
				sys_work_para_struct.term_run_status_word |= STATE_SHELL_OFF;
				ProUpAlarm(ALRM_FLAG_SHELL_OFF);
				LocalDebug("shell off alarm\r\n",StrLen("shell off alarm\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
	}
	else
	{
		shell_off_sec_counter = 0;
		shell_on_sec_counter += past_sec;
		if(shell_on_sec_counter >= 60)///合上盖，60秒解除报警
		{
			if(sys_work_para_struct.term_run_status_word & STATE_SHELL_OFF)
			{
				sys_work_para_struct.term_run_status_word &= ~STATE_SHELL_OFF;
				ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_SHELL_OFF);
				LocalDebug("shell off alarm cancel\r\n",StrLen("shell off alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
	}
}
void GpsAntMonitor(uint16 past_sec)
{
	static uint8 ant_off_sec_counter = 0x00;
	static uint8 ant_on_sec_counter = 0x00;
	
	if(GPS_ANT_STATE())
	{
		ant_on_sec_counter = 0;
		ant_off_sec_counter += past_sec;
		sys_work_para_struct.gps_ant_off_sec_counter += past_sec;
		if(sys_work_para_struct.gps_ant_off_sec_counter >= 600)
		{
			sys_work_para_struct.gps_ant_off_sec_counter = 600;
		}
		if(ant_off_sec_counter >= 120)///去除天线，120秒报警
		{
			if(!(sys_work_para_struct.term_run_status_word & STATE_GPS_ANT_OFF))
			{
				sys_work_para_struct.term_run_status_word |= STATE_GPS_ANT_OFF;
				ProUpAlarm(ALRM_FLAG_ANT_OFF);
				LocalDebug("gps ant off alarm\r\n",StrLen("gps ant off alarm\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
	}
	else
	{
		ant_off_sec_counter = 0;
		ant_on_sec_counter += past_sec;
		if(ant_on_sec_counter >= 5)///接上天线，5秒解除报警
		{
			sys_work_para_struct.gps_ant_off_sec_counter = 0;
			if(sys_work_para_struct.term_run_status_word & STATE_GPS_ANT_OFF)
			{
				sys_work_para_struct.term_run_status_word &= ~STATE_GPS_ANT_OFF;
				ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_ANT_OFF);
				LocalDebug("gps ant off alarm cancel\r\n",StrLen("gps ant off alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
	}
}
void GpsMonitor(uint16 past_sec)///GPS模块故障检测,重启策略
{
	static uint8 s_sec_counter_1 = 0;
	static uint16 s_sec_counter_2 = 0;
	
	if(gps_struct.gps_rx_right_data_flag)
	{
		gps_struct.gps_rx_right_data_flag = FALSE;
		s_sec_counter_2 = 0;
		if(sys_work_para_struct.term_run_status_word & STATE_GPS_BREAKDOWN)
		{
			sys_work_para_struct.term_run_status_word &= ~STATE_GPS_BREAKDOWN;///GPS接收到数据，解除报警
			ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_GPS_BREAKDOWN);
			LocalDebug("gps module breakdown alarm cancel\r\n",StrLen("gps module breakdown alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
		}
	}
	else
	{
		s_sec_counter_2 += past_sec;
		if(s_sec_counter_2 >= GPS_BREAK_DOWM_SEC_TIMER)///GPS无数据,报警
		{
			if(!(sys_work_para_struct.term_run_status_word & STATE_GPS_BREAKDOWN))
			{
				sys_work_para_struct.term_run_status_word |= STATE_GPS_BREAKDOWN;
				ProUpAlarm(ALRM_FLAG_GPS_BREAKDOWN);
				LocalDebug("gps module breakdown alarm\r\n",StrLen("gps module breakdown alarm\r\n",0),LOCAL_TEST_DEBUG);
			}
			s_sec_counter_2 = GPS_BREAK_DOWM_SEC_TIMER;
		}
	}
	///重启策略
	s_sec_counter_1 += 	past_sec;
	if((s_sec_counter_2 >= SYS_TASK_SEC_TIMER)&&
	   (s_sec_counter_1 >= SYS_TASK_SEC_TIMER))///无数据N秒重启1次
	{
		if(s_sec_counter_2 < GPS_BREAK_DOWM_SEC_TIMER)
		{
			sys_work_para_struct.term_run_status_word &= ~STATE_A_GPS;
			sys_misc_run_struct.gps_need_datetime_check_flag = TRUE;
			gps_struct.sms_gprmc_ack[0] = '\0';
			LocalDebug("gps module restart\r\n",StrLen("gps module restart\r\n",0),LOCAL_TEST_DEBUG);
			s_sec_counter_1 = 0x00;
			OFF_GPS_PWR();///重启GPS模块
			USART_DeInit(GPS_USART);
			SysDelay(2*WAIT_1S);
			UsartInit(GPS_USART,GPS_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
			ON_GPS_PWR();
		}
	}
}
void GprsMonitor(uint16 past_sec)///GPRS模块无数据重启策略
{
	pro_struct.no_rx_data_sec_counter += past_sec;
	
	if(pro_struct.no_rx_data_sec_counter >= 5*SYS_TASK_SEC_TIMER)
	{
		LocalDebug("GPRS no rx center data restart\r\n",StrLen("GPRS no rx center data restart\r\n",0),LOCAL_TEST_DEBUG);
		pro_struct.no_rx_data_sec_counter = 0;
		gsm_misc_struct.cur_mode = POWER_INIT_MODE;///Gsm模块重启
	}
}

void SpeedMonitor(uint16 past_sec)
{
	static uint16 s_over_speed_sec_counter = 0;

	if((sys_private_para_struct.over_speed_alarm[0] == INVALID_VAL_FF)&&
	   (sys_private_para_struct.over_speed_alarm[1] == INVALID_VAL_FF))
	{///都为FF,则屏蔽超速检测
		goto RETURN_LAB;
	}
	
	if((sys_data_struct.gps_info_of_gprs[SPEED_INDEX] > sys_private_para_struct.over_speed_alarm[0])&&
	   (sys_work_para_struct.term_run_status_word & STATE_A_GPS))
	{
		s_over_speed_sec_counter += past_sec;
		
		if(s_over_speed_sec_counter >= sys_private_para_struct.over_speed_alarm[1])
		{
			if(!(sys_work_para_struct.term_run_status_word & STATE_OVER_SPEED))
			{
				sys_work_para_struct.term_run_status_word |= STATE_OVER_SPEED;
				ProUpAlarm(ALRM_FLAG_OVER_SPEED);
				LocalDebug("over speed alarm\r\n",StrLen("over speed alarm\r\n",0),LOCAL_TEST_DEBUG);
			}
			s_over_speed_sec_counter = sys_private_para_struct.over_speed_alarm[1];
		}
	}
	else
	{
		if(s_over_speed_sec_counter >= past_sec)
		{
			s_over_speed_sec_counter -= past_sec;
		}
		else
		{
			s_over_speed_sec_counter = 0;
			if(sys_work_para_struct.term_run_status_word & STATE_OVER_SPEED)
			{
				sys_work_para_struct.term_run_status_word &= ~STATE_OVER_SPEED;
				ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_OVER_SPEED);
				LocalDebug("over speed alarm cancel\r\n",StrLen("over speed alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
	}
RETURN_LAB:
	return;
}

void TrailerMonitor(uint16 past_sec)
{
	static uint16 s_trailer_sec_counter = 0;
	
	if((!(sys_work_para_struct.term_run_status_word & STATE_ACC_ON))&&
	   (sys_work_para_struct.term_run_status_word & STATE_A_GPS)&&
	   (sys_data_struct.gps_info_of_gprs[SPEED_INDEX] > 10))///ACC关，大于10公里
	{			
		s_trailer_sec_counter += past_sec;
		if(!(sys_work_para_struct.term_run_status_word & STATE_TRAILER))///发现拖车时,延时休眠
		{
			if(s_trailer_sec_counter > 10)
			{
				sys_misc_run_struct.again_into_sleep_flag = FALSE;
				if(sys_work_para_struct.acc_off_sec_counter >= sys_private_para_struct.acc_off_sleep_sec_timer - 240)
				{
					sys_work_para_struct.acc_off_sec_counter = sys_private_para_struct.acc_off_sleep_sec_timer - 240;
				}
			}
		}
		
		if(s_trailer_sec_counter >= 120)///120秒
		{
			if(!(sys_work_para_struct.term_run_status_word & STATE_TRAILER))
			{
				sys_work_para_struct.term_run_status_word |= STATE_TRAILER;
				ProUpAlarm(ALRM_FLAG_TRAILER);
				LocalDebug("trailer alarm\r\n",StrLen("trailer alarm\r\n",0),LOCAL_TEST_DEBUG);
			}
			s_trailer_sec_counter = 120;
		}
	}
	else
	{
		///ACC开，或者ACC关且速度小于10公里
		if(sys_work_para_struct.term_run_status_word & STATE_ACC_ON)
		{
			s_trailer_sec_counter = 0;
			if(sys_work_para_struct.term_run_status_word & STATE_TRAILER)
			{
				sys_work_para_struct.term_run_status_word &= ~STATE_TRAILER;
				ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_TRAILER);
				LocalDebug("trailer alarm cancel\r\n",StrLen("trailer alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
		else
		{
			if(s_trailer_sec_counter >= past_sec)
			{
				s_trailer_sec_counter -= past_sec;
			}
			else
			{
				s_trailer_sec_counter = 0;
				if(sys_work_para_struct.term_run_status_word & STATE_TRAILER)
				{
					sys_work_para_struct.term_run_status_word &= ~STATE_TRAILER;
					ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_TRAILER);
					LocalDebug("trailer alarm cancel\r\n",StrLen("trailer alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
				}
			}
		}
	}
}
void VoltageMonitor(uint16 past_sec)
{
	static uint16 s_low_voltage_sec_counter = 0;
	static uint16 s_high_voltage_sec_counter = 0;
	
	if((sys_private_para_struct.low_voltage_alarm[0] == INVALID_VAL_FF)&&
	   (sys_private_para_struct.low_voltage_alarm[1] == INVALID_VAL_FF))
	{///都为FF,则屏蔽低压检测
		goto AD_CONVERT_LAB;
	}
	
	if(sys_misc_run_struct.power_voltage_val < sys_private_para_struct.low_voltage_alarm[0])
	{
		s_low_voltage_sec_counter += past_sec;
		s_high_voltage_sec_counter = 10;
		if(s_low_voltage_sec_counter >= sys_private_para_struct.low_voltage_alarm[1])
		{
			if(!(sys_work_para_struct.term_run_status_word & STATE_LOW_VOLTAGE))
			{
				sys_work_para_struct.term_run_status_word |= STATE_LOW_VOLTAGE;
				ProUpAlarm(ALRM_FLAG_LOW_VOLTAGE);
				LocalDebug("low voltage alarm\r\n",StrLen("low voltage alarm\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
	}
	else
	{
		if(s_high_voltage_sec_counter >= past_sec)
		{
			s_high_voltage_sec_counter -= past_sec;
		}
		else
		{
			s_low_voltage_sec_counter = 0;
			s_high_voltage_sec_counter = 0;
			if(sys_misc_run_struct.power_voltage_val > sys_private_para_struct.low_voltage_alarm[0])
			{
				if(sys_work_para_struct.term_run_status_word & STATE_LOW_VOLTAGE)
				{
					sys_work_para_struct.term_run_status_word &= ~STATE_LOW_VOLTAGE;
					ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_LOW_VOLTAGE);
					LocalDebug("low voltage alarm cancel\r\n",StrLen("low voltage alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
				}
			}
		}
	}
AD_CONVERT_LAB:
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);///系统调用，开启1次AD转换
}

void AccOnOffDataAppend(uint8 time[],uint8 is_acc_on_falg)
{
	uint16 tmp_len;
	
	if(is_acc_on_falg)
	{
		if(sys_data_struct.acc_on_off_statistic[6] >= 2*ACC_ON_OFF_DAY_STATISTIC_COUNTER)
		{
			sys_data_struct.acc_on_off_statistic[6] -= 2;
			tmp_len = (sys_data_struct.acc_on_off_statistic[1] << 8)+sys_data_struct.acc_on_off_statistic[2];
			tmp_len -= 6;
			sys_data_struct.acc_on_off_statistic[1] = tmp_len >> 8;
			sys_data_struct.acc_on_off_statistic[2] = tmp_len;
		}
		
		if(sys_data_struct.acc_on_off_statistic[6] % 2)
		{
			goto RETURN_LAB;
		}
	}
	else
	{
		if((sys_data_struct.acc_on_off_statistic[6] % 2) ==0)
		{
			goto RETURN_LAB;
		}
	}
	
	tmp_len = (sys_data_struct.acc_on_off_statistic[1] << 8)+sys_data_struct.acc_on_off_statistic[2];
	
	if(tmp_len > ACC_ON_OFF_DAY_STATISTIC_LEN - 4)
	{
		goto RETURN_LAB;
	}
	
	MemCpy(sys_data_struct.acc_on_off_statistic+tmp_len,time,3);
	tmp_len += 3;
	sys_data_struct.acc_on_off_statistic[1] = tmp_len >> 8;
	sys_data_struct.acc_on_off_statistic[2] = tmp_len;
	sys_data_struct.acc_on_off_statistic[6]++;
	AccOnOffStatisticDataSave(tmp_len);
	if(!is_acc_on_falg)
	{
		sys_data_struct.acc_on_off_statistic[6] /= 2;
		ProUpWorkTimeStatistic(sys_data_struct.acc_on_off_statistic+3,tmp_len-3);
		sys_data_struct.acc_on_off_statistic[6] *= 2;
	}
RETURN_LAB:
		return;
}
void AccOnOffDataInit(uint8 date[])
{
	uint8 i= 0;
	
	sys_data_struct.acc_on_off_statistic[i++] = VALID_VAL_2A;
	sys_data_struct.acc_on_off_statistic[i++] = 0x00;
	sys_data_struct.acc_on_off_statistic[i++] = 0x07;	
	
	MemCpy(sys_data_struct.acc_on_off_statistic+3,date,3);
	i += 3;	
	sys_data_struct.acc_on_off_statistic[i++] = 0x00;
	
	AccOnOffStatisticDataSave(i);
}
void AccOnOffStatisticDataSave(uint16 len)
{
	uint8 res;
	uint16 bias_addr;
	
	sys_data_struct.acc_on_off_statistic[len] = U8SumCheck(sys_data_struct.acc_on_off_statistic,len);
	len += 1;
	bias_addr = ((uint8*)sys_data_struct.acc_on_off_statistic) - ((uint8*)sys_data_struct.gps_info_of_gprs);
	res = SpiFramWrite(SYS_DATA_STRUCT_START_ADDR+bias_addr,sys_data_struct.acc_on_off_statistic,len);
	if(!res)
	{
		LocalDebug("acc on/off statistic data save fail.\r\n",StrLen("acc on/off statistic data save fail.\r\n",0),LOCAL_TEST_DEBUG);
	}
}
void AccOnOffStatisticDataCheck(uint8 date[])
{
	uint8 check_val;
	uint16 tmp_len;
	
	if(sys_data_struct.acc_on_off_statistic[0] != VALID_VAL_2A)
	{
		goto ACC_DAY_SET_LAB;
	} 
	tmp_len = (sys_data_struct.acc_on_off_statistic[1] << 8)+sys_data_struct.acc_on_off_statistic[2];
	if(tmp_len > ACC_ON_OFF_DAY_STATISTIC_LEN - 1)
	{
		goto ACC_DAY_SET_LAB;
	}
	check_val = U8SumCheck(sys_data_struct.acc_on_off_statistic,tmp_len);
	if(check_val == sys_data_struct.acc_on_off_statistic[tmp_len])
	{
		return;
	}
ACC_DAY_SET_LAB:
	AccOnOffDataInit(date);
}
void DayChangeMonitor(uint8 date_time[])
{
	uint8 res, tmp_time[6] = {23,59,59,0,0,0};
	uint16 bias_addr,tmp_len;

	AccOnOffStatisticDataCheck(date_time);
	res = MemCmp(date_time,sys_data_struct.acc_on_off_statistic+3,3);
	if(res)
	{
		goto RETURN_LAB;
	}
	
	if(sys_work_para_struct.term_3_clock_reset_flag)
	{
		sys_work_para_struct.term_3_clock_reset_flag = FALSE;///清3点重启标志
		bias_addr = ((uint8*)&sys_work_para_struct.term_3_clock_reset_flag) - ((uint8*)sys_work_para_struct.acc_on_sec_statistic_counter);
		res = SpiFramWrite(SYS_WORK_PARA_STRUCT_START_ADDR+bias_addr,(uint8*)&sys_work_para_struct.term_3_clock_reset_flag,1);
		if(!res)
		{
			LocalDebug("clear 3 clock reset flag fail\r\n",StrLen("clear 3 clock reset flag fail\r\n",0),LOCAL_TEST_DEBUG);
		}
	}
	
	if(sys_work_para_struct.term_run_status_word & STATE_ACC_ON)
	{
		if(date_time[3] == 0x00)///正常过0点
		{
			AccOnOffDataAppend(tmp_time,FALSE);
			AccOnOffDataInit(date_time);
			AccOnOffDataAppend(tmp_time+3,TRUE);
			goto RETURN_LAB;
		}
	}
	
	tmp_len = (sys_data_struct.acc_on_off_statistic[1] << 8)+sys_data_struct.acc_on_off_statistic[2];
	sys_data_struct.acc_on_off_statistic[6] /= 2;
	
	if(tmp_len != sys_data_struct.acc_on_off_statistic[6]*6+4)
	{
		tmp_len = sys_data_struct.acc_on_off_statistic[6]*6+4+3;
	}
	
	ProUpWorkTimeStatistic(sys_data_struct.acc_on_off_statistic+3,tmp_len-3);
	AccOnOffDataInit(date_time);
	if(sys_work_para_struct.term_run_status_word & STATE_ACC_ON)
	{
		AccOnOffDataAppend(date_time+3,TRUE);
	}
RETURN_LAB:	
	return;
}
void LatLongPositionMonitor(uint16 past_sec)
{
	static uint8 s_counter = 0;
	uint8 tmp_gps_info[GPS_INFO_LEN];
	
	if(sys_work_para_struct.term_run_status_word & STATE_ACC_ON)
	{
		sys_misc_run_struct.tx_lat_long_sec_counter += past_sec;
		if(sys_misc_run_struct.tx_lat_long_sec_counter >= 15)///
		{
			sys_misc_run_struct.tx_lat_long_sec_counter %= 15;
			GetGpsInfo(tmp_gps_info);
			if(s_counter == 0)
			{
				MemCpy(gps_struct.gps_lat_long_position,tmp_gps_info+DATE_INDEX,6);
				gps_struct.gps_lat_long_position[6] = 0x0F;
			}
			MemCpy(gps_struct.gps_lat_long_position+7+s_counter*8,tmp_gps_info,8);
			s_counter++;
			if(s_counter == 0x08)
			{
				s_counter = 0;
				ProUpLatLongPosition(gps_struct.gps_lat_long_position,GPS_LAT_LONG_POS_LEN);
				RamZero(gps_struct.gps_lat_long_position,GPS_LAT_LONG_POS_LEN);
			}
		}
	}
}
void RingSmsMonitor(uint16 past_sec)
{
	if(gsm_misc_struct.first_deal_sms_flag)
	{
		gsm_misc_struct.first_deal_sms_sec_counter += past_sec;
		if(gsm_misc_struct.first_deal_sms_sec_counter >= 2*SYS_TASK_SEC_TIMER)
		{
			gsm_misc_struct.first_deal_sms_flag = FALSE;
			gsm_misc_struct.gsm_ring_low_ms_counter = 200;
			gsm_misc_struct.gsm_rx_sms_flag = TRUE;
		}
	}
	
	
	if(gsm_misc_struct.ring_low_counter > 0)
	{
		gsm_misc_struct.ring_low_sec_counter += past_sec;
		if(gsm_misc_struct.ring_low_sec_counter > 6)
		{
			if(gsm_misc_struct.ring_low_counter == 1)
			{
				gsm_misc_struct.gsm_rx_sms_flag = TRUE;
			}
			else
			{
				gsm_misc_struct.gsm_rx_ring_flag = TRUE;
			}
			gsm_misc_struct.ring_low_counter = 0;
		}
	}
}
void LsnalLockMonitor(uint16 past_sec)
{
	static uint16 lsnal_counter = 0;
	uint8 term_lock_flag;
	
	lsnal_counter += past_sec;
	if(lsnal_counter >= 60)
	{
		lsnal_counter = lsnal_counter % 60;
		sys_work_para_struct.lsnal_min_counter++;
		if(sys_work_para_struct.lsnal_min_counter >= 57600)
		{
			sys_work_para_struct.lsnal_min_counter = 57600;
		}
		
		sys_work_para_struct.lsnal_sys_reset_min_counter++;
		if(sys_work_para_struct.lsnal_sys_reset_min_counter >= 15)
		{
			sys_work_para_struct.lsnal_sys_reset_min_counter = 0;
			TermReset();
		}

		term_lock_flag = ((sys_work_para_struct.term_run_status_word & STATE_GPS_ANT_OFF)||
						  (sys_work_para_struct.term_run_status_word & STATE_SHELL_OFF)||
					      (sys_work_para_struct.term_run_status_word & STATE_SIMCARD_CHANGE)||
					      (sys_work_para_struct.no_simcard_falg));
		if(term_lock_flag)
		{
			sys_work_para_struct.can_stop_heart_min_counter++;
			if(sys_work_para_struct.can_stop_heart_min_counter >= 43200)
			{
				sys_work_para_struct.can_stop_heart_min_counter = 43200;
			}
		}
		else
		{
			sys_work_para_struct.can_stop_heart_min_counter = 0;
		}
		
		sys_work_para_struct.lsnal_sys_comm_reset_min_counter++;
		if(sys_work_para_struct.lsnal_sys_comm_reset_min_counter >= 1440)
		{
			sys_work_para_struct.lsnal_sys_comm_reset_min_counter = 0;
			SysCommParaInit();
			SysPrivateParaWrite();
			TermReset();
		}
	}
}
void KeyDataSave(uint16 past_sec)
{
	static uint16 sec_counter = 0;
	
	sec_counter += past_sec;
	if(sec_counter >= 120)
	{
		sec_counter = 0;
		SysKeyDataSave();
	}
}
void SysNoTaskDelay(uint8 delay_sec)
{
	uint8 past_sec = 0;
	uint16 rtc_cur_val,rtc_past_val;
	
	rtc_past_val = sys_misc_run_struct.sys_rtc_sec_counter;
	while((past_sec <= delay_sec))
	{
		FeedWtd();
		rtc_cur_val = sys_misc_run_struct.sys_rtc_sec_counter;
		past_sec = (rtc_cur_val + (65535 - rtc_past_val)) % 65535;
	}
}
void SysDelay(uint8 delay_sec)
{
	uint8 past_sec = 0;
	uint16 rtc_cur_val,rtc_past_val;
	
	sys_misc_run_struct.systask_delay_flag = TRUE;
	rtc_past_val = sys_misc_run_struct.sys_rtc_sec_counter;
	while((past_sec <= delay_sec))
	{
		FeedWtd();
		rtc_cur_val = sys_misc_run_struct.sys_rtc_sec_counter;
		past_sec = (rtc_cur_val + (65535 - rtc_past_val)) % 65535;
		
		DriverMain();
		if(!sys_misc_run_struct.gprs_delay_flag)
		{
			GprsMain();	
		}
		GpsMain();
		if(!sys_misc_run_struct.localcomm_delay_flag)
		{
			LocalCommMain();
		}
		CanMain();
	}
	sys_misc_run_struct.systask_delay_flag = FALSE;
}

void SysTaskMain(void)///系统任务
{
	uint8 date_time[6];
	uint16 past_sec;
	static uint16 sys_rtc_pre_val=0;
	static uint16 sys_rtc_cur_val=0;
	static uint16 sec_total = 0;
	
	sys_rtc_cur_val = sys_misc_run_struct.sys_rtc_sec_counter;///获取系统ms计数；
	past_sec = (sys_rtc_cur_val + (65535 - sys_rtc_pre_val)) % 65535;
	
	if(past_sec > 0)
	{
		if(past_sec < 10)
		{	
			sec_total += past_sec;	
			if(sec_total >= 600)///10分钟,校对1次RTC
			{
				sec_total = 0;
				sys_misc_run_struct.gps_need_datetime_check_flag = TRUE;
			}
			
			RtcGetCalendarTime(date_time);
			if(pro_struct.tx_struct.acc_on_tx_sec_counter < 2*SYS_TASK_SEC_TIMER)
			{
				pro_struct.tx_struct.acc_on_tx_sec_counter += past_sec;
			}
			TerminalHeart(past_sec);			///终端心跳
			/**
			sys_misc_run_struct.tmp_sec_counter += past_sec;
			if(sys_misc_run_struct.tmp_sec_counter > 60)
			{
				sys_misc_run_struct.tmp_sec_counter = 0;
				date_time[2] = d++;
				date_time[3] = 0x00;
				DayChangeMonitor(date_time);
			}
			**/	
			DayChangeMonitor(date_time);		///日更新检测
			AccMonitor(past_sec,date_time+3);	///ACC开关检测
			PowerMonitor(past_sec);				///断电检测
			ShellMonitor(past_sec);				///拆壳检测
			GpsAntMonitor(past_sec);			///GPS天线检测
			GpsMonitor(past_sec);				///GPS模块检测
			CanMonitor(past_sec);				///仪表故障检测
			GprsMonitor(past_sec);				///GPRS模块检测
			SpeedMonitor(past_sec);				///超速检测
			TrailerMonitor(past_sec);			///拖车检测
			VoltageMonitor(past_sec);			///电压检测
			GsmCsqCheckMonitor(past_sec);		///GSM CSQ检测
			LatLongPositionMonitor(past_sec);	///经纬度信息上传
			ProPeriodTx(past_sec);				///上行协议发送
			RingSmsMonitor(past_sec);			///电话短信监测
			LsnalLockMonitor(past_sec);			///盲区锁车监测
			KeyDataSave(past_sec);				///2分钟存1次关键数据
		}
		sys_rtc_pre_val = sys_rtc_cur_val;
	}
}
