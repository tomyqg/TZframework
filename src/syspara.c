#define SYS_PARA_STRUCT_GLOBAL

#include "include.h"


const SERIAL_COMM_STRUCT serial_comm_struct[] =
 {
	{SYS_PARA_SMS_ALARM_CENTER_NUM_ID,0x0001},///报警短信中心
	{SYS_PARA_APN_ID,0x0002},///APN 
	{SYS_PARA_MASTER_IP_ID,0x0003},///主中心IP
	{SYS_PARA_MASTER_DNS_ID,0x0004},///主中心域名
	{SYS_PARA_SLAVER_IP_ID,0x0005},///副中心IP
	{SYS_PARA_SLAVER_DNS_ID,0x0006},///副中心域名
	{SYS_PARA_PORT_ID,0x0007},///Port
	{SYS_PARA_SLEEP_TIMER_ID,0x0008},///休眠时间
	{SYS_PARA_WORK_PARA_ID,0x0009},///工作参数上传
	{SYS_PARA_TERMINAL_ID,0x000A},///SIMCARD ID
	{SYS_PARA_PASSWORD_ID,0x000B},///密码
	{SYS_PARA_RECOVER_ID,0x000C},///恢复出厂设置
	{SYS_PARA_SOFTWARE_VERSION_ID,0x000D},///程序版本号(只读)
	{SYS_PARA_ACC_STATISTIC_ID,0x000E},///ACC统计时间
	{SYS_PARA_TERMINAL_SERIAL_ID,0x000F},///终端序列号
	{SYS_PARA_METER_ERR_TIMER_ID,0x0010},///仪表故障时间
	{SYS_PARA_HARDWARE_VERSION_ID,0x0011},///硬件版本号
	{SYS_PARA_OVER_SPEED_ID,0x0012},///超速参数
	{SYS_PARA_LOW_VOLTAGE_ALARM_ID,0x0013},///低压报警参数
	{SYS_PARA_LSNAL_LOCK_MIN_ID,0x0014},///盲区被动锁车时间，分钟
	{SYS_PARA_LSNAL_LOCK_ENABLE_ID,0x0015},///盲区被动锁车使能
	{SYS_PARA_DEBUG_ENABLE_ID,0x0100},///调试使能
	{SYS_PARA_RESET_ID,0x0101}///终端复位
};
///{"13913886859"},
const SYS_CONST_PARA_STRUCT	sys_const_para_struct = 
								{
								 {"8613800250500"},
								 {"15251861104"},
								 {"106380005668"},
								 {"TIZA_XGDLPDJ_V32_160527"}
								};

void SysCommParaInit(void)
{
	uint8 sms_alarm_center_num[] = {"106380005668"};	///短信报警中心
	
	uint8 master_ip_dns[] = {'I',58,218,196,207};///徐工1.0 Y表示是域名，I表示是IP
	uint8 slaver_ip_dns[] = {'I',211,103,127,149};
	uint8 port[2] = {0x19,0xc8}; ///6600
	
	#ifdef SYS_PARA_INIT
		///uint8 term_id[] = {0x03,0x2A,0xB5,0xDF,0x9F};///13601464223
		///uint8 term_id[] = {0x03,0x29,0xAB,0xF3,0xC3};///13584036803
		///uint8 term_id[] = {0x03,0x39,0x9c,0xc7,0xc2};///13851477954
		///uint8 term_id[] = {0x03,0x3A,0xAF,0x40,0x98};///13869465752
    uint8 term_id[] = {0x03,0x8D,0x14,0xEE,0x70};
	#else
		uint8 term_id[] = {0xFF,0xFF,0xFF,0xFF,0xFF};
	#endif
	uint8 apn[] = {"CMNET"};
	uint8 gprs_login_user_name[] = {"JSTZ"};
	uint8 gprs_login_password[] = {"JSTZ"};
	
	MemCpy(sys_private_para_struct.terminal_id,term_id,5);
	MemCpy(sys_private_para_struct.sms_alarm_center_num,sms_alarm_center_num,StrLen(sms_alarm_center_num,0)+1);
	MemCpy(sys_private_para_struct.master_ip_dns,master_ip_dns,5);///IP
    ///MemCpy(sys_private_para_struct.master_ip_dns,master_ip_dns,StrLen(master_ip_dns,0)+1);///DNS时启用
	MemCpy(sys_private_para_struct.slaver_ip_dns,slaver_ip_dns,5);///IP
    ///MemCpy(sys_private_para_struct.slaver_ip_dns,slaver_ip_dns,StrLen(slaver_ip_dns,0)+1);///DNS时启用
	MemCpy(sys_private_para_struct.port,port,2);
	MemCpy(sys_private_para_struct.apn,apn,StrLen(apn,0)+1);
	MemCpy(sys_private_para_struct.gprs_login_user_name,gprs_login_user_name,StrLen(gprs_login_user_name,0)+1);
	MemCpy(sys_private_para_struct.gprs_login_password,gprs_login_password,StrLen(gprs_login_password,0)+1);

}								
void SysPrivateParaInit(void)
{
	SysCommParaInit();
	
	sys_private_para_struct.terminal_password = 111111;

	sys_private_para_struct.up_heart_beat_sec_timer = 30;///30秒无数据上传，则上传1次心跳
	sys_private_para_struct.over_speed_alarm[0] = 120;///超速报警值
	sys_private_para_struct.over_speed_alarm[1] = 30; ///超速报警秒定时器，(超速报警值,超速报警秒定时器若都为0XFF,则关闭超速功能)
	sys_private_para_struct.low_voltage_alarm[0] = 220;///低压报警值220代表22V，110代表11V
	sys_private_para_struct.low_voltage_alarm[1] = 255;///低压报警值时间255秒
	sys_private_para_struct.meter_comm_breakdown_alarm_sec_timer = 300;///仪表故障报警时间300秒
	sys_private_para_struct.acc_off_sleep_sec_timer = 600;///ACC关，进入休眠时间600秒
	sys_private_para_struct.acc_on_up_work_para_sec_timer = 120;///工作参数ACC开上传间隔2分钟
	sys_private_para_struct.acc_off_up_work_para_sec_timer = 3600;///工作参数ACC关上传间隔60分钟
	sys_private_para_struct.lsnal_lock_enable_flag = 0;///盲区被动锁车使能
	sys_private_para_struct.lsnal_min_timer = 43200;///盲区分钟计数，30天，72*60分钟
	RamClear(sys_private_para_struct.sim_card_imsi,SIM_CARD_IMSI_LEN);
}
			
void SysWorkParaInit(void)
{
	uint8 default_gps_of_gprs_info[] = {0x01,0xE8,0x2C,0x21,0x07,0x13,0xB9,0x7A,0x00,0x00,0x00,0x00};
							/*** 
							  0x01,0xE8,0x2C,0x21,///纬度
							  0x07,0x13,0xB9,0x7A,///经度
							  0x00,///速度
							  0x00,///方向
							  0x00,0x00///海拔
							***/  
	uint8 date_time[] = {0x10,0x05,0x1b,0x0E,0x08,0x08};///年月日时分秒
	sys_work_para_struct.acc_on_sec_statistic_counter = 0x00;
	sys_work_para_struct.acc_on_sec_counter = 0x00;
	sys_work_para_struct.acc_off_sec_counter = 0x00;
	
	MemCpy(sys_data_struct.gps_info_of_gprs,default_gps_of_gprs_info,sizeof(default_gps_of_gprs_info));
	sys_data_struct.acc_on_off_statistic[0] = INVALID_VAL_FF;
	sys_data_struct.lsnal_data[0] = INVALID_VAL_FF;
	sys_work_para_struct.lsnal_head_page = 0;
	sys_work_para_struct.lsnal_tail_page = 0;
	sys_work_para_struct.lsnal_min_counter = 0;
	sys_work_para_struct.lsnal_sys_reset_min_counter = 0;
	sys_work_para_struct.lsnal_sys_must_reset_sec_counter = 0;
	sys_work_para_struct.lsnal_sys_comm_reset_min_counter = 0;
	sys_work_para_struct.can_stop_heart_min_counter = 0X00;
	sys_work_para_struct.term_run_status_word = 0x06000000;///状态字
	MemCpy(sys_work_para_struct.date_time,date_time,6);
	sys_work_para_struct.track_of_acc_on_sec_timer = 0;///追踪秒定时器，为0，则关闭追踪功能
	sys_work_para_struct.track_of_acc_on_sec_counter = 0x00;
	sys_work_para_struct.sms_lock_car_flag = INVALID_VAL_FF;
	sys_work_para_struct.term_3_clock_reset_flag = FALSE;
	sys_work_para_struct.term_self_reset_flag = FALSE;
	sys_work_para_struct.no_simcard_falg = FALSE;
	sys_work_para_struct.gps_ant_off_sec_counter = 0;
	
	lock_record_struct.flag_2a = INVALID_VAL_FF;///清锁车标志
}

uint8 LockRecordRead(void)
{
	uint8 res;
	
	res = SpiFramRead(LOCK_RECORD_STRUCT_ADDR,(uint8*)&lock_record_struct,sizeof(LOCK_RECORD_STRUCT));
	
	return res;
}
uint8 LockRecordWrite(void)
{
	uint8 res;
	
	res = SpiFramWrite(LOCK_RECORD_STRUCT_ADDR,(uint8*)&lock_record_struct,sizeof(LOCK_RECORD_STRUCT));
	
	return res;
}
void UnLockRecord(void)
{
	lock_record_struct.lock_flag = 0x00;
	lock_record_struct.lock_reason = 0x00;
	LockRecordWrite();
}
void LockRecord(uint8 lock_method,uint8 level,uint8 reason)
{
	uint8 d_t[6];

	lock_record_struct.flag_2a = VALID_VAL_2A;
	lock_record_struct.lock_flag = 0x01;
	RtcGetCalendarTime(d_t);
	MemCpy(lock_record_struct.date_time,d_t,6);
	lock_record_struct.lock_method = lock_method;
	lock_record_struct.lock_level = level;
	lock_record_struct.lock_reason = reason;
	LockRecordWrite();
}
void SysBootParaRead(void)
{
	FlashRead(SYS_BOOT_PARA_STRUCT_START_ADDR,(uint8*)&sys_boot_para_struct,sizeof(SYS_BOOT_PARA_STRUCT));
}
uint8 SysBootParaWrite(void)
{
	#ifdef SYS_PARA_INIT
		return TRUE;
	#else
	uint8 res;
	
	res = FlashErase(SYS_BOOT_PARA_STRUCT_START_ADDR);
	if(res)
	{
		res = FlashWrite(SYS_BOOT_PARA_STRUCT_START_ADDR,(uint8*)&sys_boot_para_struct,sizeof(SYS_BOOT_PARA_STRUCT));
	}
	
	return res;
	#endif
}
uint8 SysSerialRead(void)
{
	uint8 res;
	
	res = SpiFramRead(SYS_SERIAL_STRUCT_ADDR,(uint8*)&sys_serial_struct,sizeof(SYS_SERIAL_STRUCT));
	
	return res;
}	
uint8 SysSerialWrite(void)
{
	uint8 res;
	
	res = SpiFramWrite(SYS_SERIAL_STRUCT_ADDR,(uint8*)&sys_serial_struct,sizeof(SYS_SERIAL_STRUCT));
	
	return res;
}

uint8 SysPrivateParaRead(void)
{
	uint8 res;
	uint16 tmp_len;

	tmp_len = sizeof(SYS_PRIVATE_PARA_STRUCT);
	res = SpiFramRead(SYS_PRIVATE_PARA_STRUCT_START_ADDR,(uint8*)&sys_private_para_struct,tmp_len);
	
	return res;
}
uint8 SysPrivateParaWrite(void)
{
	uint8 res;
	uint16 tmp_len;
	
	tmp_len = sizeof(SYS_PRIVATE_PARA_STRUCT);
	sys_private_para_struct.check_val = U8SumCheck(sys_private_para_struct.sms_alarm_center_num,tmp_len-4);
	res = SpiFramWrite(SYS_PRIVATE_PARA_STRUCT_START_ADDR,sys_private_para_struct.sms_alarm_center_num,tmp_len);
	
	return res;
}
uint8 SysWorkParaRead(void)
{
	uint8 res;
	
	res = SpiFramRead(SYS_WORK_PARA_STRUCT_START_ADDR,(uint8*)&sys_work_para_struct,sizeof(SYS_WORK_PARA_STRUCT));
	
	return res;
}
uint8 SysWorkParaWrite(void)
{
	uint8 res;
	
	res = SpiFramWrite(SYS_WORK_PARA_STRUCT_START_ADDR,(uint8*)&sys_work_para_struct,sizeof(SYS_WORK_PARA_STRUCT));
	
	return res;
}

uint8 SysDataRead(void)
{
	uint8 res;
	
	res = SpiFramRead(SYS_DATA_STRUCT_START_ADDR,(uint8*)&sys_data_struct,sizeof(SYS_DATA_STRUCT));
	
	return res;
}
uint8 SysDataWrite(void)
{
	uint8 res;
	
	res = SpiFramWrite(SYS_DATA_STRUCT_START_ADDR,(uint8*)&sys_data_struct,sizeof(SYS_DATA_STRUCT));
	
	return res;
}
void SysParaRead(void)
{
	uint8 res;
	uint32 tmp_val;
	uint16 tmp_len;
	
	#ifndef SYS_PARA_INIT
	SysBootParaRead();
	if(sys_boot_para_struct.sys_para_init_flag == VALID_VAL_DWORD_AA)///参数还未初始化
	{
	#endif
		SysPrivateParaInit();
		
		SysWorkParaInit();
		
		CanDataInit();
		
		LockRecordWrite();
		
		res = SysPrivateParaWrite();
		if(!res)
		{
			goto RESET_LAB;
		}
		
		res = SysWorkParaWrite();
		if(!res)
		{
			goto RESET_LAB;
		}
		
		res = SysDataWrite();
		if(!res)
		{
			goto RESET_LAB;
		}
		
		sys_boot_para_struct.sys_para_init_flag = INVALID_VAL_DWORD_55;
		res = SysBootParaWrite();
		if(!res)
		{
			goto RESET_LAB;
		}
	#ifndef SYS_PARA_INIT
	}
	#endif
	res = SysSerialRead();
	if(!res)
	{
		goto RESET_LAB;
	}
	
	res = SysPrivateParaRead();
	if(res)
	{
		tmp_len = sizeof(SYS_PRIVATE_PARA_STRUCT);
		tmp_val = U8SumCheck(sys_private_para_struct.sms_alarm_center_num,tmp_len-4);
		if(sys_private_para_struct.check_val != tmp_val)
		{
			sys_boot_para_struct.sys_para_init_flag = VALID_VAL_DWORD_AA;
			SysBootParaWrite();
			goto RESET_LAB;
		}
	}
	else
	{
		goto RESET_LAB;
	}
	
	res = SysWorkParaRead();
	if(!res)
	{
		goto RESET_LAB;
	}
	res = SysDataRead();
	if(!res)
	{
		goto RESET_LAB;
	}
	res = LockRecordRead();
	if(!res)
	{
		goto RESET_LAB;
	}
	return;
	
RESET_LAB:
	while(1)
	{
		NVIC_DISABLE();
	}
}
void SysVaryInit(void)
{
	uint8 i;

	sys_work_para_struct.term_run_status_word &= ~STATE_A_GPS;
	sys_misc_run_struct.gps_need_datetime_check_flag = TRUE;
	MemCpy(sys_misc_run_struct.link_center_ip,sys_private_para_struct.master_ip_dns+1,4);
	gsm_misc_struct.cur_mode = POWER_INIT_MODE;
	gsm_misc_struct.gsm_mode_exe_flag[0] = AT_NOT_EXE;
	pro_struct.link_center_ip_index = MASTER_IP_INDEX;
	gsm_misc_struct.gsm_module_reset_counter = 0;
	sys_misc_run_struct.again_into_sleep_flag = FALSE;
	
	sys_misc_run_struct.sys_tick_ms_counter = 0;
	sys_misc_run_struct.sys_rtc_sec_counter = 0;
	for(i=0;i<4;i++)
	{
		sys_misc_run_struct.adc_conv_buf[i] = 0;
	}
	sys_misc_run_struct.power_voltage_val = 0;
	sys_misc_run_struct.battery_voltage_val = 0;
	sys_misc_run_struct.gsm_csq_val = 0;
	sys_misc_run_struct.up_heart_beat_sec_counter = 0;
	
	sys_misc_run_struct.tx_lat_long_sec_counter = 50;///上行工作参数，经纬度，心跳数据错开
	sys_misc_run_struct.up_heart_beat_sec_counter = 8;
	pro_struct.tx_struct.acc_on_tx_sec_counter = 0;
	
	if(sys_work_para_struct.term_self_reset_flag)
	{
		sys_work_para_struct.term_self_reset_flag = FALSE;
		if(sys_work_para_struct.acc_off_sec_counter > sys_private_para_struct.acc_off_up_work_para_sec_timer)
		{
			sys_work_para_struct.acc_off_sec_counter = 0;
			sys_misc_run_struct.sys_cold_start_falg = TRUE;
		}
	}
	else
	{
		sys_work_para_struct.acc_off_sec_counter = 0;
		sys_misc_run_struct.sys_cold_start_falg = TRUE;
	}
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
		pro_struct.tx_struct.acc_on_tx_buf[i][0] = INVALID_VAL_FF;
	}
	sys_misc_run_struct.wdg_counter = 0;
}
void SysReset(void)
{
	SysNoTaskDelay(3);
	while(1)
	{
		NVIC_DISABLE();
	}
}
void TermReset(void)
{
	sys_work_para_struct.term_self_reset_flag = TRUE;
	SysKeyDataSave();
	ProLsnalSysExit();
	SysReset();
}
void TermReset2(void)
{
	sys_work_para_struct.term_self_reset_flag = FALSE;
	SysKeyDataSave();
	ProLsnalSysExit();
	SysReset();
}
void SysKeyDataSave(void)
{	
	RtcGetCalendarTime(sys_work_para_struct.date_time);
	MemCpy(sys_data_struct.gps_info_of_gprs+DATE_INDEX,sys_work_para_struct.date_time,6);
	SysWorkParaWrite();
	///SysDataWrite();
	SpiFramWrite(SYS_DATA_STRUCT_START_ADDR,sys_data_struct.gps_info_of_gprs,LEN_64);
}
/***系统参数***/
uint8 GetSmsAlarmCenterNum(uint8 data[])
{
	uint8 tmp_len;
	
	tmp_len = StrLen(sys_private_para_struct.sms_alarm_center_num,LEN_16);
	MemCpy(data,sys_private_para_struct.sms_alarm_center_num,tmp_len);
	
	return tmp_len;
}
uint8 GetApn(uint8 data[])
{
	uint8 tmp_len;
	
	tmp_len = StrLen(sys_private_para_struct.apn,LEN_32);
	MemCpy(data,sys_private_para_struct.apn,tmp_len);
	
	return tmp_len;
}
uint8 GetMasterIp(uint8 data[])
{
	MemCpy(data,sys_private_para_struct.master_ip_dns+1,4);
	
	return 4;
}
uint8 GetSlaverIp(uint8 data[])
{
	MemCpy(data,sys_private_para_struct.slaver_ip_dns+1,4);
	
	return 4;
}
uint8 GetPort(uint8 data[])
{
	data[0] = sys_private_para_struct.port[0];
	data[1] = sys_private_para_struct.port[1];
	
	return 2;
}
uint8 GetTerminalId(uint8 data[])
{
	MemCpy(data,sys_private_para_struct.terminal_id,TERMINAL_ID_LEN);
	
	return TERMINAL_ID_LEN;
}
uint8 GetTerminalPassword(uint8 data[])
{
	data[0] = sys_private_para_struct.terminal_password >> 24;
	data[1] = sys_private_para_struct.terminal_password >> 16;
	data[2] = sys_private_para_struct.terminal_password >> 8;
	data[3] = sys_private_para_struct.terminal_password;
	
	return 4;
}
uint8 GetSoftWareVersion(uint8 data[])
{
	uint8 tmp_len;
	
	tmp_len = StrLen((uint8*)sys_const_para_struct.software_version,LEN_32);
	MemCpy(data,(uint8*)sys_const_para_struct.software_version,tmp_len);
	
	return tmp_len;
}
uint8 GetTerminalSerialNum(uint8 data[])
{
	uint8 tmp_len,i;
	
	for(i=0;i<LEN_20;i++)
	{
		if(sys_serial_struct.terminal_serial_num[i] == '\0')
		{
			break;
		}
	}
	
	if(i == LEN_20)
	{
		tmp_len = 0x00;
	}
	else
	{
		tmp_len = i;
		MemCpy(data,sys_serial_struct.terminal_serial_num,i);
	}
	
	return tmp_len;
}
uint8 GetHardwareVersion(uint8 data[])
{
	uint8 tmp_len,i;
	
	for(i=0;i<LEN_12;i++)
	{
		if(sys_serial_struct.hardware_version[i] == '\0')
		{
			break;
		}
	}
	
	if(i == LEN_12)
	{
		tmp_len = 0x00;
	}
	else
	{
		tmp_len = i;
		MemCpy(data,sys_serial_struct.hardware_version,i);
	}
	
	return tmp_len;
}
uint8 GetSleepTimer(uint8 data[])
{
	data[0] = sys_private_para_struct.acc_off_sleep_sec_timer >> 8;
	data[1] = sys_private_para_struct.acc_off_sleep_sec_timer;
	
	return 2;
}
uint8 GetAccAtatistic(uint8 data[])
{
	data[0] = sys_work_para_struct.acc_on_sec_statistic_counter >> 24;
	data[1] = sys_work_para_struct.acc_on_sec_statistic_counter >> 16;
	data[2] = sys_work_para_struct.acc_on_sec_statistic_counter >> 8;
	data[3] = sys_work_para_struct.acc_on_sec_statistic_counter;
	
	return 4;
}
uint8 GetMeterErrTimer(uint8 data[])
{
	data[0] = sys_private_para_struct.meter_comm_breakdown_alarm_sec_timer >> 8;
	data[1] = sys_private_para_struct.meter_comm_breakdown_alarm_sec_timer;
	
	return 2;
}
uint8 GetOverSpeed(uint8 data[])
{
	data[0] = sys_private_para_struct.over_speed_alarm[0];
	data[1] = sys_private_para_struct.over_speed_alarm[1];
	
	return 2;
}
uint8 GetLowVoltageAlarm(uint8 data[])
{
	data[0] = sys_private_para_struct.low_voltage_alarm[0];
	data[1] = sys_private_para_struct.low_voltage_alarm[1];
	
	return 2;
}
uint8 GetLsnalLockEnableFlag(uint8 data[])
{
	data[0] = sys_private_para_struct.lsnal_lock_enable_flag;
	return 1;
}
uint8 GetWorkPara(uint8 data[])
{
	data[0] = sys_private_para_struct.acc_on_up_work_para_sec_timer >> 8;
	data[1] = sys_private_para_struct.acc_on_up_work_para_sec_timer;
	data[2] = sys_private_para_struct.acc_off_up_work_para_sec_timer >> 8;
	data[3] = sys_private_para_struct.acc_off_up_work_para_sec_timer;
	
	return 4;
}
uint8 GetLsnalLockPara(uint8 data[])
{
	data[0] = sys_private_para_struct.lsnal_min_timer >> 8;
	data[1] = sys_private_para_struct.lsnal_min_timer;
	
	return 2;
}

uint8 SetSmsAlarmCenterNum(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint8 tmp_data[256],tmp_len;
	
	if((len > LEN_16 -1)||(len == 0))
	{
		goto RETURN_LAB;
	}

	if(IsValidNum(data,len))
	{
		MemCpy(tmp_data,data,len);
		tmp_data[len] = '\0';
		tmp_len = len + 1;
		
		MemCpy(sys_private_para_struct.sms_alarm_center_num,tmp_data,tmp_len); 
		res = SysPrivateParaWrite();
	}
RETURN_LAB:		
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}

uint8 SetApn(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint8 tmp_data[256],tmp_len;
	
	if(len > LEN_32 -1) 
	{
		goto RETURN_LAB;
	}
	
	if(IsValidChar(data,len))
	{
		MemCpy(tmp_data,data,len);
		tmp_data[len] = '\0';
		tmp_len = len + 1;
		
		MemCpy(sys_private_para_struct.apn,tmp_data,tmp_len);
		res = SysPrivateParaWrite();
	}
RETURN_LAB:		
	if(res)
	{
		sys_misc_run_struct.para_change_re_login_flag = TRUE;
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetMasterIp(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint8 tmp_data[256],tmp_len;
	
	if(len != 4)
	{
		goto RETURN_LAB;
	}
	
	tmp_data[0] = 'I';
	MemCpy(tmp_data+1,data,4);
	tmp_len = len+1;
	
	MemCpy(sys_private_para_struct.master_ip_dns,tmp_data,tmp_len);
	res = SysPrivateParaWrite();
RETURN_LAB:		
	if(res)
	{
		sys_misc_run_struct.para_change_re_login_flag = TRUE;
		pro_struct.link_center_ip_index = MASTER_IP_INDEX;
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetSlaverIp(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint8 tmp_data[256],tmp_len;
	
	if(len != 4) 
	{
		goto RETURN_LAB;
	}
	
	MemCpy(tmp_data+1,data,len);
	tmp_data[0] = 'I';
	tmp_len = len + 1;
	
	MemCpy(sys_private_para_struct.slaver_ip_dns,tmp_data,tmp_len);
	res = SysPrivateParaWrite();
RETURN_LAB:	
	if(res)
	{
		sys_misc_run_struct.para_change_re_login_flag = TRUE;
		pro_struct.link_center_ip_index = SLAVER_IP_INDEX;
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetPort(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	
	if(len != 2) 
	{
		goto RETURN_LAB;
	}
	
	sys_private_para_struct.port[0] = data[0];
	sys_private_para_struct.port[1] = data[1];
	res = SysPrivateParaWrite();
RETURN_LAB:	
	if(res)
	{	
		sys_misc_run_struct.para_change_re_login_flag = TRUE;
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetTerminalId(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint8 sim_num[11];
	
	if(TERMINAL_ID_LEN != len) 
	{
		goto RETURN_LAB;
	}
	
	SimTermIdToAsciiNum(data,sim_num);
	
	res = SimSetSimNum(sim_num);
	if(res)
	{
		MemCpy(sys_private_para_struct.terminal_id,data,len);
		res = SysPrivateParaWrite();
	}
	
RETURN_LAB:	
	if(res)
	{
		sys_misc_run_struct.para_change_re_login_flag = TRUE;
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetTerminalPassword(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint32 tmp_val_32;
	
	if(len != 4) 
	{
		goto RETURN_LAB;
	}
	
	tmp_val_32 = data[0];
	tmp_val_32 = (tmp_val_32 << 8) + data[1];
	tmp_val_32 = (tmp_val_32 << 8) + data[2];
	tmp_val_32 = (tmp_val_32 << 8) + data[3];
	
	sys_private_para_struct.terminal_password = tmp_val_32;
	res = SysPrivateParaWrite();
RETURN_LAB:	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 ParaRecover(uint8 data[],uint8 len)
{
	uint8 res,ret = FAILURE_ACK;

	sys_boot_para_struct.sys_para_init_flag = VALID_VAL_DWORD_AA;
	res = SysBootParaWrite();
	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetTerminalSerialNum(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint8 tmp_data[256],tmp_len;
	
	if(len > LEN_20 -1) 
	{
		goto RETURN_LAB;
	}
	
	if(!IsValidNumOrChar(data,len))
	{
		goto RETURN_LAB;
	}
	
	tmp_data[0] = 'T';
	tmp_data[1] = 'Z';
	MemCpy(tmp_data+2,data,len);
	tmp_data[len+2] = '\0';
	tmp_len = len + 3;

	MemCpy(sys_serial_struct.terminal_serial_num,tmp_data,tmp_len);

	res = SysSerialWrite();

RETURN_LAB:		
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetHardwareVersion(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint8 tmp_data[256],tmp_len;
	
	if(len != 2) 
	{
		goto RETURN_LAB;
	}
	
	if(!IsValidNum(data,len))
	{
		goto RETURN_LAB;
	}
	
	tmp_data[0] = 'V';
	tmp_data[1] = data[0];
	tmp_data[2] = '.';
	tmp_data[3] = data[1];
	tmp_data[4] = '\0';
	tmp_len = 5;
	
	MemCpy(sys_serial_struct.hardware_version,tmp_data,tmp_len);
	res = SysSerialWrite();

RETURN_LAB:		
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetSleepTimer(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint16 tmp_val_16;
	
	if(len != 2) 
	{
		goto RETURN_LAB;
	}
	
	tmp_val_16 = data[0] << 8;
	tmp_val_16 += data[1];
	
	if(tmp_val_16 < 600)
	{
		goto RETURN_LAB;
	}
	
	sys_private_para_struct.acc_off_sleep_sec_timer = tmp_val_16;
	res = SysPrivateParaWrite();
RETURN_LAB:	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetAccStatistic(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint32 tmp_val_32;
	
	if(len != 4) 
	{
		goto RETURN_LAB;
	}
	
	tmp_val_32 = data[0];
	tmp_val_32 = (tmp_val_32 << 8) + data[1];
	tmp_val_32 = (tmp_val_32 << 8) + data[2];
	tmp_val_32 = (tmp_val_32 << 8) + data[3];
	
	res = SpiFramWrite(SYS_WORK_PARA_STRUCT_START_ADDR,(uint8*)&tmp_val_32,4);
	if(res)
	{
		sys_work_para_struct.acc_on_sec_statistic_counter = tmp_val_32;
	}
	
RETURN_LAB:	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetMeterErrTimer(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	uint16 tmp_val_16;
	
	if(len != 2) 
	{
		goto RETURN_LAB;
	}
	
	tmp_val_16 = data[0] << 8;
	tmp_val_16 += data[1];

	sys_private_para_struct.meter_comm_breakdown_alarm_sec_timer = tmp_val_16;
	res = SysPrivateParaWrite();
RETURN_LAB:	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetOverSpeed(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	
	if((len != 1) &&(len != 2))
	{
		goto RETURN_LAB;
	}
	
	MemCpy(sys_private_para_struct.over_speed_alarm,data,len);
	res = SysPrivateParaWrite();
RETURN_LAB:	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetLowVoltageAlarm(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	
	if(len != 2)
	{
		goto RETURN_LAB;
	}
	
	MemCpy(sys_private_para_struct.low_voltage_alarm,data,len);
	res = SysPrivateParaWrite();
	
RETURN_LAB:	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	return ret;
}
uint8 SetLsnalLockEnableFlag(uint8 data[],uint8 len)
{
	uint8 res = FALSE,ret = FAILURE_ACK;
	
	sys_private_para_struct.lsnal_lock_enable_flag = data[0];

	res = SysPrivateParaWrite();
	
	if(res)
	{
		ret = SUCCESS_ACK;
	}
	
	return ret;
}
uint8 SetWorkPara(uint8 data[],uint8 len)
{
	uint8 wr_res,res = PRO_EXE_FAILURE;
	uint16 tmp_val_1,tmp_val_2;
	
	tmp_val_1 = data[0] << 8;
	tmp_val_1 += data[1];
	tmp_val_2 = data[2] << 8;
	tmp_val_2 += data[3];
	if(((tmp_val_1 != 0)&&(tmp_val_1 < 5))||///ACC开最小间隔为5秒
	   ((tmp_val_2 != 0)&&(tmp_val_2  < 1800)))///ACC关最小间隔为1800秒
	{
		goto RETURN_LAB;
	}
	
	sys_private_para_struct.acc_on_up_work_para_sec_timer = tmp_val_1;
	sys_private_para_struct.acc_off_up_work_para_sec_timer = tmp_val_2;
	
	wr_res = SysPrivateParaWrite();
	if(wr_res)
	{
		res = PRO_EXE_SUCCESS;
	}
RETURN_LAB:
	return res;
}

uint8 SetLsnalLockPara(uint8 data[],uint8 len)
{
	uint8 res = PRO_EXE_FAILURE;
	uint8 wr_res;
	uint16 tmp_val;
	
	if(len != 2)
	{
		goto RETURN_LAB;
	}
	tmp_val = (data[0] << 8)+data[1];
	
	sys_private_para_struct.lsnal_min_timer = tmp_val;
	wr_res = SysPrivateParaWrite();
	if(wr_res)
	{
		res = PRO_EXE_SUCCESS;
	}
RETURN_LAB:
	return res;
}

uint16 ParaCorrespond(uint16 cmd)
{
	uint8 i,array_size;
	uint16 re_cmd = 0xffff;
	
	array_size = sizeof(serial_comm_struct)/sizeof(serial_comm_struct[0]);
	for(i=0;i<array_size;i++)
	{
        if(cmd == serial_comm_struct[i].serial_id)
		{
			re_cmd = serial_comm_struct[i].pro_id;
			break;
		}
	}
	
	return re_cmd;
}
uint16 QuerySysPrivatePara(uint8 dst[],uint8 src[],uint16 len,uint8 is_local_flag)
{
	uint8 *p;
	uint16 cmd,i;
	
	len /= 2;
	p = dst;
	
	if(len > 64)///最大支持64个参数ID
	{
		goto RETURN_LAB;
	}

	for(i=0;i<len;i++)
	{
	
		cmd = src[i*2];	
		cmd = (cmd << 8)+src[i*2+1];
		
		p[0] = FAILURE_ACK;
		p[1] = src[i*2];
		p[2] = src[i*2+1];
		p[3] = 0x00;
		if(is_local_flag)
		{
			cmd = ParaCorrespond(cmd);
		}
		switch(cmd)
		{
			case SYS_PARA_SMS_ALARM_CENTER_NUM_ID:
			{
				p[3] = GetSmsAlarmCenterNum(p+4);
				break;
			}
			case SYS_PARA_APN_ID:
			{
				p[3] = GetApn(p+4);
				break;
			}
			case SYS_PARA_MASTER_IP_ID:
			{
				p[3] = GetMasterIp(p+4);
				break;
			}
			case SYS_PARA_SLAVER_IP_ID:
			{
				p[3] = GetSlaverIp(p+4);
				break;
			}
			case SYS_PARA_PORT_ID:
			{
				p[3] = GetPort(p+4);
				break;
			}
			case SYS_PARA_TERMINAL_ID:
			{
				p[3] = GetTerminalId(p+4);
				break;
			}
			case SYS_PARA_SOFTWARE_VERSION_ID:
			{
				p[3] = GetSoftWareVersion(p+4);
				break;
			}
			case SYS_PARA_TERMINAL_SERIAL_ID:
			{
				p[3] = GetTerminalSerialNum(p+4);
				break;
			}
			case SYS_PARA_HARDWARE_VERSION_ID:
			{
				p[3] = GetHardwareVersion(p+4);
				break;
			}
			case SYS_PARA_SLEEP_TIMER_ID:
			{
				p[3] = GetSleepTimer(p+4);
				break;
			}
			case SYS_PARA_ACC_STATISTIC_ID:
			{
				p[3] = GetAccAtatistic(p+4);
				break;
			}
			case SYS_PARA_METER_ERR_TIMER_ID:
			{
				p[3] = GetMeterErrTimer(p+4);
				break;
			}
			case SYS_PARA_OVER_SPEED_ID:
			{
				p[3] = GetOverSpeed(p+4);
				break;
			}
			case SYS_PARA_LOW_VOLTAGE_ALARM_ID:
			{
				p[3] = GetLowVoltageAlarm(p+4);
				break;
			}
			case SYS_PARA_LSNAL_LOCK_ENABLE_ID:
			{
				p[3] = GetLsnalLockEnableFlag(p+4);
				break;
			}
			case SYS_PARA_PASSWORD_ID:
			{
				p[3] = GetTerminalPassword(p+4);
				break;
			}
			case SYS_PARA_WORK_PARA_ID:
			{
				p[3] = GetWorkPara(p+4);
				break;
			}
			case SYS_PARA_LSNAL_LOCK_MIN_ID:
			{
				p[3] = GetLsnalLockPara(p+4);
				break;
			}
			case SYS_PARA_DEBUG_ENABLE_ID:
			{
				p[4] = sys_misc_run_struct.local_debug_enalbe_flag;
				p[3] = 1;
				break;
			}
			default:
			{
				break;
			}
		}
		
		if(p[3] > 0)
		{
			p[0] = SUCCESS_ACK;
		}		
		p += (4+p[3]);
	}
	
RETURN_LAB:
	return(p-dst);
	
}
uint8 SetSysPrivatePara(uint8 data[],uint16 len,uint8 is_local_flag)
{
	uint8 res = FAILURE_ACK;
	uint16 cmd;
	
	if(len < 3)
	{
		goto RETURN_LAB;
	}

	len -= 2;
	cmd = data[0];
	cmd = (cmd << 8)+data[1];
	if(is_local_flag)
	{
		cmd = ParaCorrespond(cmd);
	}
	switch(cmd)
	{
		case SYS_PARA_SMS_ALARM_CENTER_NUM_ID:///报警短信中心
		{
			res = SetSmsAlarmCenterNum(data+3,len-1);
			break;
		}
		case SYS_PARA_APN_ID:///APN
		{
			res = SetApn(data+3,len-1);
			break;
		}
		case SYS_PARA_MASTER_IP_ID:///主中心IP
		{
			res = SetMasterIp(data+3,len-1);
			break;
		}
		case SYS_PARA_SLAVER_IP_ID:///副中心IP
		{
			res = SetSlaverIp(data+3,len-1);
			break;
		}
		case SYS_PARA_PORT_ID:///Port
		{
			res = SetPort(data+3,len-1);
			break;
		}
		case SYS_PARA_TERMINAL_ID:///SIMCARD ID
		{	
			res = SetTerminalId(data+3,len-1);
			break;
		}
		case SYS_PARA_PASSWORD_ID:///终端密码 ID
		{	
			res = SetTerminalPassword(data+3,len-1);
			break;
		}	
		case SYS_PARA_RECOVER_ID:///恢复出厂设置
		{
			res = ParaRecover(data+3,len-1);
			break;
		}
		case SYS_PARA_TERMINAL_SERIAL_ID:///终端序列号
		{
			res = SetTerminalSerialNum(data+3,len-1);
			break;
		}
		case SYS_PARA_HARDWARE_VERSION_ID:///硬件版本号
		{
			res = SetHardwareVersion(data+3,len-1);
			break;
		}
		case SYS_PARA_SLEEP_TIMER_ID:///休眠
		{
			res = SetSleepTimer(data+3,len-1);
			break;
		}
		case SYS_PARA_ACC_STATISTIC_ID:///ACC开统计时间
		{
			res = SetAccStatistic(data+3,len-1);
			break;
		}
		case SYS_PARA_METER_ERR_TIMER_ID:///仪表故障定时器
		{
			res = SetMeterErrTimer(data+3,len-1);
			break;
		}
		case SYS_PARA_OVER_SPEED_ID:///超速
		{
			res = SetOverSpeed(data+3,len-1);
			break;
		}
		case SYS_PARA_LOW_VOLTAGE_ALARM_ID:///低压
		{
			res = SetLowVoltageAlarm(data+3,len-1);
			break;
		}
		case SYS_PARA_LSNAL_LOCK_ENABLE_ID:
		{
			res = SetLsnalLockEnableFlag(data+3,len-1);
			break;
		}
		case SYS_PARA_WORK_PARA_ID:
		{
			res = SetWorkPara(data+3,len-1);
			break;
		}
		case SYS_PARA_LSNAL_LOCK_MIN_ID:
		{
			res = SetLsnalLockPara(data+3,len-1);
			break;
		}
		case SYS_PARA_DEBUG_ENABLE_ID:
		{
			sys_misc_run_struct.local_debug_enalbe_flag = data[3];
			res = SUCCESS_ACK;
			break;
		}
		case SYS_PARA_RESET_ID:
		{
			sys_misc_run_struct.term_reset_falg = VALID_VAL_AA;
			res = SUCCESS_ACK;
			break;
		}
		default:
		{
			break;
		}
	}	
	
RETURN_LAB:
	return res;
}

