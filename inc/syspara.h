#ifndef __SYS_PARA_H
#define __SYS_PARA_H

	#ifdef SYS_PARA_STRUCT_GLOBAL
		#define EXTERN_SYS_PARA
	#else
		#define EXTERN_SYS_PARA extern
	#endif
	
	#define 	IRQ_DISABLE()     	(__disable_irq())
	#define  	IRQ_ENABLE()  		(__enable_irq())
	
	
	#define SYS_SERIAL_STRUCT_ADDR   140
	#define SYS_PRIVATE_PARA_STRUCT_START_ADDR  (SYS_SERIAL_STRUCT_ADDR+sizeof(SYS_SERIAL_STRUCT))
	#define SYS_WORK_PARA_STRUCT_START_ADDR  	(SYS_PRIVATE_PARA_STRUCT_START_ADDR+sizeof(SYS_PRIVATE_PARA_STRUCT))
	#define SYS_DATA_STRUCT_START_ADDR  		(SYS_WORK_PARA_STRUCT_START_ADDR+sizeof(SYS_WORK_PARA_STRUCT))
	#define LOCK_RECORD_STRUCT_ADDR				(SYS_DATA_STRUCT_START_ADDR+sizeof(SYS_DATA_STRUCT))
	#define METER_DATA_START_ADDR				(LOCK_RECORD_STRUCT_ADDR+sizeof(LOCK_RECORD_STRUCT))
	
	
	#define TERMINAL_ID_LEN				5			///终端ID长度
	#define SIM_CARD_IMSI_LEN			15			///SIM卡IMSI长度
	#define GSM_IMEI_LEN				15
	#define ACC_ON_OFF_DAY_STATISTIC_COUNTER 30		///1天30对开关机记录
	#define ACC_ON_OFF_DAY_STATISTIC_LEN (ACC_ON_OFF_DAY_STATISTIC_COUNTER*6 + 8)
	
	#define VALID_VAL_DWORD_AA		0xAAAAAAAA
	#define INVALID_VAL_DWORD_55	0x55555555
	#define VALID_VAL_AA			0xAA
	#define VALID_VAL_BB			0xBB
	#define INVALID_VAL_55			0x55	
	#define VALID_VAL_2A			0x2A
	#define INVALID_VAL_FF			0xFF
	
	#define FAILURE_ACK 0X01
	#define SUCCESS_ACK 0X00
	

	#define LSNAL_PAGE_SIZE 		0x200///盲区数据512字节/页
	typedef struct 
	{
		char sms_city_center_num[LEN_32];///SIM卡所在城市短信中心
		char second_sms_center_num[LEN_32];///部门内部短信中心2
		char third_sms_cetner_num[LEN_32];///公司默认短信中心3
		char software_version[LEN_32];///TIZA_开头，车机电压24V，版本号1.0,年月日
	}SYS_CONST_PARA_STRUCT;///系统启动参数

	typedef struct
	{
		uint32 program_update_flag;///程序更新标志
		uint32 program_total_size;///字节总数
		uint32 sys_para_init_flag;///参数重新初始化标志
	}SYS_BOOT_PARA_STRUCT;///系统启动参数
	
	typedef struct
	{
		uint8 terminal_serial_num[LEN_20];///终端序列号
		uint8 hardware_version[LEN_12];///终端硬件版本号
	}SYS_SERIAL_STRUCT;///系统序列号参数
	typedef struct
	{
		uint8 sms_alarm_center_num[LEN_16];///短信报警中心号码
		uint8 master_ip_dns[LEN_32];///主中心IP或DNS
		uint8 slaver_ip_dns[LEN_32];///副中心IP或DNS
		uint8 port[2];///端口号
		uint8 apn[LEN_32];///APN
		uint8 gprs_login_user_name[LEN_32];///GPRS登录用户名
		uint8 gprs_login_password[LEN_32];///GPRS登录密码
		uint8 terminal_id[TERMINAL_ID_LEN];///上行协议中终端ID,读SIM卡本机号获得
		uint8 sim_card_imsi[SIM_CARD_IMSI_LEN];///SIM卡IMSI
		uint8 gsm_imei[GSM_IMEI_LEN];///GSM模块IMEI
		uint8 up_heart_beat_sec_timer;///上行心跳定时器
		uint8 over_speed_alarm[2];///超速报警值,超速报警秒定时器
		uint8 low_voltage_alarm[2];///低压报警值,低压秒定时器
		uint16 meter_comm_breakdown_alarm_sec_timer;///仪表故障报警时间
		uint16 acc_off_sleep_sec_timer;///ACC_ON后，进入休眠秒定时器
		uint16 acc_on_up_work_para_sec_timer;///ACC_ON时，上传工作参数定时器
		uint16 acc_off_up_work_para_sec_timer;///ACC_OFF时，上传工作参数定时器
		uint8  lsnal_lock_enable_flag;///盲区被动锁车使能标志
		uint16 lsnal_min_timer;///盲区分钟定时器
		uint32 terminal_password;///终端密码
		uint32 check_val;///终端参数校验
	}SYS_PRIVATE_PARA_STRUCT;///系统私有参数

	typedef struct
	{
		uint32 acc_on_sec_statistic_counter;///ACC_ON秒统计计数器(自恢复初厂值)
		uint32 acc_on_sec_counter;///ACC_ON秒计数器(单次)
		uint32 acc_off_sec_counter;///ACC_OFF秒计数器(单次)
		uint32 term_run_status_word;///终端运行状态字
		uint16 lsnal_head_page;///片内FLASH中，盲区数据FIFO头页位置
		uint16 lsnal_tail_page;
		uint16 lsnal_min_counter;///盲区分钟计数,超出时间则锁车
		uint16 lsnal_sys_reset_min_counter;///盲区系统复位，分钟计数
		uint16 lsnal_sys_must_reset_sec_counter;///盲区系统中断复位
		uint16 lsnal_sys_comm_reset_min_counter;///盲区系统通信参数复位，分钟计数
		uint16 track_of_acc_on_sec_timer;///ACC_ON时，追踪秒定时器
		uint16 track_of_acc_on_sec_counter;///ACC_ON时，追踪秒定时器
		uint8 date_time[6];///系统时钟
		uint8 sms_lock_car_flag;///远方短信锁车标志
		uint8 term_3_clock_reset_flag;///终端3点复位标志
		uint8 term_self_reset_flag;///终端自复位标志
		uint8 no_simcard_falg;///无SIM卡标志
		uint16 gps_ant_off_sec_counter;///gps天线剪线计时
		uint16 can_stop_heart_min_counter;///CAN总线停发心跳分钟计时器
		
	}SYS_WORK_PARA_STRUCT;///系统工作参数
	
	typedef struct
	{
		uint8 gps_info_of_gprs[LEN_64];///上行协议数据包中，附加的GPS数据
		///ACC开关机记录，
		///2A(有效标志)+总长度(高低2字节)+年月日+总条数+(时分秒(开)+时分秒(关))*总条数+和校验1字节
		uint8 acc_on_off_statistic[8+ACC_ON_OFF_DAY_STATISTIC_COUNTER*6];
		///2A(有效标志),长度(高低2字节,总条数开始，校验之前)，总条数，第1包ID,第1包长度(高低2字节)，第1包内容...，和校验(1字节)
		uint8 lsnal_data[LSNAL_PAGE_SIZE];
	}SYS_DATA_STRUCT;
	
	typedef struct
	{
		uint8 flag_2a;///存储有效标志
		uint8 lock_flag;///是否锁车
		uint8 date_time[6];
		uint8 lock_method;///锁车方式
		uint8 lock_level;///锁车等级
		uint8 lock_reason;///锁车原因
	}LOCK_RECORD_STRUCT;
	
	typedef struct
	{
		uint16 sys_tick_ms_counter;///系统毫秒计数器
		uint16 sys_rtc_sec_counter;///系统秒计数器
		uint16 adc_conv_buf[4];///AD转换值
		uint16 power_voltage_val;///车机电池，终端电源电压值，单位0.1V
		uint16 battery_voltage_val;///终端电池电压值，单位0.1V
		uint8 gsm_csq_val;///GSM信号强度
		uint8 up_heart_beat_sec_counter;///上行心跳秒计数器
		uint16 gsm_csq_check_sec_counter;///GSM CSQ检测计数
		uint16 tx_lat_long_sec_counter;///发送经纬度秒计数
		uint8 gsm_csq_check_flag;///GSM CSQ检测标志
		uint8 term_enter_sleep_flag;///终端需要进入休眠标志
		uint8 gps_need_datetime_check_flag;///GPS时钟校对
		uint8 again_into_sleep_flag;///休眠唤醒,再次进入休眠标志
		uint8 wake_up_by_exit_flag;///休眠通过外部中断唤醒标志
		uint8 local_debug_enalbe_flag;///本地调试使能标志
		uint8 para_change_re_login_flag;///参数更改重新登录标志
		uint8 tmp_sec_counter;
		uint8 gprs_delay_flag;///GPRS延时标志
		uint8 systask_delay_flag;///系统任务延时标志
		uint8 localcomm_delay_flag;///本志串口延时标志
		uint8 link_center_ip[4];///链接中心IP
		uint8 sys_cold_start_falg;///冷启动标志
		uint32 wdg_counter;
		uint8 term_reset_falg;///终端复位标志
	}SYS_MISC_RUN_STRUCT;///系统运行参数

	EXTERN_SYS_PARA SYS_SERIAL_STRUCT			sys_serial_struct;
	EXTERN_SYS_PARA SYS_BOOT_PARA_STRUCT 		sys_boot_para_struct;
	EXTERN_SYS_PARA SYS_PRIVATE_PARA_STRUCT 	sys_private_para_struct;
	EXTERN_SYS_PARA SYS_WORK_PARA_STRUCT 		sys_work_para_struct;
	EXTERN_SYS_PARA SYS_MISC_RUN_STRUCT 		sys_misc_run_struct;
	EXTERN_SYS_PARA SYS_DATA_STRUCT 			sys_data_struct;
	EXTERN_SYS_PARA const SYS_CONST_PARA_STRUCT	sys_const_para_struct;
	EXTERN_SYS_PARA LOCK_RECORD_STRUCT			lock_record_struct;
	/**
	EXTERN_SYS_PARA uint8 const g_terminal_soft_ware_version[];
	EXTERN_SYS_PARA uint8 const g_second_sms_center_num[];
	EXTERN_SYS_PARA uint8 const g_third_sms_cetner_num[];
	**/
	void SysCommParaInit(void);
	uint8 SysSerialRead(void);
	uint8 SysSerialWrite(void);
	void SysPrivateParaInit(void);
	void SysWorkParaInit(void);
	void SysBootParaRead(void);
	uint8 SysBootParaWrite(void);
	uint8 SysPrivateParaRead(void);
	uint8 SysPrivateParaWrite(void);
	uint8 SysWorkParaRead(void);
	uint8 SysWorkParaWrite(void);
	void SysReset(void);
	void TermReset(void);
	void TermReset2(void);
	void TermRingReset(void);
	void SysKeyDataSave(void);
	void SysVaryInit(void);
	void SysParaRead(void);
	uint16 ParaCorrespond(uint16 cmd);
	void LockRecord(uint8 lock_method,uint8 level,uint8 reason);
	void UnLockRecord(void);
	uint16 QuerySysPrivatePara(uint8 dst[],uint8 src[],uint16 len,uint8 is_local_flag);
	uint8 SetSysPrivatePara(uint8 data[],uint16 len,uint8 is_local_flag);
	
	uint8 GetSmsAlarmCenterNum(uint8 data[]);
	uint8 GetApn(uint8 data[]);
	uint8 GetMasterIp(uint8 data[]);
	uint8 GetSlaverIp(uint8 data[]);
	uint8 GetPort(uint8 data[]);
	uint8 GetTerminalId(uint8 data[]);
	uint8 GetTerminalPassword(uint8 data[]);
	uint8 GetTerminalSerialNum(uint8 data[]);
	uint8 GetHardwareVersion(uint8 data[]);
	uint8 GetSleepTimer(uint8 data[]);
	uint8 GetAccAtatistic(uint8 data[]);
	uint8 GetMeterErrTimer(uint8 data[]);
	uint8 GetOverSpeed(uint8 data[]);
	uint8 GetLowVoltageAlarm(uint8 data[]);
	uint8 GetBindImsi(uint8 data[]);
	
	uint8 SetSmsAlarmCenterNum(uint8 data[],uint8 len);
	uint8 SetApn(uint8 data[],uint8 len);
	uint8 SetMasterIp(uint8 data[],uint8 len);
	uint8 SetSlaverIp(uint8 data[],uint8 len);
	uint8 SetPort(uint8 data[],uint8 len);
	uint8 SetTerminalId(uint8 data[],uint8 len);
	uint8 SetTerminalPassword(uint8 data[],uint8 len);
	uint8 ParaRecover(uint8 data[],uint8 len);
	uint8 SetTerminalSerialNum(uint8 data[],uint8 len);
	uint8 SetHardwareVersion(uint8 data[],uint8 len);
	uint8 SetSleepTimer(uint8 data[],uint8 len);
	uint8 SetAccStatistic(uint8 data[],uint8 len);
	uint8 SetMeterErrTimer(uint8 data[],uint8 len);
	uint8 SetOverSpeed(uint8 data[],uint8 len);
	uint8 SetLowVoltageAlarm(uint8 data[],uint8 len);
	uint8 SetBindImsi(uint8 data[],uint8 len);
	uint8 SetWorkPara(uint8 data[],uint8 len);

#endif
