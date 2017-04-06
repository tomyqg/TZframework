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
	
	
	#define TERMINAL_ID_LEN				5			///�ն�ID����
	#define SIM_CARD_IMSI_LEN			15			///SIM��IMSI����
	#define GSM_IMEI_LEN				15
	#define ACC_ON_OFF_DAY_STATISTIC_COUNTER 30		///1��30�Կ��ػ���¼
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
	

	#define LSNAL_PAGE_SIZE 		0x200///ä������512�ֽ�/ҳ
	typedef struct 
	{
		char sms_city_center_num[LEN_32];///SIM�����ڳ��ж�������
		char second_sms_center_num[LEN_32];///�����ڲ���������2
		char third_sms_cetner_num[LEN_32];///��˾Ĭ�϶�������3
		char software_version[LEN_32];///TIZA_��ͷ��������ѹ24V���汾��1.0,������
	}SYS_CONST_PARA_STRUCT;///ϵͳ��������

	typedef struct
	{
		uint32 program_update_flag;///������±�־
		uint32 program_total_size;///�ֽ�����
		uint32 sys_para_init_flag;///�������³�ʼ����־
	}SYS_BOOT_PARA_STRUCT;///ϵͳ��������
	
	typedef struct
	{
		uint8 terminal_serial_num[LEN_20];///�ն����к�
		uint8 hardware_version[LEN_12];///�ն�Ӳ���汾��
	}SYS_SERIAL_STRUCT;///ϵͳ���кŲ���
	typedef struct
	{
		uint8 sms_alarm_center_num[LEN_16];///���ű������ĺ���
		uint8 master_ip_dns[LEN_32];///������IP��DNS
		uint8 slaver_ip_dns[LEN_32];///������IP��DNS
		uint8 port[2];///�˿ں�
		uint8 apn[LEN_32];///APN
		uint8 gprs_login_user_name[LEN_32];///GPRS��¼�û���
		uint8 gprs_login_password[LEN_32];///GPRS��¼����
		uint8 terminal_id[TERMINAL_ID_LEN];///����Э�����ն�ID,��SIM�������Ż��
		uint8 sim_card_imsi[SIM_CARD_IMSI_LEN];///SIM��IMSI
		uint8 gsm_imei[GSM_IMEI_LEN];///GSMģ��IMEI
		uint8 up_heart_beat_sec_timer;///����������ʱ��
		uint8 over_speed_alarm[2];///���ٱ���ֵ,���ٱ����붨ʱ��
		uint8 low_voltage_alarm[2];///��ѹ����ֵ,��ѹ�붨ʱ��
		uint16 meter_comm_breakdown_alarm_sec_timer;///�Ǳ���ϱ���ʱ��
		uint16 acc_off_sleep_sec_timer;///ACC_ON�󣬽��������붨ʱ��
		uint16 acc_on_up_work_para_sec_timer;///ACC_ONʱ���ϴ�����������ʱ��
		uint16 acc_off_up_work_para_sec_timer;///ACC_OFFʱ���ϴ�����������ʱ��
		uint8  lsnal_lock_enable_flag;///ä����������ʹ�ܱ�־
		uint16 lsnal_min_timer;///ä�����Ӷ�ʱ��
		uint32 terminal_password;///�ն�����
		uint32 check_val;///�ն˲���У��
	}SYS_PRIVATE_PARA_STRUCT;///ϵͳ˽�в���

	typedef struct
	{
		uint32 acc_on_sec_statistic_counter;///ACC_ON��ͳ�Ƽ�����(�Իָ�����ֵ)
		uint32 acc_on_sec_counter;///ACC_ON�������(����)
		uint32 acc_off_sec_counter;///ACC_OFF�������(����)
		uint32 term_run_status_word;///�ն�����״̬��
		uint16 lsnal_head_page;///Ƭ��FLASH�У�ä������FIFOͷҳλ��
		uint16 lsnal_tail_page;
		uint16 lsnal_min_counter;///ä�����Ӽ���,����ʱ��������
		uint16 lsnal_sys_reset_min_counter;///ä��ϵͳ��λ�����Ӽ���
		uint16 lsnal_sys_must_reset_sec_counter;///ä��ϵͳ�жϸ�λ
		uint16 lsnal_sys_comm_reset_min_counter;///ä��ϵͳͨ�Ų�����λ�����Ӽ���
		uint16 track_of_acc_on_sec_timer;///ACC_ONʱ��׷���붨ʱ��
		uint16 track_of_acc_on_sec_counter;///ACC_ONʱ��׷���붨ʱ��
		uint8 date_time[6];///ϵͳʱ��
		uint8 sms_lock_car_flag;///Զ������������־
		uint8 term_3_clock_reset_flag;///�ն�3�㸴λ��־
		uint8 term_self_reset_flag;///�ն��Ը�λ��־
		uint8 no_simcard_falg;///��SIM����־
		uint16 gps_ant_off_sec_counter;///gps���߼��߼�ʱ
		uint16 can_stop_heart_min_counter;///CAN����ͣ���������Ӽ�ʱ��
		
	}SYS_WORK_PARA_STRUCT;///ϵͳ��������
	
	typedef struct
	{
		uint8 gps_info_of_gprs[LEN_64];///����Э�����ݰ��У����ӵ�GPS����
		///ACC���ػ���¼��
		///2A(��Ч��־)+�ܳ���(�ߵ�2�ֽ�)+������+������+(ʱ����(��)+ʱ����(��))*������+��У��1�ֽ�
		uint8 acc_on_off_statistic[8+ACC_ON_OFF_DAY_STATISTIC_COUNTER*6];
		///2A(��Ч��־),����(�ߵ�2�ֽ�,��������ʼ��У��֮ǰ)������������1��ID,��1������(�ߵ�2�ֽ�)����1������...����У��(1�ֽ�)
		uint8 lsnal_data[LSNAL_PAGE_SIZE];
	}SYS_DATA_STRUCT;
	
	typedef struct
	{
		uint8 flag_2a;///�洢��Ч��־
		uint8 lock_flag;///�Ƿ�����
		uint8 date_time[6];
		uint8 lock_method;///������ʽ
		uint8 lock_level;///�����ȼ�
		uint8 lock_reason;///����ԭ��
	}LOCK_RECORD_STRUCT;
	
	typedef struct
	{
		uint16 sys_tick_ms_counter;///ϵͳ���������
		uint16 sys_rtc_sec_counter;///ϵͳ�������
		uint16 adc_conv_buf[4];///ADת��ֵ
		uint16 power_voltage_val;///������أ��ն˵�Դ��ѹֵ����λ0.1V
		uint16 battery_voltage_val;///�ն˵�ص�ѹֵ����λ0.1V
		uint8 gsm_csq_val;///GSM�ź�ǿ��
		uint8 up_heart_beat_sec_counter;///���������������
		uint16 gsm_csq_check_sec_counter;///GSM CSQ������
		uint16 tx_lat_long_sec_counter;///���;�γ�������
		uint8 gsm_csq_check_flag;///GSM CSQ����־
		uint8 term_enter_sleep_flag;///�ն���Ҫ�������߱�־
		uint8 gps_need_datetime_check_flag;///GPSʱ��У��
		uint8 again_into_sleep_flag;///���߻���,�ٴν������߱�־
		uint8 wake_up_by_exit_flag;///����ͨ���ⲿ�жϻ��ѱ�־
		uint8 local_debug_enalbe_flag;///���ص���ʹ�ܱ�־
		uint8 para_change_re_login_flag;///�����������µ�¼��־
		uint8 tmp_sec_counter;
		uint8 gprs_delay_flag;///GPRS��ʱ��־
		uint8 systask_delay_flag;///ϵͳ������ʱ��־
		uint8 localcomm_delay_flag;///��־������ʱ��־
		uint8 link_center_ip[4];///��������IP
		uint8 sys_cold_start_falg;///��������־
		uint32 wdg_counter;
		uint8 term_reset_falg;///�ն˸�λ��־
	}SYS_MISC_RUN_STRUCT;///ϵͳ���в���

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
