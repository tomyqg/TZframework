#ifndef __SYS_TASK_H
#define __SYS_TASK_H

	#ifdef SYS_TASK_STRUCT_GLOBAL
		#define EXTERN_SYS_TASK
	#else
		#define EXTERN_SYS_TASK extern
	#endif
	/***
	#define STATE_A_GPS					(0x01 << 24)
	#define STATE_N_LAT					(0x01 << 25)
	#define STATE_E_LONG				(0x01 << 26)
	
	#define STATE_ACC_ON				(0x01 << 16)///ACC��
	#define STATE_METER_LOCK			(0x01 << 23)///�Ǳ�����
	
	#define STATE_OVER_SPEED			(0x01 << 9)///����
	#define STATE_PWR_DOWN				(0x01 << 10)///�ϵ�
	#define STATE_TRAILER				(0x01 << 11)///�ϳ�
	#define STATE_SHELL_OFF				(0x01 << 12)///�ۿ�
	
	#define STATE_GPS_BREAKDOWN 		(0x01 << 0)///GPSģ�����
	#define STATE_GPS_ANT_OFF			(0x01 << 1)///GPS����δ��
	#define STATE_LOW_VOLTAGE			(0x01 << 2)///������ƿ��ѹ��
	#define STATE_METER_COMM_BREAKDOWN	(0x01 << 7)///�Ǳ�ͨѶ����
	***/	
		
	#define ALRM_FLAG_OVER_SPEED			0X01
	#define ALRM_FLAG_ANT_OFF				0X02
	#define ALRM_FLAG_GPS_BREAKDOWN			0X03
	#define ALRM_FLAG_LOW_VOLTAGE			0X04
	#define ALRM_FLAG_METER_COMM_BREAKDOWN	0X05
	#define ALRM_FLAG_PWR_DOWN				0X06
	#define ALRM_FLAG_TRAILER				0X07
	#define ALRM_FLAG_SHELL_OFF				0X09
	#define ALRM_FLAG_SIMCARD_CHANGE		0X0D
	
	#define ALRM_FLAG_CLEAR_BYTE			0X80
	
	#define STATE_A_GPS					(0x01 << 24)
	#define STATE_N_LAT					(0x01 << 25)
	#define STATE_E_LONG				(0x01 << 26)
	#define STATE_ACC_ON				(0x01 << 27)
	#define STATE_METER_LOCK			(0x01 << 28)///�Ǳ�����
	
	#define STATE_METER_COMM_BREAKDOWN	(0x01 << 16)///�Ǳ�ͨѶ�ж�
	#define STATE_PWR_DOWN				(0x01 << 17)///�����
	#define STATE_LOW_VOLTAGE			(0x01 << 18)///������ƿ��ѹ��
	#define STATE_SIMCARD_CHANGE		(0x01 << 19)///����
	#define STATE_GPS_BREAKDOWN 		(0x01 << 20)///GPS��λģ�����
	#define STATE_GPS_ANT_OFF			(0x01 << 21)///GPS����δ��
	
	#define STATE_OVER_SPEED			(0x01 << 8)///����
	#define STATE_TRAILER				(0x01 << 9)///�ϳ�
	#define STATE_SHELL_OFF				(0x01 << 10)///�ۿ�
	
	#define STATE_MASK_BIT3				(STATE_A_GPS|STATE_N_LAT|STATE_E_LONG|STATE_ACC_ON|STATE_METER_LOCK)
	#define STATE_MASK_BIT2             (STATE_METER_COMM_BREAKDOWN|STATE_PWR_DOWN|STATE_LOW_VOLTAGE|STATE_SIMCARD_CHANGE|STATE_GPS_BREAKDOWN|STATE_GPS_ANT_OFF)
	#define STATE_MASK_BIT1             (STATE_OVER_SPEED|STATE_TRAILER|STATE_SHELL_OFF)
	#define STATE_MASK_BIT0             (0x00)
	
	#define STATE_MASK_BITS				(STATE_MASK_BIT3|STATE_MASK_BIT2|STATE_MASK_BIT1|STATE_MASK_BIT0)
	
	
			
	void SysTaskMain(void);
	void TermEnterSleep(void);
	void DayChangeMonitor(uint8 date_time[]);
	void AccOnOffStatisticDataCheck(uint8 date[]);
	void AccOnOffStatisticDataSave(uint16 len);
	void AccOnOffDataInit(uint8 date[]);
	void AccOnOffDataAppend(uint8 time[],uint8 is_acc_on_falg);
	void SysNoTaskDelay(uint8 delay_sec);
	void SysDelay(uint8 delay_sec);
	
#endif
