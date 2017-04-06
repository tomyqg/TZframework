#ifndef __GPS_H
#define __GPS_H
	
	#ifdef GPS_STRUCT_GLOBAL
		#define EXTERN_GPS
	#else
		#define EXTERN_GPS extern
	#endif
	
	#include "stm32f10x.h"
	
	#define A_POS_INDEX 10
	
	#define LAT_TYPE 	0
	#define LONG_TYPE 	1
	#define SPEED_TYPE 	2
	#define DIR_TYPE 	3
	#define AMP_TYPE 	4
	
	#define RMC_TYPE 	1
	#define GGA_TYPE 	2

	#define LAT_INDEX						0			///γ�ȵ�ַ���ٷ��룬4
	#define LONG_INDEX						4			///���ȵ�ַ���ٷ��룬4
	#define SPEED_INDEX						8			///GPS�ٶ�1km/h 1
	#define DIR_INDEX						9			///����1
	#define AMP_INDEX						10			///����
	#define STATUS_INDEX					12			///״̬4
	#define DATE_INDEX						16			///������ʼ��ַ
	#define TIME_INDEX						19			///ʱ����ʼ��ַ
	#define GPS_INFO_LEN					(TIME_INDEX+3)///����Э��GPS��Ϣ����
	#define SAT_INDEX						GPS_INFO_LEN  ///��������
	
	
	#define GPS_BREAK_DOWM_SEC_TIMER		300			///gpsģ������붨ʱ��
	#define GPS_LAT_LONG_POS_LEN			71			///��γ����Ϣ
	
	typedef struct
	{
		uint8 gps_rx_right_data_flag;
		uint8 gps_lat_long_position[GPS_LAT_LONG_POS_LEN];
		uint8 sms_gprmc_ack[256];
	}GPS_STRUCT;
	
	EXTERN_GPS GPS_STRUCT gps_struct;
	void GpsMain(void);
	uint8 GpsFrameCheck(uint8 gps_data[],uint8 len);
	void GpsGetGprmcGpggaInfo(uint8 gps_data[],uint8 len,uint8 type);
	uint16 GetMeterUartRxData(uint8 rx_data[]);
	uint8 GpsCharFloatDataToHex(uint8 gps_data[],uint8 len,uint8 type,uint32 *r_val);
	uint8 GpsGetDate(uint8 utc_date[],uint8 hex_date[]);
	uint8 GpsGetTime(uint8 utc_time[],uint8 hex_time[]);
	uint8 GetGpsInfo(uint8 data[]);

#endif
