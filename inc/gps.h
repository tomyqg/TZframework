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

	#define LAT_INDEX						0			///纬度地址，百分秒，4
	#define LONG_INDEX						4			///经度地址，百分秒，4
	#define SPEED_INDEX						8			///GPS速度1km/h 1
	#define DIR_INDEX						9			///方向1
	#define AMP_INDEX						10			///海拔
	#define STATUS_INDEX					12			///状态4
	#define DATE_INDEX						16			///日期起始地址
	#define TIME_INDEX						19			///时间起始地址
	#define GPS_INFO_LEN					(TIME_INDEX+3)///上行协议GPS信息长度
	#define SAT_INDEX						GPS_INFO_LEN  ///卫星数量
	
	
	#define GPS_BREAK_DOWM_SEC_TIMER		300			///gps模块故障秒定时器
	#define GPS_LAT_LONG_POS_LEN			71			///经纬度信息
	
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
