#ifndef __CAN_H
#define __CAN_H

	#ifdef CAN_GLOBAL
		#define EXTN_CAN
	#else
		#define EXTN_CAN extern
	#endif

	#define CAN_R_0 0x0CF00300
	#define CAN_R_1 0x0CF00400
	#define CAN_R_2 0x18FEEF00
	#define CAN_R_3 0x18FEE500
	#define CAN_R_4 0x18FEEE00
	#define CAN_R_5 0x18FEF600
	#define CAN_R_6 0x18FEE900
	#define CAN_R_7 0x18FECA00
	#define CAN_R_8 0x18FEF200
	#define CAN_R_9 0x18FF0500

	#define CAN_R_10 0x18FF1E03
	#define CAN_R_11 0x18FF1F03
  #define CAN_R_12 0x0CFF3003
		
		
	
	#define CAN_T_1  0x10FBFBF1
//	#define CAN_T_2  0x405

	#define CAN_LOCK_DATA_LEN        10
	
	#define ONE_PACKET_LEN 			 12
	#define CAN_RX_ID_NUM			 14
	#define CAN_BUF_LEN 			 (ONE_PACKET_LEN*CAN_RX_ID_NUM)
	
	
	#define CAN_DATA_START_ADDR 	METER_DATA_START_ADDR///仪表非易失数据起始地址

	#define CAN_LOCK_DATA_ADDR 		CAN_DATA_START_ADDR
	
	
	#define RELAY_LOCK_TYPE 1
	#define METER_LOCK_TYPE 2

	#define LOCK_LEVEL_0    0
	#define LOCK_LEVEL_1    1
	#define LOCK_LEVEL_2    2
	#define LOCK_LEVEL_3    3

	#define LOCK_CENTER_REASON  			1
	#define LOCK_SMS_REASON  				2
	#define LOCK_TERM_REASON  				3
	#define LOCK_OTHER_REASON  				4

	#define CLOSE_MONITOR_TYPE 			0X01
	#define OPEN_MONITOR_TYPE			0X02
	#define CENTER_UNLOCK_TYPE			0X03
	#define SMS_UNLOCK_TYPE				0X04
	#define CENTER_NORMAL_LOCK_TYPE		0X05
	#define IMMEDIATE_LOCK_TYPE			0X06
	#define SMS_LOCK_TYPE				0X07


	typedef struct 
	{
		uint8 tx_data[8];///发送数据
		uint8 tx_center_data_flag;///发送平台数据
		uint8 tx_data_type;///发送数据类型
		
		uint8 tx_lock_flag;///发送锁车标志
		uint8 check_val;///校验值
		
	}CAN_LOCK_STRUCT;
		
	typedef struct
	{
		uint8 can_open_flag;///CAN总线打开标志
		uint8 rx_right_data_flag;///接收到正确数据标志
		uint8 tx_ack_flag;///发送CAN应答数据标志
		
		uint8 rx_can_buf[CAN_BUF_LEN];///CAN数据
		
		uint8 set_controller_para_flag;
		uint8 set_controller_para[20];
	}CAN_STRUCT;
	
	EXTN_CAN CAN_STRUCT can_struct;
	EXTN_CAN CAN_LOCK_STRUCT can_lock_struct;
  
	void CanFilterInit(uint8 filter_index,uint32 can_id_1,uint32 can_id_2);
	void CanDeInit(void);
	void CanInit(void);
	void CanDataInit(void) ;
	uint8 CanDataWrite(void);
	void CanTxData(uint8 data[],uint32 id);
	void CanTx(void);
	void CanRx(void);
	void CanProcess(void);
	void CanMonitor(uint16 past_sec);
	void CanMain(void);
	
#endif

