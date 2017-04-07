
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

	#ifdef PROTOCOL_GLOBAL
		#define PROTOCOL_EXTERN
	#else
		#define PROTOCOL_EXTERN extern
	#endif
			/***
			#define PRO_VERSION 					0x02///协议软件版本
			#define TERM_PRODUCER_ID				0X23///终端厂家：天泽
			
			#define PRO_LEN_INDEX 					0
			#define PRO_TERM_ID_INDEX 				2
			#define PRO_SEQ_INDEX 					8
			#define PRO_CMD_ID_INDEX 				10
			#define PRO_DATA_INDEX					11

			///下行命令ID
			#define PRO_DOWN_ACK_ID 				0X01
			#define PRO_DOWN_POSITION_ID 			0X02
			#define PRO_DOWN_TRACK_ID 				0X03
			#define PRO_DOWN_SET_PARA_ID 			0X04
			#define PRO_DOWN_UPGRADE_ID 			0X10
			#define PRO_DOWN_QUERY_PARA_ID  		0X11
			#define PRO_DOWN_SET_WORK_PARA_ID 		0X2C///工作参数设置，ACC开，关，上传时间间隔
			#define PRO_DOWN_SET_CONTROLLER_PARA_ID 0X40///控制器参数设置
			#define PRO_DOWN_RELAY_LOCK_UNLOCK_ID 	0X48///继电器锁车解锁
			///上行命令ID
			#define PRO_UP_HEARTBEAT_ID 			0X70
			#define PRO_UP_ACK_ID 					0X71
			#define PRO_UP_POSITION_ID 				0X72
			#define PRO_UP_PARA_QUERY_ID 			0x78
			#define PRO_UP_LOGIN_ID 				0X7b
			#define PRO_UP_UPGRADE_RESULT_ID 		0X7d
			#define PRO_UP_WORK_PARA_ID 			0X7f
			#define PRO_UP_WORK_TIME_STATISTIC_ID 	0X81
			#define PRO_UP_SLEEP_ID 				0X8b
			#define PRO_UP_ALARM_ID 				0X92
			#define PRO_UP_LSNAL_DATA_ID 			0X93
			#define PRO_UP_ACC_ON_OFF_ID 			0Xa1		
			#define PRO_UP_CONTROLLER_ACK_ID 		0X84///控制器应答	
			
			
			#define SYS_PARA_SMS_ALARM_CENTER_NUM_ID 	0X0103///报警短信中心
			#define SYS_PARA_APN_ID						0x0200///APN
			#define SYS_PARA_MASTER_IP_ID 				0x0201///主中心IP
			#define SYS_PARA_SLAVER_IP_ID 				0x0203///副中心IP
			#define SYS_PARA_PORT_ID 					0x0205///Port
			#define SYS_PARA_WORK_PARA_ID 				0x020B///工作参数上传查寻
			#define SYS_PARA_TERMINAL_ID 				0x0300///SIMCARD ID
			#define SYS_PARA_PASSWORD_ID 				0x0301///密码
			#define SYS_PARA_RECOVER_ID 				0x0302///恢复出厂设置
			#define SYS_PARA_SOFTWARE_VERSION_ID 		0x0303///程序版本号
			#define SYS_PARA_TERMINAL_SERIAL_ID 		0x030D///终端序列号
			#define SYS_PARA_HARDWARE_VERSION_ID 		0x0317///硬件版本号
			#define SYS_PARA_SLEEP_TIMER_ID 			0x020A///休眠时间
			#define SYS_PARA_ACC_STATISTIC_ID 			0x0304///ACC统计时间
			#define SYS_PARA_METER_ERR_TIMER_ID 		0x0312///仪表故障时间
			#define SYS_PARA_OVER_SPEED_ID 				0x0400///超速参数
			#define SYS_PARA_LOW_VOLTAGE_ALARM_ID 		0x0402///低压报警参数
			#define SYS_PARA_BIND_IMSI_ID 				0x0110///绑定SIM卡IMSI
			***/

			#define VEHICLE_TYPE 					0x1106
			#define PRO_VERSION 					0x04///协议软件版本
			#define TERM_PRODUCER_ID				0X01///终端厂家：天泽
			#define TERM_TYPE_ID					0X01///终端类型,TZ24N
			#define TERM_USER_ID					0X02///徐工科技
			
			#define PRO_LEN_INDEX 					0
			#define PRO_TERM_ID_INDEX 				2
			#define PRO_SEQ_INDEX 					11	//-命令序号
			#define PRO_CMD_ID_INDEX 				13	//-命令ID
			#define PRO_DATA_INDEX					14	//-结尾是两个字节的校验加一个结束符,所以多3个字节
			///下行命令ID
			#define PRO_DOWN_HEART_ID 	    		0X00
			#define PRO_DOWN_COMM_PARA_ACK_ID 	    0X01///通信参数应答
			#define PRO_DOWN_ACK_ID 				0X02
			#define PRO_DOWN_POSITION_ID 			0X03
			#define PRO_DOWN_SET_PARA_ID 			0X04
			#define PRO_DOWN_UPGRADE_ID 			0X06
			#define PRO_DOWN_QUERY_PARA_ID  		0X07
			#define PRO_DOWN_QUERY_WORK_PARA_ID 	0X08///工作参数查询
			#define PRO_DOWN_SET_WORK_PARA_ID 		0X09///工作参数设置，ACC开，关，上传时间间隔
			#define PRO_DOWN_SET_CONTROLLER_PARA_ID 0X0A///控制器参数设置
			#define PRO_DOWN_RELAY_LOCK_UNLOCK_ID 	0X0B///继电器锁车解锁

			///上行命令ID
			#define PRO_UP_AQUIRE_COMM_PARA_ID 		0X80///获取通信参数
			#define PRO_UP_HEARTBEAT_ID 			0X81
			#define PRO_UP_ACK_ID 					0X82
			#define PRO_UP_POSITION_ID 				0X83
			#define PRO_UP_PARA_QUERY_ID 			0x84
			#define PRO_UP_LOGIN_ID 				0X85
			#define PRO_UP_UPGRADE_RESULT_ID 		0X86
			#define PRO_UP_WORK_PARA_ID 			0X87
			#define PRO_UP_WORK_TIME_STATISTIC_ID 	0X88
			#define PRO_UP_SLEEP_ID 				0X89
			#define PRO_UP_ALARM_ID 				0X8A
			#define PRO_UP_LSNAL_DATA_ID 			0X8B		
			#define PRO_UP_CONTROLLER_ACK_ID 		0X8C///控制器应答	
			#define PRO_UP_ACC_ON_OFF_ID 			0X8D
			#define PRO_UP_CONTROLLER_ERROR_ID      0x8E
			#define PRO_UP_LAT_LONG_POSITION_ID     0x8F///经纬度信息止传
			
			#define SYS_PARA_SMS_ALARM_CENTER_NUM_ID 	0X0003///报警短信中心
			#define SYS_PARA_APN_ID						0x0005///APN
			#define SYS_PARA_MASTER_IP_ID 				0x0006///主中心IP
			#define SYS_PARA_MASTER_DNS_ID 				0x0007///主中心域名
			#define SYS_PARA_SLAVER_IP_ID 				0x0008///副中心IP
			#define SYS_PARA_SLAVER_DNS_ID 				0x0009///副中心域名
			#define SYS_PARA_PORT_ID 					0x000A///Port
			#define SYS_PARA_WORK_PARA_ID 				0x000E///工作参数上传
			#define SYS_PARA_TERMINAL_ID 				0x000F///SIMCARD ID
			#define SYS_PARA_PASSWORD_ID 				0x0010///密码
			#define SYS_PARA_RECOVER_ID 				0x0011///恢复出厂设置
			#define SYS_PARA_SOFTWARE_VERSION_ID 		0x0012///程序版本号
			#define SYS_PARA_TERMINAL_SERIAL_ID 		0x0015///终端序列号
			#define SYS_PARA_HARDWARE_VERSION_ID 		0x0019///硬件版本号
			#define SYS_PARA_SLEEP_TIMER_ID 			0x000D///休眠时间
			#define SYS_PARA_ACC_STATISTIC_ID 			0x0013///ACC统计时间
			#define SYS_PARA_METER_ERR_TIMER_ID 		0x0017///仪表故障时间
			#define SYS_PARA_OVER_SPEED_ID 				0x001B///超速参数
			#define SYS_PARA_LOW_VOLTAGE_ALARM_ID 		0x001C///低压报警参数
			#define SYS_PARA_LSNAL_LOCK_MIN_ID 			0x001D///盲区被动锁车时间，分钟
			
			#define SYS_PARA_DEBUG_ENABLE_ID 			0x0100///调试功能打开
			#define SYS_PARA_LSNAL_LOCK_ENABLE_ID 		0x1001///扩展盲区被动锁车使能
			#define SYS_PARA_RESET_ID 					0x1002///终端复位
		
			#define PRO_EXE_SUCCESS 				0
			#define PRO_EXE_FAILURE 				1

			#define PRO_MAX_ACC_ON_OFF_COUNT 		60///30对开关机信息

			
			#define PRO_MAX_TX_BUF_ARRAY 			4///最大4个发送缓存
			#define PRO_MAX_TX_BUF_COUNTER 			600///每个发送缓存最大600字节
			#define MASTER_IP_INDEX					1
			#define SLAVER_IP_INDEX					2
			#define PRO_ACC_ON_MAX_TX_BUF_COUNTER	256
			
			typedef struct
			{
				uint8  re_tx_buf[PRO_MAX_TX_BUF_ARRAY][PRO_MAX_TX_BUF_COUNTER];
				uint16 re_tx_len[PRO_MAX_TX_BUF_ARRAY];
				uint8 re_tx_counter[PRO_MAX_TX_BUF_ARRAY];
				uint8 re_tx_sec_counter[PRO_MAX_TX_BUF_ARRAY];
				uint8 re_tx_full_flag[PRO_MAX_TX_BUF_ARRAY];
				uint8 acc_on_tx_buf[PRO_MAX_TX_BUF_ARRAY][PRO_ACC_ON_MAX_TX_BUF_COUNTER];///ACC开,1分钟内,没有登录至中心时启用;
				uint8 acc_on_tx_sec_counter;///ACC开,1分钟秒计数
			}TX_STRUCT;
			typedef struct
			{
				uint8 aquire_comm_para_flag;///获取通信参数
				uint8 comm_para_ip[4];///均衡IP
				uint8 comm_para_port[2];///均衡PORT
				uint8 tx_login_flag;///发送登录使能标志
				uint8 login_center_flag;///登录中心指令
				uint8 tx_lsnal_data_flag;///发送盲区数据
				uint8 setup_link_falg;///TRUE,成功与中心建立连接
				uint8 link_center_ip_index;///TRUE,链接主中心，FALSE是副中心
				uint8 link_master_ip_counter;///连接主中心计数
				
				uint8 tx_flash_lsnal_falg;///发送的保存在FLASH中的盲区数据标志
				uint8 tx_lsnal_falg;///发送盲区数据标志
				uint16 no_rx_data_sec_counter;///没有收到数据计数
				uint16 tx_seq;
				uint16 tx_len;
				uint8  tx_cmd;
				TX_STRUCT tx_struct;
			}PRO_STRUCT;
			
			typedef struct
			{
				uint16 pro_id;
				uint16 serial_id;
			}SERIAL_COMM_STRUCT;
			

			void ProParaInit(void);
			void ProUpAquireCommPara(void);
			void ProUpHeartBeat(void);
			void ProUpAck(uint8 data[],uint8 res);
			void ProUpPosition(void);
			void ProUpLogin(void);
			void ProUpUpgradeResult(uint8 res);
			void ProUpWorkPara(void);
			void ProUpWorkTimeStatistic(uint8 data[],uint8 len);
			void ProUpSleep(void);
			void ProUpAlarm(uint8 alarm_type);
			void ProUpLsnalData(void);
			void ProUpAccOnOff(uint8 acc_state);
			void ProUpControllerErrorCode(uint8 tx_data[],uint8 data_len);
			void ProUpControllerAck(uint8 tx_data[],uint8 data_len);
			void ProUpLatLongPosition(uint8 data[],uint8 len);
			void ProDownAckFun(uint8 data[],uint16 len);
			void ProDownCommParaAck(uint8 data[],uint16 len);
			void ProDownAck(uint8 data[],uint16 len);
			void ProDownTrack(uint8 data[],uint16 len);
			void ProDownSetPara(uint8 data[],uint8 len);
			void ProDownParaQuery(uint8 data[],uint8 len);
			void ProDownUpgrade(uint8 data[],uint8 len);
			void ProDownSetWorkPara(uint8 data[],uint8 len);
			void ProDownSetControlPara(uint8 data[],uint8 len);
			
			void ProPutIntoAccOnBuf(uint8 index,uint16 tx_len,uint8 tx_cmd,uint8 tx_data[]);
			void ProGetFromAccOnBuf(void);
			
			void ProConstructFrameHead(uint8 data[],uint16 tx_len,uint8 cmd);
			void ProConstructFrameTail(uint8 data[],uint16 tx_len);
			void ProPacket(uint8 tx_data[],u16 tx_len,uint8 tx_cmd,uint8 ack_flag);
			void ProPutIntoLsnal(uint8 data[],uint16 len,uint8 cmd);
			
			void ProPeriodTx(uint16 past_sec);
			void ProProcess(uint8 data[],uint16 len);
			
			void ProLsnalHeadTailSave(void);
			void ProLsnalDataInit(void);
			void ProLsnalPageSave(uint8 data[],uint16 len);
			void ProLsnalDataSave(uint8 data[],uint16 len,uint8 cmd);
			void ProLsnalSysExit(void);
			
			PROTOCOL_EXTERN PRO_STRUCT pro_struct;
#endif
