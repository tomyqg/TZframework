#define CAN_GLOBAL
#include "include.h"


void CanFilterInit(uint8 filter_index,uint32 can_id_1,uint32 can_id_2)
{
	u32 tmp_u32_val;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	CAN_FilterInitStructure.CAN_FilterNumber = filter_index;		///???¨1y???÷0 ￡????μ·??§ê? 0 - 13
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	///1y???÷???í 16 or 32; 16 bit ê± ??×éóé2??16??1y???÷×é3é￡?32ê±?ò??óDò???1y???÷

	tmp_u32_val = can_id_1;
	tmp_u32_val = tmp_u32_val << 3;
	tmp_u32_val |= 0x04;
	CAN_FilterInitStructure.CAN_FilterIdHigh = tmp_u32_val >> 16;			
	CAN_FilterInitStructure.CAN_FilterIdLow = tmp_u32_val;
	
	tmp_u32_val = can_id_2;
	tmp_u32_val = tmp_u32_val << 3;
	tmp_u32_val |= 0x04;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = tmp_u32_val >> 16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = tmp_u32_val;

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;		///1y???÷1?áaμ?FIFO
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}
void CanDeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
	CAN_DeInit(CAN1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void CanDataInit(void) 
{
	RamZero((uint8*)&can_lock_struct,CAN_LOCK_DATA_LEN+1);
	
	CanDataWrite();
}
uint8 CanDataWrite(void)
{
	uint8 res;
	
	can_lock_struct.check_val = U8SumCheck((uint8*)&can_lock_struct,CAN_LOCK_DATA_LEN+1);
	res = SpiFramWrite(CAN_LOCK_DATA_ADDR,(uint8*)&can_lock_struct,CAN_LOCK_DATA_LEN+2);
	
	return res;
}
void CanDataRead(void) 
{
	uint8 check_val;
	
	SpiFramRead(CAN_LOCK_DATA_ADDR,(uint8*)&can_lock_struct,CAN_LOCK_DATA_LEN+2);
	check_val = U8SumCheck((uint8*)&can_lock_struct,CAN_LOCK_DATA_LEN+1);
	if(check_val != can_lock_struct.check_val)
	{
		CanDataInit();
	}
}
void CanInit(void)
{
	GPIO_InitTypeDef 	   GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	///CanTxMsg TxMessage;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	///Configure CAN pin: TX 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	RCC_GetClocksFreq(&RCC_ClocksStatus);
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/// CAN cell init
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;

	CAN_InitStructure.CAN_Prescaler=RCC_ClocksStatus.PCLK1_Frequency/(16*250000);      ///波特率计算，16M/（1+8+7）/4=250k
	if(!CAN_Init(CAN1, &CAN_InitStructure))
	{
		///CAN初始化失败
		return;
	}
	CanFilterInit(0,CAN_R_0,CAN_R_1);
	CanFilterInit(1,CAN_R_2,CAN_R_3);
	CanFilterInit(2,CAN_R_4,CAN_R_5);
	CanFilterInit(3,CAN_R_6,CAN_R_7);
	CanFilterInit(4,CAN_R_8,CAN_R_9);
	CanFilterInit(5,CAN_R_10,CAN_R_11);
	CanFilterInit(6,CAN_R_12,0);
	
	
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CanDataRead();
}

void CanTxData(uint8 data[],uint32 id)
{
	CanTxMsg TxMessage;
//	TxMessage.StdId = id;
	TxMessage.ExtId = id;
	TxMessage.IDE = CAN_ID_EXT;//CAN_ID_STD;   
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	MemCpy(TxMessage.Data,data,8);
	CAN_Transmit(CAN1, &TxMessage);
}


void CanTx(void)
{
	static uint8 center_tx_counter = 0x00;
	static uint8 controller_para_tx_counter = 0x00;
	uint8 tx_can_data_flag;
	uint8 mask_val,term_lock_flag;
	uint8 lock_val;
	uint16 bias_addr,t3_id;
	uint8 data[8] = {0x34,0xe7,0x00,0x00,0x00,0x00,0x00,0x00};
	uint8 heart_data[8] = {0x34,0xe7,0x00,0x00,0x00,0x00,0x00,0x00};
	
	
	
	
	term_lock_flag = (((sys_private_para_struct.lsnal_lock_enable_flag == 1)&&
					  (sys_work_para_struct.lsnal_min_counter >= sys_private_para_struct.lsnal_min_timer))||
					  (sys_work_para_struct.can_stop_heart_min_counter >= 43200));///盲区被动锁
	
	if((term_lock_flag)||(can_lock_struct.tx_lock_flag == VALID_VAL_AA))
	{
		
	}
	else
	{	
		CanTxData(heart_data,CAN_T_1);
	}
	
	///平台控制
	if(can_lock_struct.tx_center_data_flag == VALID_VAL_AA)
	{
		tx_can_data_flag = TRUE;
		mask_val = can_lock_struct.tx_data[2];
		lock_val = can_lock_struct.tx_data[3];
		if(mask_val == 0x64)
		{
			can_lock_struct.tx_data_type = CLOSE_MONITOR_TYPE;///屏蔽防拆
		}
		else if(mask_val == 0x65)
		{
			can_lock_struct.tx_data_type = OPEN_MONITOR_TYPE;///开启防拆
		}
		else if(lock_val == 0xAA)
		{
			can_lock_struct.tx_data_type = CENTER_UNLOCK_TYPE;///解锁
			lock_record_struct.lock_level = LOCK_LEVEL_0;///0级锁为解锁
			UnLockRecord();
		}
		else if(lock_val == 0x55)
		{
			can_lock_struct.tx_data_type = CENTER_NORMAL_LOCK_TYPE;///普通锁
			lock_record_struct.lock_level = LOCK_LEVEL_1;///一级锁为平台锁
			LockRecord(METER_LOCK_TYPE,LOCK_LEVEL_1,LOCK_CENTER_REASON);
		}
		else if(lock_val == 0xff)
		{
			can_lock_struct.tx_data_type = IMMEDIATE_LOCK_TYPE;///立即锁
			lock_record_struct.lock_level = LOCK_LEVEL_1;///一级锁为平台锁
			LockRecord(METER_LOCK_TYPE,LOCK_LEVEL_1,LOCK_CENTER_REASON);
		}
		else
		{
			tx_can_data_flag = FALSE;
		}
		
		if(tx_can_data_flag)
		{
			MemCpy(data,can_lock_struct.tx_data,8);
			CanTxData(data,CAN_T_1);
			center_tx_counter++;
			if(center_tx_counter >= 2)
			{
				center_tx_counter = 0;
				if((lock_val == 0x55)||(lock_val == 0xff))
				{
					RamZero((uint8*)&can_lock_struct,CAN_LOCK_DATA_LEN);
				}
				else
				{
					RamZero((uint8*)&can_lock_struct,CAN_LOCK_DATA_LEN+1);
				}
				CanDataWrite();
			}
		}
	}
	///短信控制
	if((sys_work_para_struct.sms_lock_car_flag == VALID_VAL_AA)||(sys_work_para_struct.sms_lock_car_flag == VALID_VAL_BB))///短信控制
	{
		tx_can_data_flag = FALSE;
		RamZero(data,8);
		if(sys_work_para_struct.sms_lock_car_flag == VALID_VAL_AA)
		{
			data[2] = 0x65;
			data[3] = 0x55;
			lock_record_struct.lock_level = LOCK_LEVEL_2;///二级锁为短信锁
			can_lock_struct.tx_data_type = SMS_LOCK_TYPE;///锁车
			tx_can_data_flag = TRUE;
			LockRecord(METER_LOCK_TYPE,LOCK_LEVEL_2,LOCK_SMS_REASON);
		}
		else
		{///短信解锁
			data[3] = 0xAA;
			tx_can_data_flag = TRUE;
			can_lock_struct.tx_data_type = SMS_UNLOCK_TYPE;
			lock_record_struct.lock_level = LOCK_LEVEL_0;///0级锁为解锁
			UnLockRecord();
		}
		
		if(tx_can_data_flag)
		{
			CanTxData(data,CAN_T_1);
			center_tx_counter++;
			if(center_tx_counter >= 2)
			{
				center_tx_counter = 0;
				sys_work_para_struct.sms_lock_car_flag = INVALID_VAL_FF;
				bias_addr = ((uint8*)&sys_work_para_struct.sms_lock_car_flag) - ((uint8*)sys_work_para_struct.acc_on_sec_statistic_counter);
				SpiFramWrite(SYS_WORK_PARA_STRUCT_START_ADDR+bias_addr,(uint8*)&sys_work_para_struct.sms_lock_car_flag,1);
				
			}
		}
	}

/*	
	if(can_struct.set_controller_para_flag == VALID_VAL_2A)
	{
		CanTxData(can_struct.set_controller_para+2,CAN_T_2);
		t3_id = can_struct.set_controller_para[10] << 8;
		t3_id += can_struct.set_controller_para[11];
		CanTxData(can_struct.set_controller_para+12,t3_id);
		controller_para_tx_counter++;
		if(controller_para_tx_counter >= 5)
		{
			controller_para_tx_counter =0;
			can_struct.set_controller_para_flag = INVALID_VAL_FF;
		}
	}
*/	
}

void CanRx(void)
{
	uint8 *p_tmp = NULL;
	CanRxMsg can_msg;
	
	CAN_Receive(CAN1,CAN_FIFO0,&can_msg);
	
	if(0x08 == can_msg.DLC)
	{
		switch(can_msg.ExtId)
		{
			case CAN_R_0:
			{
				p_tmp = can_struct.rx_can_buf;
				break;
			}
			case CAN_R_1:
			{
				p_tmp = can_struct.rx_can_buf + 1*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_2:
			{
				p_tmp = can_struct.rx_can_buf + 2*ONE_PACKET_LEN;
				break;
			}
			
			case CAN_R_3:
			{
				p_tmp = can_struct.rx_can_buf + 3*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_4:
			{
				p_tmp = can_struct.rx_can_buf + 4*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_5:
			{
				p_tmp = can_struct.rx_can_buf + 5*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_6:
			{
				p_tmp = can_struct.rx_can_buf + 6*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_7:
			{
				p_tmp = can_struct.rx_can_buf + 7*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_8:
			{
				p_tmp = can_struct.rx_can_buf + 8*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_9:
			{
				p_tmp = can_struct.rx_can_buf + 9*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_10:
			{
				p_tmp = can_struct.rx_can_buf + 10*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_11:
			{
				p_tmp = can_struct.rx_can_buf + 11*ONE_PACKET_LEN;
				break;
			}
			case CAN_R_12:
			{
				p_tmp = can_struct.rx_can_buf + 12*ONE_PACKET_LEN;
				break;
			}
/*
			case CAN_R_12:
			{
				p_tmp = can_struct.rx_can_buf + 12*ONE_PACKET_LEN;
				break;
			}
*/
			default:
			{
				p_tmp = NULL;
				break;
			}
		}
	}
	
	if(can_msg.ExtId == CAN_R_9)
	{
		///中心处理

		if(can_lock_struct.tx_data_type == CLOSE_MONITOR_TYPE)
		{
//			if((can_msg.Data[0] & 0X80) == 0X80)
			if((can_msg.Data[0] & 0X08) == 0X08)
			{
				can_struct.tx_ack_flag = TRUE;
			}
		}
	
		if(can_lock_struct.tx_data_type == OPEN_MONITOR_TYPE)
		{
//			if((can_msg.Data[0] & 0X80) == 0X00)
			if((can_msg.Data[0] & 0X08) == 0X00)
			{
				can_struct.tx_ack_flag = TRUE;
			}
		}
		
		if((can_lock_struct.tx_data_type == CENTER_NORMAL_LOCK_TYPE)||
		   (can_lock_struct.tx_data_type == IMMEDIATE_LOCK_TYPE))
		{
//			if(((can_msg.Data[0] & 0X08) == 0x08)||((can_msg.Data[0] & 0X40) == 0X40))
			if(((can_msg.Data[0] & 0X02) == 0x02)||((can_msg.Data[0] & 0X04) == 0X04))
			{
				can_struct.tx_ack_flag = TRUE;
			}
		}
		
		if(can_lock_struct.tx_data_type == CENTER_UNLOCK_TYPE)
		{
//			if((can_msg.Data[0] & 0X08) == 0x00)
			if((can_msg.Data[0] & 0X02) == 0x00)
			{
				can_struct.tx_ack_flag = TRUE;
			}
		}
		
		///终端处理
		
//		if((can_msg.Data[0] & 0X08) == 0x08)
		if((can_msg.Data[0] & 0X02) == 0x02)
		{
			sys_work_para_struct.term_run_status_word |= STATE_METER_LOCK;///置锁车标志
		}
		else
		{
			sys_work_para_struct.term_run_status_word &= ~STATE_METER_LOCK;///清锁车标志
		}
	}
	
	can_struct.rx_right_data_flag = TRUE;
	
	if(p_tmp == NULL)
	{
		return;
	}
	*p_tmp++ = (uint8)(can_msg.ExtId >>24);
	*p_tmp++ = (uint8)(can_msg.ExtId >>16);
	*p_tmp++ = (uint8)(can_msg.ExtId >>8);
	*p_tmp++ = (uint8)can_msg.ExtId;
	
	MemCpy(p_tmp,can_msg.Data,8);
}

void CanMonitor(uint16 past_sec)
{
	static uint8 s_sec_counter_1 = 0;
	static uint16 s_sec_counter_2 = 0;

	if((!ACC_STATE())||(sys_work_para_struct.term_run_status_word & STATE_ACC_ON))
	{
		CanTx();
		
		if(can_struct.rx_right_data_flag)
		{
			can_struct.rx_right_data_flag = FALSE;
			s_sec_counter_2 = 0;
			if(sys_work_para_struct.term_run_status_word & STATE_METER_COMM_BREAKDOWN)
			{
				sys_work_para_struct.term_run_status_word &= ~STATE_METER_COMM_BREAKDOWN;///仪表接收到数据，解除报警
				ProUpAlarm(ALRM_FLAG_CLEAR_BYTE|ALRM_FLAG_METER_COMM_BREAKDOWN);
				LocalDebug("meter break down alarm cancel\r\n",StrLen("meter break down alarm cancel\r\n",0),LOCAL_TEST_DEBUG);
			}
		}
		else
		{
			s_sec_counter_2 += past_sec;
			if(s_sec_counter_2 >= sys_private_para_struct.meter_comm_breakdown_alarm_sec_timer)///仪表无数据,报警
			{
				if(!(sys_work_para_struct.term_run_status_word & STATE_METER_COMM_BREAKDOWN))
				{
					sys_work_para_struct.term_run_status_word |= STATE_METER_COMM_BREAKDOWN;
					ProUpAlarm(ALRM_FLAG_METER_COMM_BREAKDOWN);
					LocalDebug("meter break down alarm\r\n",StrLen("meter break down alarm\r\n",0),LOCAL_TEST_DEBUG);
				}
			///	s_sec_counter_2 = sys_private_para_struct.meter_comm_breakdown_alarm_sec_timer;
				if(s_sec_counter_2 >= 300)
				{
					s_sec_counter_2 = 300;
				}
			}
		}
		///重启策略
		s_sec_counter_1 += past_sec;
		if((s_sec_counter_2 >= SYS_TASK_SEC_TIMER)&&
		   (s_sec_counter_1 >= SYS_TASK_SEC_TIMER))///无数据N秒重启1次
		{
			if(s_sec_counter_2 < 300)
		///	if(s_sec_counter_2 < sys_private_para_struct.meter_comm_breakdown_alarm_sec_timer)
			{
				LocalDebug("meter no rx data restart\r\n",StrLen("meter no rx data restart\r\n",0),LOCAL_TEST_DEBUG);
			///	RamZero(can_struct.rx_can_buf,CAN_BUF_LEN);
				s_sec_counter_1 = 0x00;
				OFF_CAN_PWR();			
				CanDeInit();
				SysDelay(2*WAIT_1S);
				CanInit();
				ON_CAN_PWR();
			}
		}
	}
}

void CanProcess(void)
{
	if(can_struct.tx_ack_flag)
	{
		can_struct.tx_ack_flag = FALSE;
		ProUpControllerAck(can_struct.rx_can_buf+9*ONE_PACKET_LEN,ONE_PACKET_LEN);
	}                                         
}
void CanMain(void)
{
	if(!ACC_STATE())///ACC开
	{
		if(!can_struct.can_open_flag)
		{
			can_struct.can_open_flag = TRUE;
			ON_CAN_PWR();
			CanInit();
		}
		
		CanProcess();
	}
	else
	{
		if((sys_work_para_struct.term_run_status_word & STATE_ACC_ON) == 0x00)
		{
			if(can_struct.can_open_flag)
			{
				can_struct.can_open_flag = FALSE;
				CanDeInit();
				OFF_CAN_PWR();
				RamZero(can_struct.rx_can_buf,CAN_BUF_LEN);
			}
		}
	}
}
