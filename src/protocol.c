#define PROTOCOL_GLOBAL
#include "include.h"

void ProParaInit(void)///上行协议参数初始化
{
	uint8 i;
	
	ftp_struct.ftp_upgrade_flag = FALSE;
	pro_struct.aquire_comm_para_flag = FALSE;
	pro_struct.tx_login_flag = FALSE;
	pro_struct.tx_lsnal_data_flag = FALSE;
	sys_misc_run_struct.up_heart_beat_sec_counter = 0x00;
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		if(pro_struct.tx_struct.re_tx_full_flag[i])
		{
			pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
			ProPutIntoLsnal(pro_struct.tx_struct.re_tx_buf[i]+PRO_DATA_INDEX,
							pro_struct.tx_struct.re_tx_len[i]-PRO_DATA_INDEX,
							pro_struct.tx_struct.re_tx_buf[i][PRO_CMD_ID_INDEX]);
		}
	}
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
	}
}
/*****上行通讯函数******/
void ProUpAquireCommPara(void)
{
	uint8 tx_buf[256],tx_cmd,tx_len;

	tx_len = GetGpsInfo(tx_buf);
	tx_cmd = PRO_UP_AQUIRE_COMM_PARA_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
}
void ProUpHeartBeat(void)
{
	uint8 tx_cmd;

	tx_cmd = PRO_UP_HEARTBEAT_ID;
	ProPacket(NULL,0,tx_cmd,FALSE);
}
void ProUpAck(uint8 data[],uint8 res)
{
	uint8 tx_buf[256],tx_cmd,i;
	
	i = 0;
	tx_buf[i++] = data[PRO_SEQ_INDEX];
	tx_buf[i++] = data[PRO_SEQ_INDEX+1];
	tx_buf[i++] = data[PRO_CMD_ID_INDEX];
	tx_buf[i++] = res;

	tx_cmd = PRO_UP_ACK_ID;
	ProPacket(tx_buf,i,tx_cmd,FALSE);
}
void ProUpPosition(void)
{
	uint8 tx_buf[256],tx_cmd,tx_len;
	
	tx_len = GetGpsInfo(tx_buf);
	
	tx_cmd = PRO_UP_POSITION_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,FALSE);
}
void ProUpLogin(void)
{
	uint8 tx_buf[256],tx_len,tx_cmd;

	tx_len = GetGpsInfo(tx_buf);
	
	tx_cmd = PRO_UP_LOGIN_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
	///sys_work_para_struct.up_work_para_sec_counter = sys_work_para_struct.up_work_para_sec_timer - 2;///登录后,上传1条工作参数
}
void ProUpUpgradeResult(uint8 res)
{
	uint8 tx_buf[256],tx_cmd;
	
	tx_buf[0] = res;
	tx_cmd = PRO_UP_UPGRADE_RESULT_ID;
	ProPacket(tx_buf,1,tx_cmd,FALSE);
}
void ProUpWorkPara(void)
{
	uint8 tx_buf[256],*p,tx_len,tx_cmd,i,i_index,can_data_len;
	uint16 tmp_val;
///	uint32 can_esr_reg;
	p = tx_buf;
	tx_len = GetGpsInfo(p);
	p += tx_len;
	
	*p++ = 0x00;
	*p++ = 0x01;
	*p++ = 0x00;
	*p++ = 0x04;
	*p++ = sys_work_para_struct.acc_on_sec_statistic_counter >> 24;
    *p++ = sys_work_para_struct.acc_on_sec_statistic_counter >> 16;
    *p++ = sys_work_para_struct.acc_on_sec_statistic_counter >> 8;
    *p++ = sys_work_para_struct.acc_on_sec_statistic_counter;

	*p++ = 0x00;
	*p++ = 0x02;
	*p++ = 0x00;
	*p++ = 0x01;
	*p++ = sys_misc_run_struct.gsm_csq_val;///GSM信号强度
	
	*p++ = 0x00;
	*p++ = 0x03;
	*p++ = 0x00;
	*p++ = 0x02;
	tmp_val = sys_misc_run_struct.power_voltage_val * 100;
	*p++ = tmp_val >> 8;///车机电瓶电压
	*p++ = tmp_val;
	
	*p++ = 0x00;
	*p++ = 0x04;
	*p++ = 0x00;
	*p++ = 0x01;
	*p++ = sys_data_struct.gps_info_of_gprs[SAT_INDEX];///卫星个数
	
	*p++ = VEHICLE_TYPE >> 8;
	*p++ = (VEHICLE_TYPE&0xff);
	
	*p++ = 0x00;
	i_index = p - tx_buf;
	*p++;
	can_data_len = 0;
	
	for(i=0;i<CAN_RX_ID_NUM;i++)
	{
		if(can_struct.rx_can_buf[ONE_PACKET_LEN*i] != 0x00)
		{
			MemCpy(p,can_struct.rx_can_buf+ONE_PACKET_LEN*i,ONE_PACKET_LEN);
			can_data_len += ONE_PACKET_LEN;
			p += ONE_PACKET_LEN;
    }
	}
/**	
	if(sys_work_para_struct.term_run_status_word & STATE_ACC_ON)
	{
		can_esr_reg = CAN1->ESR;
		*p++ = 0X18;
		*p++ = 0X18;
		*p++ = 0X18;
		*p++ = 0X18;
		
		*p++ = can_esr_reg >> 24;
		*p++ = can_esr_reg >> 16;
		*p++ = can_esr_reg >> 8;
		*p++ = can_esr_reg;
	}
**/	
	tx_buf[i_index] = can_data_len;
	RamZero(can_struct.rx_can_buf,CAN_BUF_LEN);
	
	tx_len = p - tx_buf;
	tx_cmd = PRO_UP_WORK_PARA_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
}
void ProUpWorkTimeStatistic(uint8 data[],uint8 len)
{
	uint8 tx_cmd;
	
	tx_cmd = PRO_UP_WORK_TIME_STATISTIC_ID;
	ProPacket(data,len,tx_cmd,TRUE);
}
void ProUpSleep(void)
{
	uint8 tx_buf[256],tx_len,tx_cmd;

	tx_len = GetGpsInfo(tx_buf);
	tx_cmd = PRO_UP_SLEEP_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
}
void ProUpAlarm(uint8 alarm_type)
{
	uint8 tx_buf[256],tx_len,tx_cmd;

	tx_buf[0] = alarm_type & 0x8F;
	tx_len = GetGpsInfo(tx_buf+1);
	tx_len += 1;
	tx_cmd = PRO_UP_ALARM_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
}
void ProUpControllerAck(uint8 tx_data[],uint8 data_len)
{
	uint8 tx_buf[256],tx_len,tx_cmd;
	uint8 *p;
	
	p = tx_buf;
	GetGpsInfo(p);
	p += GPS_INFO_LEN;
	*p++ = VEHICLE_TYPE >> 8;
	*p++ = (VEHICLE_TYPE & 0XFF);
	
	*p++ = 0x00;
	*p++ = data_len+1;
	
	*p++ = data_len;
	
	MemCpy(p,tx_data,data_len);
	p += data_len;
	tx_cmd = PRO_UP_CONTROLLER_ACK_ID;
	tx_len = p - tx_buf;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
}

void ProUpControllerErrorCode(uint8 tx_data[],uint8 data_len)
{/**
	uint8 tx_buf[256],tx_len,tx_cmd;
	uint8 *p;
	
	p = tx_buf;
	GetGpsInfo(p);
	p += 8;
	MemCpy(p+8,tx_buf+DATE_INDEX,6);
	p += 6;
	*p++ = VEHICLE_TYPE >> 8;
	*p++ = (VEHICLE_TYPE & 0XFF);
	if(data_len > 0)
	{
		*p++ = 0x01;///故障源种数
		*p++ = (4+data_len) >> 8;
		*p++ = 4+data_len;
		Uint32ToU8(p,CAN_R_CS);
		p += 4;
		MemCpy(p,tx_data,data_len);
		p += data_len;
	}
	else
	{
		*p++ = 0x00;
	}
	tx_len = p - tx_buf;
	tx_cmd = PRO_UP_CONTROLLER_ERROR_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
**/
}
void ProUpLsnalData(void)
{
	uint8 tx_buf[PRO_MAX_TX_BUF_COUNTER],*p,tx_cmd;
	uint8 tmp_buf[LSNAL_PAGE_SIZE];
	uint16 tmp_len;
	uint16 tx_len;

	if(pro_struct.tx_lsnal_falg)///没有盲区数据在发送
	{
		goto RETURN_LAB;
	}
	
	if(sys_work_para_struct.lsnal_tail_page >= FLASH_LSNAL_MAX_PAGES)
	{
		sys_work_para_struct.lsnal_head_page = 0;
		sys_work_para_struct.lsnal_tail_page = 0;
		ProLsnalHeadTailSave();
	}
	
	if(sys_work_para_struct.lsnal_head_page != sys_work_para_struct.lsnal_tail_page)
	{
		FlashRead(FLASH_LSNAL_START_ADDR+sys_work_para_struct.lsnal_tail_page*LSNAL_PAGE_SIZE,tmp_buf,LSNAL_PAGE_SIZE);
		tmp_len = (tmp_buf[1] << 8) + tmp_buf[2];
		if((tmp_buf[0] != 0x2A)||(tmp_len > LSNAL_PAGE_SIZE - 1))
		{
			sys_work_para_struct.lsnal_tail_page = (sys_work_para_struct.lsnal_tail_page+1)%FLASH_LSNAL_MAX_PAGES;
			ProLsnalHeadTailSave();
			goto RETURN_LAB;
		}
		if(tmp_buf[tmp_len] != U8SumCheck(tmp_buf,tmp_len))
		{
			sys_work_para_struct.lsnal_tail_page = (sys_work_para_struct.lsnal_tail_page+1)%FLASH_LSNAL_MAX_PAGES;
			ProLsnalHeadTailSave();
			goto RETURN_LAB;
		}
		pro_struct.tx_flash_lsnal_falg = TRUE;
	}
	else
	{
		if(sys_data_struct.lsnal_data[0] != VALID_VAL_2A)
		{
			goto RETURN_LAB;
		}
		MemCpy(tmp_buf,sys_data_struct.lsnal_data,LSNAL_PAGE_SIZE);
		tmp_len = (tmp_buf[1] << 8) + tmp_buf[2];
		pro_struct.tx_flash_lsnal_falg = FALSE;
	}
	
	tmp_len -= 3;
	p = tx_buf;
	GetGpsInfo(p);
	p += 8;
	MemCpy(p,tx_buf+DATE_INDEX,6);
	p += 6;
	MemCpy(p,tmp_buf+3,tmp_len);
	p += tmp_len;
	tx_len = p - tx_buf;
	tx_cmd = PRO_UP_LSNAL_DATA_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
	pro_struct.tx_lsnal_falg = TRUE;
RETURN_LAB:
	return;
}
void ProUpAccOnOff(uint8 acc_state)
{
	uint8 tx_buf[256],tx_len,tx_cmd;

	tx_len = GetGpsInfo(tx_buf);
	tx_buf[tx_len] = acc_state;
	tx_len += 1;
	tx_cmd = PRO_UP_ACC_ON_OFF_ID;
	ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
}
void ProUpLatLongPosition(uint8 data[],uint8 len)
{
	uint8 tx_cmd;

	tx_cmd = PRO_UP_LAT_LONG_POSITION_ID;
	ProPacket(data,len,tx_cmd,FALSE);
}
/******下行通讯函数******/
void ProDownCommParaAck(uint8 data[],uint16 len)
{
	uint8 i,apn_len,*p_tmp,res = PRO_EXE_FAILURE;

	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
	}
	p_tmp = data + PRO_DATA_INDEX;
	len -= PRO_DATA_INDEX;
	
	apn_len = p_tmp[0];
	if(apn_len > LEN_32 - 1)
	{
		goto RETURN_LAB;
	}
	if(len != 1 + apn_len + 4 + 2)
	{
		goto RETURN_LAB;
	}
	
	MemCpy(pro_struct.comm_para_ip,p_tmp+1+apn_len,4);
	MemCpy(pro_struct.comm_para_port,p_tmp+1+apn_len+4,2);
	
	if(pro_struct.link_center_ip_index == SLAVER_IP_INDEX)
	{
		pro_struct.link_center_ip_index = MASTER_IP_INDEX;
	}	
	else
	{
		pro_struct.link_center_ip_index = SLAVER_IP_INDEX;
	}	
	
	pro_struct.aquire_comm_para_flag = TRUE;
	pro_struct.tx_login_flag = TRUE;
	gsm_misc_struct.gsm_mode_exe_flag[3] = AT_EXE_SUCESS;
	
	res = PRO_EXE_SUCCESS;
	
RETURN_LAB:
	ProUpAck(data,res);
}
void ProDownAck(uint8 data[],uint16 len)
{
	uint8 i,tx_cmd,ack_cmd;
	uint16 tx_seq,ack_seq,bias_addr;

	ack_seq = data[PRO_DATA_INDEX] << 8;
	ack_seq += data[PRO_DATA_INDEX+1];
	ack_cmd = data[PRO_DATA_INDEX+2];
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		tx_seq = pro_struct.tx_struct.re_tx_buf[i][PRO_SEQ_INDEX] << 8;
		tx_seq += pro_struct.tx_struct.re_tx_buf[i][PRO_SEQ_INDEX+1];
		tx_cmd = pro_struct.tx_struct.re_tx_buf[i][PRO_CMD_ID_INDEX];
	
		if((tx_cmd == ack_cmd)&&(tx_seq == ack_seq))
		{
			pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
			break;
		}
	}
	
	if(i == PRO_MAX_TX_BUF_ARRAY)
	{
		goto RETUR_LAB;
	}
	
	switch(data[PRO_DATA_INDEX+2])
	{	
		case PRO_UP_LOGIN_ID:
		{
			pro_struct.login_center_flag = TRUE;
			for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
			{
				pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
			}
			if(sys_misc_run_struct.sys_cold_start_falg)
			{
				sys_misc_run_struct.sys_cold_start_falg = FALSE;
				ProUpWorkPara();
			}
			break;
		}
		case PRO_UP_LSNAL_DATA_ID:
		{
			if(pro_struct.tx_flash_lsnal_falg)
			{
				sys_work_para_struct.lsnal_tail_page = (sys_work_para_struct.lsnal_tail_page+1) % FLASH_LSNAL_MAX_PAGES;
				ProLsnalHeadTailSave();
			}
			else
			{
				sys_data_struct.lsnal_data[0] = INVALID_VAL_FF;
				bias_addr = ((uint8*)sys_data_struct.lsnal_data) - ((uint8*)sys_data_struct.gps_info_of_gprs);
				SpiFramWrite(SYS_DATA_STRUCT_START_ADDR+bias_addr,sys_data_struct.lsnal_data,1);
			}
			pro_struct.tx_lsnal_falg = FALSE;
			break;
		}
		default:
		{
			break;
		}
	}
	pro_struct.tx_lsnal_data_flag = TRUE;
	pro_struct.tx_struct.acc_on_tx_sec_counter = 2*SYS_TASK_SEC_TIMER;///ACC开,延时60秒到
RETUR_LAB:
	return;
}
void ProDownTrack(uint8 data[],uint16 len)
{
	uint8 res = PRO_EXE_FAILURE;
	uint8 wr_res;
	uint16 tmp_val;
	
	tmp_val = data[PRO_DATA_INDEX] << 8;
	tmp_val += data[PRO_DATA_INDEX+1];
	if((tmp_val != 0)&&(tmp_val <= 5))///追踪最小间隔为5秒
	{
		goto RETURN_LAB;
	}
	
	sys_work_para_struct.track_of_acc_on_sec_timer = tmp_val;
	sys_work_para_struct.track_of_acc_on_sec_counter = 0x00;
	

	wr_res = SysWorkParaWrite();
	if(wr_res)
	{
		res = PRO_EXE_SUCCESS;
	}
RETURN_LAB:
	ProUpAck(data,res);
}
void ProDownSetPara(uint8 data[],uint8 len)
{
	uint8 res,*tm_p;
	
	tm_p = data + PRO_DATA_INDEX;
	res = SetSysPrivatePara(tm_p,len-PRO_DATA_INDEX,FALSE);
	
	ProUpAck(data,res);
	
	if(sys_boot_para_struct.sys_para_init_flag == VALID_VAL_DWORD_AA)
	{
		SysBootParaWrite();
		SysReset();
	}
}
void ProDownParaQuery(uint8 data[],uint8 len)
{
	uint8 tmp_data[PRO_MAX_TX_BUF_COUNTER];
	uint16 tmp_len;
	
	tmp_len = QuerySysPrivatePara(tmp_data,data+PRO_DATA_INDEX,len-PRO_DATA_INDEX,FALSE);
	
	ProPacket(tmp_data,tmp_len,PRO_UP_PARA_QUERY_ID,FALSE);
}
void ProDownUpgrade(uint8 data[],uint8 len)
{
	uint8 res,ack = PRO_EXE_FAILURE;
	uint8 upgrade_res = FAILURE_ACK;

	res = FtpAddrAnalysis(data+PRO_DATA_INDEX+1,len-PRO_DATA_INDEX-1);
	if(res)
	{
		ack = PRO_EXE_SUCCESS;
	}
	
	ProUpAck(data,ack);	//-给予应答
	
	if(res)
	{
		res = FtpMain();
		if(res)
		{
			upgrade_res = SUCCESS_ACK;
		}
		
		UdpIpPortInit();
		ProUpUpgradeResult(upgrade_res);///上传后台,升级标志
		
		if(res)
		{
			LocalDebug("upgrade_success\r\n",StrLen("upgrade_success\r\n",0),LOCAL_TEST_DEBUG);
			LocalCommFtpState(0,ftp_struct.ftp_file_total_size,ftp_struct.ftp_rx_file_byte_counter);
			SysReset();///置程序更新标志，复位；
		}
		else
		{
			LocalDebug("upgrade_fail\r\n",StrLen("upgrade_fail\r\n",0),LOCAL_TEST_DEBUG);
			LocalCommFtpState(1,ftp_struct.ftp_file_total_size,ftp_struct.ftp_rx_file_byte_counter);
		}
	}
}
void ProDownSetWorkPara(uint8 data[],uint8 len)
{
	uint8 res;
	
	res = SetWorkPara(data+PRO_DATA_INDEX,len-PRO_DATA_INDEX);
	ProUpAck(data,res);
}
void ProDownSetControlPara(uint8 data[],uint8 len)
{
	uint8 res,*tm_p,ack = PRO_EXE_FAILURE;
	uint16 bias_addr;
	uint32 can_id;

	ack = PRO_EXE_FAILURE;
	tm_p = data + PRO_DATA_INDEX;
	
	tm_p += 2;
	can_id = tm_p[0];
	can_id = (can_id << 24) + (tm_p[1]<<16) + (tm_p[2]<<8) + tm_p[3];
	
	if(can_id == CAN_T_1)
	{
		can_lock_struct.tx_lock_flag = 0x00;

		MemCpy(can_lock_struct.tx_data,tm_p+4,8);
		if((can_lock_struct.tx_data[3] == 0x55)||(can_lock_struct.tx_data[3] == 0xff))
		{
			can_lock_struct.tx_lock_flag = VALID_VAL_AA;
		}
		
		sys_work_para_struct.sms_lock_car_flag = INVALID_VAL_FF;
		bias_addr = ((uint8*)&sys_work_para_struct.sms_lock_car_flag) - ((uint8*)sys_work_para_struct.acc_on_sec_statistic_counter);
		SpiFramWrite(SYS_WORK_PARA_STRUCT_START_ADDR+bias_addr,(uint8*)&sys_work_para_struct.sms_lock_car_flag,1);
		
		
		can_lock_struct.tx_center_data_flag = VALID_VAL_AA;
		
		res = CanDataWrite();
		if(res)
		{
			ack = PRO_EXE_SUCCESS;
		}
		else
		{
			can_lock_struct.tx_center_data_flag = FALSE;
		}
	}
/*
	else if(can_id == CAN_T_2)
	{
		can_struct.set_controller_para_flag = VALID_VAL_2A;
		MemCpy(can_struct.set_controller_para,tm_p,20);
		ack = PRO_EXE_SUCCESS;
	}
*/	
	ProUpAck(data,ack);
}
void ProConstructFrameHead(uint8 data[],uint16 tx_len,uint8 cmd)
{
	uint8 *p = data;
	
	*p++= (tx_len+PRO_DATA_INDEX) >> 8;
	*p++ = tx_len+PRO_DATA_INDEX;
	*p++ = sys_private_para_struct.terminal_id[0];
	*p++ = sys_private_para_struct.terminal_id[1];
	*p++ = sys_private_para_struct.terminal_id[2];
	*p++ = sys_private_para_struct.terminal_id[3];
	*p++ = sys_private_para_struct.terminal_id[4];
	*p++ = PRO_VERSION;
	*p++ = TERM_PRODUCER_ID;
	*p++ = TERM_TYPE_ID;
	*p++ = TERM_USER_ID;
	*p++ = pro_struct.tx_seq >> 8;
	*p++ = pro_struct.tx_seq;
	*p++ = cmd;
	pro_struct.tx_seq++;
}
void ProConstructFrameTail(uint8 data[],uint16 tx_len)
{
	data[tx_len] = XorCheck(data,tx_len);
	data[tx_len+1] = 0x0d;
	data[tx_len+2] = 0x0a;
}
void ProPacket(uint8 tx_data[],uint16 tx_len,uint8 tx_cmd,uint8 ack_flag)
{
	uint8 i,*p,tmp_data[PRO_MAX_TX_BUF_COUNTER];
	uint16 tmp_len;
	#ifdef PRO_DEBUG
		char str_ch[10];
		uint8 str_len;
	#endif

	if(!pro_struct.aquire_comm_para_flag)
	{
		if(tx_cmd != PRO_UP_AQUIRE_COMM_PARA_ID)
		{
			if(ack_flag)
			{
				goto LSNAL_LAB;
			}
			else if(tx_cmd == PRO_UP_ACK_ID)
			{
			
			}
			else
			{
				goto RETURN_LAB;
			}
		}
	}
	
	if(!pro_struct.login_center_flag)
	{
		if((tx_cmd != PRO_UP_AQUIRE_COMM_PARA_ID)&&(tx_cmd != PRO_UP_LOGIN_ID))
		{
			if(ack_flag)
			{
				goto LSNAL_LAB;
			}
			else if(tx_cmd == PRO_UP_ACK_ID)
			{
			
			}
			else
			{
				goto RETURN_LAB;
			}
		}
	}
	
	
	p = tmp_data;
	ProConstructFrameHead(p,tx_len,tx_cmd);
	p += PRO_DATA_INDEX;
	MemCpy(p,tx_data,tx_len);
	tmp_len = PRO_DATA_INDEX + tx_len;
	
	ProConstructFrameTail(tmp_data,tmp_len);
	tmp_len += 3;
	
	if(!ack_flag)
	{
		if(gsm_misc_struct.cur_mode == UDP_MODE)
		{
			#ifdef PRO_DEBUG
				str_len = sprintf(str_ch,"%s ","上行：");
				LocalDebug((uint8*)str_ch,str_len,LOCAL_PRO_DEBUG);	
				for(i=0;i<tmp_len;i++)
				{
					str_len = sprintf(str_ch,"%02X ",tmp_data[i]);
					LocalDebug((uint8*)str_ch,str_len,LOCAL_PRO_DEBUG);
				}
				LocalDebug("\r\n",2,LOCAL_PRO_DEBUG);
			#endif

				UdpTx(tmp_data,tmp_len);	//-一层层组包
				
				sys_misc_run_struct.up_heart_beat_sec_counter = 0x00;///上行心跳计时清0
		}
		goto RETURN_LAB;
	}
	
	if(tx_cmd == PRO_UP_AQUIRE_COMM_PARA_ID)
	{
		i = 0;
	}
	else
	{
		for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
		{
			if(!pro_struct.tx_struct.re_tx_full_flag[i])
			{
				break;
			}
		}
	}
	
	if(i == PRO_MAX_TX_BUF_ARRAY)
	{
		if(ack_flag)
		{
			goto LSNAL_LAB;
		}
	}
	
	MemCpy(pro_struct.tx_struct.re_tx_buf[i],tmp_data,tmp_len);
	pro_struct.tx_struct.re_tx_len[i] = tmp_len;
	pro_struct.tx_struct.re_tx_counter[i] = 0;
	pro_struct.tx_struct.re_tx_sec_counter[i] = 8;
	pro_struct.tx_struct.re_tx_full_flag[i] = TRUE;
	
	return;
LSNAL_LAB:
	if(pro_struct.tx_struct.acc_on_tx_sec_counter < 2*SYS_TASK_SEC_TIMER)
	{
		for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
		{
			if(pro_struct.tx_struct.acc_on_tx_buf[i][0] != VALID_VAL_2A)
			{
				break;
			}
		}
		if(i != PRO_MAX_TX_BUF_ARRAY)
		{
			if((tx_cmd == PRO_UP_WORK_TIME_STATISTIC_ID)||
			   (tx_cmd == PRO_UP_ALARM_ID)||
			   (tx_cmd == PRO_UP_ACC_ON_OFF_ID))
			{
				ProPutIntoAccOnBuf(i,tx_len,tx_cmd,tx_data);
				goto RETURN_LAB;
			}
		}
	}
	ProPutIntoLsnal(tx_data,tx_len,tx_cmd);
RETURN_LAB:
	return;
}
void ProPutIntoAccOnBuf(uint8 index,uint16 tx_len,uint8 tx_cmd,uint8 tx_data[])
{
	uint16 i = 0;
	
	pro_struct.tx_struct.acc_on_tx_buf[index][i++] = VALID_VAL_2A;
	pro_struct.tx_struct.acc_on_tx_buf[index][i++] = tx_cmd;
	pro_struct.tx_struct.acc_on_tx_buf[index][i++] = tx_len >> 8;
	pro_struct.tx_struct.acc_on_tx_buf[index][i++] = tx_len;
	MemCpy(&pro_struct.tx_struct.acc_on_tx_buf[index][i],tx_data,tx_len);
}
void ProGetFromAccOnBuf(void)
{
	uint8 i,tx_buf[256],tx_cmd;
	uint16 tx_len;
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		if(pro_struct.tx_struct.acc_on_tx_buf[i][0] == VALID_VAL_2A)
		{
			break;
		}
	}
	if(i != PRO_MAX_TX_BUF_ARRAY)
	{
		pro_struct.tx_struct.acc_on_tx_buf[i][0] = INVALID_VAL_FF;
		tx_cmd = pro_struct.tx_struct.acc_on_tx_buf[i][1];
		tx_len = (pro_struct.tx_struct.acc_on_tx_buf[i][2] << 8)+pro_struct.tx_struct.acc_on_tx_buf[i][3];
		MemCpy(tx_buf,&pro_struct.tx_struct.acc_on_tx_buf[i][4],tx_len);
		ProPacket(tx_buf,tx_len,tx_cmd,TRUE);
	}
}
void ProPutIntoLsnal(uint8 data[],uint16 len,uint8 cmd)
{
	
	switch(cmd)
	{
		case PRO_UP_WORK_PARA_ID:
		case PRO_UP_WORK_TIME_STATISTIC_ID:
		case PRO_UP_ALARM_ID:
		case PRO_UP_ACC_ON_OFF_ID:
		case PRO_UP_LSNAL_DATA_ID:
		{
			if(cmd == PRO_UP_LSNAL_DATA_ID)
			{
				pro_struct.tx_lsnal_falg = FALSE; 
			}
			ProLsnalDataSave(data,len,cmd);///写盲区
			break;
		}
		case PRO_UP_AQUIRE_COMM_PARA_ID:///对
		{
			if(pro_struct.link_center_ip_index == MASTER_IP_INDEX)
			{
				pro_struct.link_center_ip_index = SLAVER_IP_INDEX;
				MemCpy(udp_struct.udp_dst_ip,sys_private_para_struct.slaver_ip_dns+1,4);
				MemCpy(sys_misc_run_struct.link_center_ip,sys_private_para_struct.slaver_ip_dns+1,4);
				UdpIpPortInit();
				ProUpAquireCommPara();
			}
			else
			{
				pro_struct.link_center_ip_index = MASTER_IP_INDEX;
				pro_struct.link_master_ip_counter++;
				pro_struct.no_rx_data_sec_counter = 0;
				if(pro_struct.link_master_ip_counter > 3)///连接主副中心3次不成功，复位GSM模块
				{
					pro_struct.link_master_ip_counter = 1;
					gsm_misc_struct.cur_mode = POWER_INIT_MODE;
				}
				else
				{
					MemCpy(udp_struct.udp_dst_ip,sys_private_para_struct.master_ip_dns+1,4);
					MemCpy(sys_misc_run_struct.link_center_ip,sys_private_para_struct.master_ip_dns+1,4);
					UdpIpPortInit();
					ProUpAquireCommPara();
				}
			}
			break;
		}
		case PRO_UP_LOGIN_ID:
		{
			gsm_misc_struct.cur_mode = POWER_INIT_MODE;
			break;
		}
		default:
		{
			break;
		}
	}
}
void ProLsnalDataInit(void)
{
	sys_data_struct.lsnal_data[0] = VALID_VAL_2A;///有效标志
	sys_data_struct.lsnal_data[1] = 0x00;///长度,2字节,从标志开始，不含1字节校验
	sys_data_struct.lsnal_data[2] = 0x04;///
	sys_data_struct.lsnal_data[3] = 0x00;///总包数
	sys_data_struct.lsnal_data[4] = 0x2E;///校验
}
void ProLsnalHeadTailSave(void)///保存页码
{
	uint16 bias_addr;
	
	bias_addr = ((uint8*)&sys_work_para_struct.lsnal_head_page) - ((uint8*)&sys_work_para_struct.acc_on_sec_statistic_counter);
	SpiFramWrite(SYS_WORK_PARA_STRUCT_START_ADDR+bias_addr,(uint8*)&sys_work_para_struct.lsnal_head_page,4);
}
void ProLsnalPageSave(uint8 data[],uint16 len)
{
	uint8 res;
	
	if(sys_work_para_struct.lsnal_head_page >= FLASH_LSNAL_MAX_PAGES)
	{
		sys_work_para_struct.lsnal_head_page = 0;
		sys_work_para_struct.lsnal_tail_page = 0;
		ProLsnalHeadTailSave();
	}
	
	if((sys_work_para_struct.lsnal_head_page % 4)==0)///FLASH擦除 2K字节/页
	{
		res = FlashErase(FLASH_LSNAL_START_ADDR+sys_work_para_struct.lsnal_head_page*LSNAL_PAGE_SIZE);
		if(!res)
		{
			goto RETURN_LAB;
		}
	}
	
	FlashWrite(FLASH_LSNAL_START_ADDR+sys_work_para_struct.lsnal_head_page*LSNAL_PAGE_SIZE,data,len);
	sys_work_para_struct.lsnal_head_page = (sys_work_para_struct.lsnal_head_page + 1) % FLASH_LSNAL_MAX_PAGES;
	
	if(sys_work_para_struct.lsnal_head_page == sys_work_para_struct.lsnal_tail_page)///判断是否已满
	{
		sys_work_para_struct.lsnal_tail_page = (sys_work_para_struct.lsnal_tail_page + 4) % FLASH_LSNAL_MAX_PAGES;
	}	
	
	ProLsnalHeadTailSave();
RETURN_LAB:
	return;
}
void ProLsnalDataSave(uint8 data[],uint16 len,uint8 cmd)
{
	uint16 tmp_len;
	uint16 bias_addr;
	
	if(cmd == PRO_UP_LSNAL_DATA_ID)
	{
		goto RETURN_LAB;
	}
	
	if(sys_data_struct.lsnal_data[0] != VALID_VAL_2A)
	{
		ProLsnalDataInit();
	}
	
	tmp_len = sys_data_struct.lsnal_data[1] << 8;
	tmp_len += sys_data_struct.lsnal_data[2];
	
	if(U8SumCheck(sys_data_struct.lsnal_data,tmp_len) != sys_data_struct.lsnal_data[tmp_len])
	{
		ProLsnalDataInit();
		tmp_len = 4;
	}
	
	if(tmp_len + 1 + len + 3 > LSNAL_PAGE_SIZE)
	{
		ProLsnalPageSave(sys_data_struct.lsnal_data,tmp_len+1);
		
		ProLsnalDataInit();
		tmp_len = 4;
	}
	
	sys_data_struct.lsnal_data[1] = (tmp_len + len + 3) >> 8;
	sys_data_struct.lsnal_data[2] = tmp_len + len + 3;
	
	sys_data_struct.lsnal_data[3] += 1;
	
	sys_data_struct.lsnal_data[tmp_len] = cmd;
	
	sys_data_struct.lsnal_data[tmp_len + 1] = len >> 8;
	sys_data_struct.lsnal_data[tmp_len + 2] = len;
	
	MemCpy(sys_data_struct.lsnal_data+3+tmp_len,data,len);
	
	sys_data_struct.lsnal_data[tmp_len + 3 + len] = U8SumCheck(sys_data_struct.lsnal_data,tmp_len + 3 + len);
	
	bias_addr = ((uint8*)sys_data_struct.lsnal_data) - ((uint8*)sys_data_struct.gps_info_of_gprs);
	SpiFramWrite(SYS_DATA_STRUCT_START_ADDR+bias_addr,sys_data_struct.lsnal_data,tmp_len + 3 + len + 1);
RETURN_LAB:
	return;
}
void ProPeriodTx(uint16 past_sec)///周期发送
{
	uint16 i;
	static uint16 last_sec = 0;
	#ifdef PRO_DEBUG
		char str_ch[10];
		uint8 str_len;
		uint16 j;
	#endif
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		if(pro_struct.tx_struct.re_tx_sec_counter[i] >= 8)///8秒重发机制
		{
			pro_struct.tx_struct.re_tx_sec_counter[i] = 0;
			if(pro_struct.tx_struct.re_tx_full_flag[i])///有效数据
			{
				if(pro_struct.tx_struct.re_tx_counter[i] < 3)///重发小于3次
				{
					pro_struct.tx_struct.re_tx_counter[i]++;
					
					if(gsm_misc_struct.cur_mode == UDP_MODE)
					{
						#ifdef PRO_DEBUG
							str_len = sprintf(str_ch,"%s ","上行：");
							LocalDebug((uint8*)str_ch,str_len,LOCAL_PRO_DEBUG);	
							for(j=0;j<pro_struct.tx_struct.re_tx_len[i];j++)
							{
								str_len = sprintf(str_ch,"%02X ",pro_struct.tx_struct.re_tx_buf[i][j]);
								LocalDebug((uint8*)str_ch,str_len,LOCAL_PRO_DEBUG);
							}
							LocalDebug("\r\n",2,LOCAL_PRO_DEBUG);
						#endif
						UdpTx(pro_struct.tx_struct.re_tx_buf[i],pro_struct.tx_struct.re_tx_len[i]);
						sys_misc_run_struct.up_heart_beat_sec_counter = 0x00;///上行心跳计时清0
					}
					break;
				}
				else
				{
					pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
					ProPutIntoLsnal(pro_struct.tx_struct.re_tx_buf[i]+PRO_DATA_INDEX,
									pro_struct.tx_struct.re_tx_len[i]-PRO_DATA_INDEX-3,
									pro_struct.tx_struct.re_tx_buf[i][PRO_CMD_ID_INDEX]);
				}
			}
		}
	}
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		if(pro_struct.tx_struct.re_tx_full_flag[i])
		{
			pro_struct.tx_struct.re_tx_sec_counter[i] += past_sec;
		}
	}
	
	if(pro_struct.tx_lsnal_data_flag)
	{
		pro_struct.tx_lsnal_data_flag = FALSE;
		ProUpLsnalData();///上传盲区数据
	}
	
	if(pro_struct.tx_login_flag)
	{
		pro_struct.tx_login_flag = FALSE;
		MemCpy(udp_struct.udp_dst_ip,pro_struct.comm_para_ip,4);
		MemCpy(udp_struct.udp_dst_port,pro_struct.comm_para_port,2);
		UdpIpPortInit();
		ProUpLogin();
	}
	
	if(pro_struct.tx_struct.acc_on_tx_sec_counter >= 2*SYS_TASK_SEC_TIMER)
	{
		last_sec += past_sec;
		if(last_sec > 3)
		{
			last_sec = 0;
			ProGetFromAccOnBuf();
		}
	}
}
void ProLsnalSysExit(void)///系统退出盲区补偿
{
	uint8 i;
	uint16 len;
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		if(pro_struct.tx_struct.re_tx_full_flag[i])
		{
			pro_struct.tx_struct.re_tx_full_flag[i] = FALSE;
			ProPutIntoLsnal(pro_struct.tx_struct.re_tx_buf[i]+PRO_DATA_INDEX,
							pro_struct.tx_struct.re_tx_len[i]-PRO_DATA_INDEX-3,
							pro_struct.tx_struct.re_tx_buf[i][PRO_CMD_ID_INDEX]);
		}
	}
	
	for(i=0;i<PRO_MAX_TX_BUF_ARRAY;i++)
	{
		if(pro_struct.tx_struct.acc_on_tx_buf[i][0] == VALID_VAL_2A)
		{
			pro_struct.tx_struct.acc_on_tx_buf[i][0] = INVALID_VAL_FF;
			len = pro_struct.tx_struct.acc_on_tx_buf[i][2] << 8;
			len += pro_struct.tx_struct.acc_on_tx_buf[i][3];
			ProPutIntoLsnal(pro_struct.tx_struct.acc_on_tx_buf[i]+4,len,pro_struct.tx_struct.acc_on_tx_buf[i][1]);
		}
	}
}
/*****协议处理主函数******/
void ProProcess(uint8 data[],uint16 len)	//-开始处理V1.0的主协议
{
	uint16 tmp_len;
	
	#ifdef PRO_DEBUG
		uint8 str_len;
		char str_ch[20];
		uint16 i;

		str_len = sprintf(str_ch,"%s ","下行：");
		LocalDebug((uint8*)str_ch,str_len,LOCAL_PRO_DEBUG);	
		for(i=0;i<len;i++)
		{
			str_len = sprintf(str_ch,"%02X ",data[i]);
			LocalDebug((uint8*)str_ch,str_len,LOCAL_PRO_DEBUG);
		}
		LocalDebug("\r\n",2,LOCAL_PRO_DEBUG);
	#endif
	/**	
	if((len == 2)&&(data[0] == 0x00)&&(data[1] == 0x02))///心跳
	{
		goto RETURN_LAB;
	}
	**/
	tmp_len = data[0] >> 8;
	tmp_len += data[1];
	
	if((tmp_len == PRO_DATA_INDEX)&&(data[PRO_CMD_ID_INDEX] == PRO_DOWN_HEART_ID))
	{
		goto RETURN_LAB;
	}
	
	if((len < PRO_DATA_INDEX)||(len != tmp_len + 3))///长度不对，直接返回
	{
		return;
	}
	
	if((data[tmp_len] != XorCheck(data,tmp_len))||
	   (0x0d != data[tmp_len + 1])||
	   (0x0a != data[tmp_len + 2]))///校验,结束码判断
	{
		return;
	}
	
	len -= 3;	//-去掉结尾3个字符

	if(!MemCmp(data+PRO_TERM_ID_INDEX,sys_private_para_struct.terminal_id,TERMINAL_ID_LEN))///终端ID不对，直接返回
	{
		return;
	}

	switch(data[PRO_CMD_ID_INDEX])
	{
		case PRO_DOWN_COMM_PARA_ACK_ID:
		{
			ProDownCommParaAck(data,len);
			break;
		}
		case PRO_DOWN_ACK_ID:
		{
			ProDownAck(data,len);
			break;
		}
		
		case PRO_DOWN_POSITION_ID:
		{
			ProUpPosition();
			break;
		}
		/***
		case PRO_DOWN_TRACK_ID:
		{
			ProDownTrack(data,len);
			break;
		}
		***/
		case PRO_DOWN_QUERY_WORK_PARA_ID:
		{
			ProUpWorkPara();
			break;
		}
		case PRO_DOWN_SET_PARA_ID:
		{
			ProDownSetPara(data,len);
			break;
		}
		case PRO_DOWN_QUERY_PARA_ID:
		{
			ProDownParaQuery(data,len);
			break;
		}
		case PRO_DOWN_UPGRADE_ID:
		{
			ProDownUpgrade(data,len);
			break;
		}
		case PRO_DOWN_SET_WORK_PARA_ID:
		{
			ProDownSetWorkPara(data,len);
			break;
		}
		case PRO_DOWN_SET_CONTROLLER_PARA_ID:
		{
			ProDownSetControlPara(data,len);
			break;
		}
		default:
			break;
	}
	
RETURN_LAB:	
	sys_work_para_struct.lsnal_min_counter = 0;
	sys_work_para_struct.lsnal_sys_reset_min_counter = 0;
	sys_work_para_struct.lsnal_sys_comm_reset_min_counter = 0;
	sys_work_para_struct.lsnal_sys_must_reset_sec_counter = 0;
	pro_struct.no_rx_data_sec_counter = 0;
}
