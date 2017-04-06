#define GPS_STRUCT_GLOBAL

#include "include.h"


///1节=1海里/小时=1.852千米/小时

uint8 GpsGetTime(uint8 utc_time[],uint8 hex_time[])
{
	uint8 i,res,u8_val;
	
	res = IsValidNum(utc_time,6);
	if(!res)
	{
		goto RETURN_LAB;
	}
	
	for(i=0;i<3;i++)
	{
		u8_val = (utc_time[i*2] - '0')*10;
		u8_val += utc_time[i*2+1] - '0';
		hex_time[i] = u8_val;
	}
	
	if((hex_time[0] >= 24)||(hex_time[1] >= 60)||(hex_time[2] >= 60))
	{
		res = FALSE;
	}
	else
	{
		res = TRUE;
	}
RETURN_LAB:
	return res;
}
uint8 GpsGetDate(uint8 utc_date[],uint8 hex_date[])
{
	uint8 i,u8_val,res;
	
	res = IsValidNum(utc_date,6);
	if(!res)
	{
		goto RETURN_LAB;
	}
	
	for(i=0;i<3;i++)
	{
		u8_val = (utc_date[i*2] - '0')*10;
		u8_val += utc_date[i*2+1] - '0';
		hex_date[2-i] = u8_val;
	}
	
	if((hex_date[0] < 14 )||
	   (hex_date[0] >= 99)||
	   (hex_date[1] > 12 )||
	   (hex_date[1] < 1 )||
	   (hex_date[2] > 31)||
	   (hex_date[2] < 1))///年，月，日
	{
		res = FALSE;
	}
	else
	{
		res = TRUE;
	}
RETURN_LAB:
	return res;
}

uint8 GpsCharFloatDataToHex(uint8 gps_data[],uint8 len,uint8 type,uint32 *r_val)///GPS字符浮点数转十六进制
{
	uint8 i,j,k,m,res;
	uint32 u32_val = 0,u32_val_div = 0;
	
	res = IsValidCharFloatNum(gps_data,len);
	if(!res)
	{
		goto RETURN_LAB;
	}
	
	for(i=0;i<len;i++)
	{
		if(gps_data[i] != '.')
		{
			u32_val *= 10;
			u32_val += gps_data[i] - 0x30;
			
		}
		else
		{
			j = len - i - 1;///小数点后的位数
			break;
		}
	}
	
	i++;
	if((type == LAT_TYPE)||(type == LONG_TYPE))///经纬度取小数点后5位
	{
		k = 0;
		if(j > 5)
		{
			j = 5;
		}
		else
		{
			if(j < 5)
			{
				k = 5 - j;///不足5位，补0个数
			}
		}
	}
	
	if((type == LAT_TYPE)||(type == LONG_TYPE))
	{
		u32_val_div = u32_val % 100;
		u32_val /= 100;
		u32_val *= 100;
		for(m=0;m<j;m++)///小数
		{
			u32_val *= 10;
			u32_val_div *= 10;
			u32_val_div += gps_data[i++] - 0x30;
		}
		
		for(m=0;m<k;m++)
		{
			u32_val *= 10;
			u32_val_div *= 10;
		}
		u32_val_div += 5;///l四舍五入
		
		u32_val_div = u32_val_div * 5;
		u32_val_div /= 3;
	}
	else
	{
		for(m=0;m<j;m++)///小数
		{
			u32_val *= 10;
			u32_val += gps_data[i++] - 0x30;
		}
	}

	if(type == LAT_TYPE)
	{
		u32_val += u32_val_div;
		u32_val /= 10;///精度，小数点后4位
		if(u32_val > 90000000)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == LONG_TYPE)
	{
		u32_val += u32_val_div;
		
		u32_val /= 10;
		if(u32_val > 180000000)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == SPEED_TYPE)
	{		
		for(i=0;i<j;i++)///取整
		{
			u32_val /= 10;
		}
		
		u32_val *= 1852;
		u32_val /= 1000;
		
		if(u32_val > 255)///节转KM/H
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == DIR_TYPE)
	{
		for(i=0;i<j;i++)///取整
		{
			u32_val /= 10;
		}
		
		u32_val /= 2;///1代表2度
		
		if(u32_val > 180)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == AMP_TYPE)
	{
		for(i=0;i<j;i++)///取整
		{
			u32_val /= 10;
		}
		
		if(u32_val > 9999)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else
	{
		res = FALSE;
		goto RETURN_LAB;
	}
	
	*r_val = u32_val;
	res = TRUE;
RETURN_LAB:
	return res;
}
void GpsAdd8Hour(uint8 d_t[])
{
	if(d_t[3] < 16)
	{
		d_t[3] += 8;
	}
	else
	{
		d_t[3] += 8;
		d_t[3] = d_t[3] % 24;
		
		d_t[2] += 1;
		if(d_t[2] > 28)
		{
			switch(d_t[1])
			{
				case 1:
				case 3:
				case 5:
				case 7:
				case 8:
				case 10:
				{
					if(d_t[2] > 31)
					{
						d_t[2] = 1;
						d_t[1] += 1;
					}
					break;
				}
				case 2:
				{
					if((d_t[0] % 4 == 0)&&(d_t[0] % 100 != 0))
					{
						if(d_t[2] > 29)
						{
							d_t[2] = 1;
							d_t[1] += 1;
						}
					}
					else
					{
						if(d_t[2] > 28)
						{
							d_t[2] = 1;
							d_t[1] += 1;
						}
					}
					break;
				}
				case 4:
				case 6:
				case 9:
				case 11:
				{
					if(d_t[2] > 30)
					{
						d_t[2] = 1;
						d_t[1] += 1;
					}
					break;
				}
				case 12:
				{
					if(d_t[2] > 31)
					{
						d_t[2] = 1;
						d_t[1] = 1;
						d_t[0] += 1;
					}
					break;
				}
			}
		}
	}
}
void GpsGetGprmcGpggaInfo(uint8 gps_data[],uint8 len,uint8 type)///获取GPRMC数据
{
	uint8 i,j,dot_index[9],t_d[6],res,res_1,res_2;
	uint8 lat_len,long_len,speed_len,dir_len,sat_len,amp_len;
	uint16 tmp_amp_val;
	uint32 lat_val,long_val,speed_val,dir_val,sat_val,amp_val;
	
	#ifdef GPS_DEBUG
		char str_ch[256];
		uint8 str_len;
	#endif
	
	i = 0;j = 0;
	while((i<len)&&(j<11))
	{
		if(gps_data[i] == ',')
		{
			dot_index[j++] = i;
		}
		i++;
	}
	
	if(j != 11)
	{
		goto RETURN_LAB;
	}
	
	if(type == RMC_TYPE)
	{
		if((gps_data[0] != ',')&&(dot_index[0] >= 6)&&
		   (gps_data[dot_index[7]+1] != ',')&&((dot_index[8]-dot_index[7]) >= 6))///提取日期时间
		{
			res_1 = GpsGetTime(gps_data,t_d+3);
			res_2 = GpsGetDate(gps_data+dot_index[7]+1,t_d);
			
			if(res_1 && res_2)
			{
				GpsAdd8Hour(t_d);
				
				MemCpy(sys_work_para_struct.date_time,t_d,6);
				
				if(sys_misc_run_struct.gps_need_datetime_check_flag)
				{	
					sys_misc_run_struct.gps_need_datetime_check_flag = FALSE;
					RtcSetCalendarTime();
				}
				
				
				
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"%02d年%02d月%02d日 %02d时%02d分%02d秒\t",t_d[0],t_d[1],t_d[2],t_d[3],t_d[4],t_d[5]);
					LocalDebug((uint8*)str_ch,str_len,LOCAL_GPS_DEBUG);
				#endif
			}
		}
		
		if(type == RMC_TYPE)
		{
			if(gps_data[dot_index[0]+1] != 'A')
			{
				OFF_GPS_LED();
				sys_work_para_struct.term_run_status_word &= ~STATE_A_GPS;
			}
			else
			{
				sys_work_para_struct.term_run_status_word |= STATE_A_GPS;
				ON_GPS_LED();
			}
		}
	
	
		if(!(sys_work_para_struct.term_run_status_word & STATE_A_GPS))
		{
			goto RETURN_LAB;
		}
		
		lat_len = dot_index[2]-dot_index[1]-1;
		long_len = dot_index[4]-dot_index[3]-1;
		if((gps_data[dot_index[1]+1] != ',')&&(lat_len > 0)&&
		   (gps_data[dot_index[3]+1] != ',')&&(long_len > 0))
		{
			if(((gps_data[dot_index[2]+1] == 'N')||(gps_data[dot_index[2]+1] == 'S'))&&
			   ((gps_data[dot_index[4]+1] == 'E')||(gps_data[dot_index[4]+1] == 'W')))///提取纬度与经度
			{
				res_1 = GpsCharFloatDataToHex(gps_data+dot_index[1]+1,lat_len,LAT_TYPE,&lat_val);
				res_2 = GpsCharFloatDataToHex(gps_data+dot_index[3]+1,long_len,LONG_TYPE,&long_val);
				
				if(res_1 && res_2)
				{
					if(gps_data[dot_index[2]+1] == 'N')
					{
						sys_work_para_struct.term_run_status_word |= STATE_N_LAT;
					}
					else
					{
						sys_work_para_struct.term_run_status_word &= ~STATE_N_LAT;
					}
					
					if(gps_data[dot_index[4]+1] == 'E')
					{
						sys_work_para_struct.term_run_status_word |= STATE_E_LONG;
					}
					else
					{
						sys_work_para_struct.term_run_status_word &= ~STATE_E_LONG;
					}
					
					sys_data_struct.gps_info_of_gprs[LAT_INDEX]   = lat_val >> 24;
					sys_data_struct.gps_info_of_gprs[LAT_INDEX+1] = lat_val >> 16;
					sys_data_struct.gps_info_of_gprs[LAT_INDEX+2] = lat_val >> 8;
					sys_data_struct.gps_info_of_gprs[LAT_INDEX+3] = lat_val;
					
					sys_data_struct.gps_info_of_gprs[LONG_INDEX]   = long_val >> 24;
					sys_data_struct.gps_info_of_gprs[LONG_INDEX+1] = long_val >> 16;
					sys_data_struct.gps_info_of_gprs[LONG_INDEX+2] = long_val >> 8;
					sys_data_struct.gps_info_of_gprs[LONG_INDEX+3] = long_val;
					#ifdef GPS_DEBUG
						str_len = sprintf(str_ch,"%ld%c %ld%c\t",lat_val,gps_data[dot_index[2]+1],long_val,gps_data[dot_index[4]+1]);
						LocalDebug((uint8*)str_ch,str_len,LOCAL_GPS_DEBUG);
					#endif
					
				}
			}
		}
		
		speed_len = dot_index[6]-dot_index[5]-1;
		if((gps_data[dot_index[5]+1] != ',')&&(speed_len > 0))///提取速度
		{
			res = GpsCharFloatDataToHex(gps_data+dot_index[5]+1,speed_len,SPEED_TYPE,&speed_val);
			if(res)
			{
				sys_data_struct.gps_info_of_gprs[SPEED_INDEX]   = speed_val;
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"速度：%03d\t",speed_val);
					LocalDebug((uint8*)str_ch,str_len,LOCAL_GPS_DEBUG);
				#endif
			}
		}
		
		dir_len = dot_index[7]-dot_index[6]-1;
		if((gps_data[dot_index[6]+1] != ',')&&(dir_len > 0))///提取航向
		{
			res = GpsCharFloatDataToHex(gps_data+dot_index[6]+1,dir_len,DIR_TYPE,&dir_val);
			if(res)
			{
				sys_data_struct.gps_info_of_gprs[DIR_INDEX]   = dir_val;
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"航向：%03d 度\t",dir_val*2);
					LocalDebug((uint8*)str_ch,str_len,LOCAL_GPS_DEBUG);
				#endif
			}
		}
	}
	else if(type == GGA_TYPE)
	{
		sat_len = dot_index[6]-dot_index[5]-1;
		if((gps_data[dot_index[5]+1] != ',')&&(sat_len > 0)&&(sat_len <= 2))///提取卫星个数
		{
			res = IsValidNum(gps_data+dot_index[5]+1,sat_len);
			if(res)
			{
				sat_val = 0;
				for(i=0;i<sat_len;i++)
				{
					sat_val *= 10;
					sat_val += gps_data[dot_index[5]+1+i] - 0x30;
				}
				sys_data_struct.gps_info_of_gprs[SAT_INDEX] = sat_val;
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"可用卫星数: %02d\t",sat_val);
					LocalDebug((uint8*)str_ch,str_len,LOCAL_GPS_DEBUG);
				#endif
			}
		}
		else
		{
			sys_data_struct.gps_info_of_gprs[SAT_INDEX] = 0;
		}
		
		amp_len = dot_index[8]-dot_index[7]-1;
		if((gps_data[dot_index[7]+1] != ',')&&(amp_len > 0))///提取海拔
		{
			tmp_amp_val = 0x0000;
			i = 1;
			if(gps_data[dot_index[7]+1] == '-')
			{
				tmp_amp_val = 0x8000;
				i = 2;
				amp_len -= 1;
			}
			res = GpsCharFloatDataToHex(gps_data+dot_index[7]+i,amp_len,AMP_TYPE,&amp_val);
			if(res)
			{
				amp_val = tmp_amp_val | amp_val;
				sys_data_struct.gps_info_of_gprs[AMP_INDEX]   = amp_val >> 8;
				sys_data_struct.gps_info_of_gprs[AMP_INDEX+1] = amp_val;
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"海拔: %d\r\n",amp_val);
					LocalDebug((uint8*)str_ch,str_len,LOCAL_GPS_DEBUG);
				#endif
			}
		}
	}
RETURN_LAB:
	return;
}

uint8 GpsFrameCheck(uint8 gps_data[],uint8 len)
{
	uint8 i,res = FALSE;
	uint8 gps_check,cal_check;
	
	i = 0;
	cal_check = gps_data[i++];
	
	for(i=1;i<len-3;i++)
	{
		cal_check ^= gps_data[i];
	}

	gps_check = AsciiToHexVal(gps_data[len-2],gps_data[len-1]);
	
	if(gps_check == cal_check)
	{
		res = TRUE;
	}

	return res;
}
uint8 GetGpsInfo(uint8 data[])
{
	uint32 tmp_status;
	
	MemCpy(data,sys_data_struct.gps_info_of_gprs,12);
	
	tmp_status = sys_work_para_struct.term_run_status_word;
	tmp_status &= STATE_MASK_BITS;
	MemCpy(data+12,(uint8*)&tmp_status,4);
	RtcGetCalendarTime(data+16);

	return GPS_INFO_LEN;
}
void GpsMain(void)
{
	uint8 res,rx_data[GPS_UART_BUF_LEN],rx_gprmc_flag = FALSE;
	uint16 mat_index,rx_len,i,j;
	
	static uint16 sys_rtc_pre_val=0;
	static uint16 sys_rtc_cur_val=0;
	uint16 past_sec_val;
	
	sys_rtc_cur_val = sys_misc_run_struct.sys_rtc_sec_counter;///获取系统rtc sec计数；
	past_sec_val = (sys_rtc_cur_val + (65535 - sys_rtc_pre_val)) % 65535;
	
	rx_len = GetGpsUartRxData(rx_data);

	if(rx_len > 0)
	{
		sys_rtc_pre_val = sys_rtc_cur_val;
		#ifdef GPS_DEBUG
			LocalDebug(rx_data,rx_len,LOCAL_GPS_DEBUG);
		#endif
		
		i = 0;
		j = 0;
		while(rx_len > 3)///除去$,CH,CR三个字符
		{	
			while((rx_data[i] != '$')&&(i<rx_len))
			{
				i++;
			}
			if(i == rx_len)
			{
				break;
			}
			
			j=0;
			while((rx_data[i+j] != 0x0a)&&((i+j)<rx_len))
			{
				j++;
			}
			if((i+j) == rx_len)
			{
				break;
			}
			
			res = GpsFrameCheck(rx_data+i+1,j-2);///帧校验
			
			if(res)
			{
				gps_struct.gps_rx_right_data_flag = TRUE;
				mat_index = SubMatch("$GPRMC,",StrLen("$GPRMC,",0),rx_data+i,j);
				if(mat_index > 0)
				{
					gps_struct.sms_gprmc_ack[0] = VALID_VAL_2A;
					MemCpy(gps_struct.sms_gprmc_ack+1,rx_data+i,j-1);
					rx_gprmc_flag = TRUE;
					gps_struct.sms_gprmc_ack[j] = '\0';
					GpsGetGprmcGpggaInfo(rx_data+i+mat_index,j-mat_index,RMC_TYPE);///GPRMC,GPGGA处理
				}
				if(!rx_gprmc_flag)
				{
					gps_struct.sms_gprmc_ack[0] = '\0';
				}
				if(mat_index > 0)
				{
					i += j;
					continue;
				}
				mat_index = SubMatch("$GPGGA,",StrLen("$GPGGA,",0),rx_data+i,j);
				if(mat_index > 0)
				{
					GpsGetGprmcGpggaInfo(rx_data+i+mat_index,j-mat_index,GGA_TYPE);///GPRMC,GPGGA处理
					break;///GGA处理后，直接结束
				}
			}
			i += j;
		}
	}
	else
	{
		if(past_sec_val > 10)///大于10秒,未收到数据,清GPS定位标志
		{
			sys_work_para_struct.term_run_status_word &= ~STATE_A_GPS;
			sys_misc_run_struct.gps_need_datetime_check_flag = TRUE;
			gps_struct.sms_gprmc_ack[0] = '\0';
			OFF_GPS_LED();
		}
	}
}

