#ifndef __INCLUDE_H
#define __INCLUDE_H   
	#include "stm32f10x.h"
	#include "stm32f10x_conf.h"
	#define false FALSE	
	#define true TRUE
	#define NULL 0
	
	#define ENABLE_WATCHDOG
//	#define SYS_PARA_INIT
	#define PPP_DEBUG
	#define PRO_DEBUG
	#define GPS_DEBUG
	#define GPRS_DEBUG
	
	
	
	#define LEN_12 12
	#define LEN_16 16
	#define LEN_20 20
	#define LEN_32 32
	#define LEN_64 64

	#include <time.h>
	#include <stdio.h>
	#include <string.h>

	#include "driver.h"
	#include "gps.h"
	#include "gprs.h"
	#include "syspara.h"
	#include "selfdef.h"
	#include "sim.h"
	#include "ppp.h"
	#include "protocol.h"
	#include "fram.h"
	#include "flash.h"
	#include "systask.h"
	#include "ftp.h"
	#include "sms.h"
	#include "stm32f10x_it.h"
	#include "can.h"
	#include "localcomm.h"
#endif

