//////////////////////////////////////////////////////////////////
//
// Sample2AxesControl_RTX.h - header file
//
// This file was generated using the RTX 2016 SDK 12.0.0.1324
// RTX Application Template.
//
// Created: 5/20/2016 2:30:58 PM 
// User: NET3640
//
//////////////////////////////////////////////////////////////////
#pragma once
//This define will deprecate all unsupported Microsoft C-runtime functions when compiled under RTSS.
//If using this define, #include <rtapi.h> should remain below all windows headers
//#define UNDER_RTSS_UNSUPPORTED_CRT_APIS

//#include <SDKDDKVer.h>
//#include <windows.h>
//#include <tchar.h>
//#include <rtapi.h>    // RTX APIs that can be used in real-time and Windows applications.

//#include "NexECM_RTX/nex_type.h"
//#include "NexECM_RTX/EcErrors.h"
//#include "NexECM_RTX/NexECMRtx.h"
//#include "NexECM_RTX/NexCoeMotion.h"

#include <cstddef>
#define TOTAL_AXES        (6)     // 伺服电机轴数，设置为6
#define FIRST_AXIS_ADDR   (0)	  // 第一个伺服电机轴的偏移索引，设置为0
#define CYCLETIME		  (1)    // 控制周期，设置为1ms

//static  PVOID  location;

//定义与C#进程通信的共享内存结构体
typedef	struct
{
	void* axes[TOTAL_AXES];
	int masterId;
	double actualPos[TOTAL_AXES];
	 double targetPos[TOTAL_AXES];
	double actualPosPCS[TOTAL_AXES];
	double targetPosPCS[TOTAL_AXES];
	double middlePosPCS[TOTAL_AXES];
	double acceleration;
	double velocity;
size_t	 targetReached;
size_t	 cmdAxisIndex;
size_t	fullCircleFlag;
size_t	 inputIO;
size_t	 outputIO;
size_t	outputIOChanged;
size_t	startIOcontrol;
}USER_DAT;

enum COORD_SYS
{
	ACS = 0,    // 关机坐标系
	PCS = 1     // 笛卡尔直角坐标系
};

