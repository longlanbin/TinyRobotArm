#pragma once

#include <string>
#include <map>
#include <iostream>

//状态(错误)ID和描述类型
typedef size_t ErrorID;
typedef std::string ERROR_DESCRIPTION;

#ifndef RBTINTERF_NO_ERRORQUERY

#define PRINT_ERROR(e) StatQuery.Print(std::cerr, e)
#define GET_ERROR_DESCRIPT(e) (StatQuery.Msg(e))

/*****************************************************************************/
/*! Errorcode to Error description Query (english only)                */
/*****************************************************************************/
/**
* @class	RbtInterfStatusQuery RbtInterfStatus.h
* @brief	用于状态(错误)ID和描述类型的映射查询
*/
class RbtInterfStatusQuery
{
public:
	RbtInterfStatusQuery();
	void Print(std::ostream &os, ErrorID sid)
	{
		if (m_Status.find(sid) != m_Status.end())
		{
			switch ((sid & 0xD0000000L) >> 30)
			{
			case 0:
				break;
			case 1:
				os << "RobotInterface: Informational: " << m_Status[sid] << std::endl;
				break;
			case 2:
				os << "RobotInterface: Warning: " << m_Status[sid] << std::endl;
				break;
			case 3:
				os << "RobotInterface: Error: " << m_Status[sid] << std::endl;
				break;
			default:
				break;
			}
		}
		else
		{
			os << "RototInterface: the Status/Error ID " << sid << " is not exist" << std::endl;
		}
	}
	ERROR_DESCRIPTION Msg(ErrorID sid)
	{
		ERROR_DESCRIPTION msg;
		if (m_Status.find(sid) != m_Status.end())
		{
			switch ((sid & 0xD0000000L) >> 30)
			{
			case 0:
				break;
			case 1:
				msg = "RobotInterface: Informational: " + m_Status[sid];
				break;
			case 2:
				msg = "RobotInterface: Warning: " + m_Status[sid];
				break;
			case 3:
				msg = "RobotInterface: Error: " + m_Status[sid];
				break;
			default:
				break;
			}
		}
		else
		{
			msg = "\r\n";
		}
		return msg;
	}
private:
	std::map<ErrorID, ERROR_DESCRIPTION> m_Status;
};
extern RbtInterfStatusQuery StatQuery; /**< @brief 用于状态(出错)信息查询 */
#endif

									   /*******************************************************************************
									   * RobotInterface Library Errors
									   *******************************************************************************/
									   /*  */
									   /*   Values are 32 bit values laid out as follows: */
									   /*    3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1 */
									   /*    1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 */
									   /*   +---+-+-+-----------------------+-------------------------------+ */
									   /*   |Sev|C|R|     Facility          |               Code            | */
									   /*   +---+-+-+-----------------------+-------------------------------+ */
									   /*  */
									   /*   where */
									   /*  */
									   /*       Sev - is the severity code */
									   /*  */
									   /*           00 - Success */
									   /*           01 - Informational */
									   /*           10 - Warning */
									   /*           11 - Error */
									   /*  */
									   /*       C - is the Customer code flag */
									   /*  */
									   /*       R - is a reserved bit */
									   /*  */
									   /*       Facility - is the facility code */
									   /*  */
									   /*           0x001 - config file */
									   /*           0x002 - plcopen */
									   /*           0x003 - kine */
									   /*           0x004 - interplation */
									   /*           0x005 - drivers */
									   /*  */
									   /*       Code - is the facility's status code */
									   /*  */

									   //------------------------------ SUCCESS --------------------------------
									   /*  */
									   /*  MessageId: RBTINTERF_NO_ERROR */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  No Error */
									   /*  */
#define RBTINTERF_NO_ERROR                     ((size_t)0x20000000L)
									   //-----------------------------------------------------------------------

									   //                           < CONFIG FILE >
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
									   /*  */
#define RBTINTERF_NO_CONFIG_TXT                ((size_t)0xE0010001L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define CONFIG_TXT_INVALID_FORMAT             ((size_t)0xE0010002L)
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define RBTINTERF_NO_MODEL_FILE                ((size_t)0xE0010003L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */

#define DH_CFG_FILE_INVALID_FORMAT             ((size_t)0xE0010004L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define DH_CFG_FILE_DATA_INVALID               ((size_t)0xE0010005L)


									   //                              < PLCOPEN >
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_NULL_POINTER                   ((size_t)0xE0020002L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_ADD_AXIS_UNDER_CURRENT_GROUPSTATE ((size_t)0xE0020002L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_ADD_AXIS               ((size_t)0xE0020003L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_REMOVE_AXIS_UNDER_CURRENT_GROUPSTATE ((size_t)0xE0020004L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_MOVE_INSTRUCT_ARC_COLLINEATION         ((size_t)0xE0020014L)



#define PLCOPEN_FAIL_TO_REMOVE_AXIS            ((size_t)0xE0020005L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_UNGROUP_UNDER_CURRENT_GROUPSTATE ((size_t)0xE0020006L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_ENABLEGROUP_UNDER_CURRENT_GROUPSTATE ((size_t)0xE0020007L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_DISABLEGROUP_UNDER_CURRENT_GROUPSTATE ((size_t)0xE0020008L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_ENABLEGROUP            ((size_t)0xE0020009L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_FAIL_TO_DISABLEGROUP           ((size_t)0xE002000AL)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_READ_POS_INSTRUCT_ARG_ERROR     ((size_t)0xE002000BL)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_MOVE_INSTRUCT_INVOKE_ERROR      ((size_t)0xE002000CL)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_MOVE_INSTRUCT_ARG_ERROR         ((size_t)0xE002000DL)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_NO_MOVE_IS_NEEDED               ((size_t)0xA002000EL)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_MOVE_INSTRUCT_CALC_ERROR         ((size_t)0xE002000FL)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_MOVE_INSTRUCT_NOT_APPLICABLE      ((size_t)0xA0020010L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_MOVE_INSTRUCT_LIST_NOT_EMPTY      ((size_t)0xA0020011L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_POSITION_NOT_REACHABLE            ((size_t)0xA0020012L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define PLCOPEN_NO_DOUBLE_JOG                     ((size_t)0xA0020013L)

									   //                              < KINEMATICS >
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define FORWARD_KIN_TRANS_ARG_ERROR             ((size_t)0xE0030001L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define FORWARD_KIN_ARG_ERROR                  ((size_t)0xE0030002L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define INVERSE_KIN_TRANS_ARG_ERROR            ((size_t)0xE0030003L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define INVERSE_KIN_ARG_ERROR                  ((size_t)0xE0030004L)

									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define INVERSE_KIN_NOT_REACHABLE              ((size_t)0xE0030005L)

									   //                               < Interpolation >
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define JOINT_INTERPLATE_ARG_ERROR             ((size_t)0xE0040001L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define LINE_INTERPLATE_ARG_ERROR             ((size_t)0xE0040002L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define ARC_INTERPLATE_ARG_ERROR             ((size_t)0xE0040003L)
#define WELDING_INTERPLATE_ARG_ERROR             ((size_t)0xE0040004L)

									   //                               < DRIVERS >
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define POINT_NOT_ENUOGH_ERROR               ((size_t)0xE0040005L)

									   /* NURBS给定的行径点太少 */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define TATGET_VEC_EXCEED_LIMIT               ((size_t)0xE0040006L)

									   /* NURBS速度设置过大 */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define DRVS_OPRERATION_MODE_DIFFER             ((size_t)0xE0050001L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define DRVS_NOT_ALL_READY                      ((size_t)0xE0050002L)
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define DRVS_NOT_ALL_POWER_ON                   ((size_t)0xE0050003L)

									   //                                < CRobot >
									   /*  */
									   /*  MessageId:  */
									   /*  */
									   /*  MessageText: */
									   /*  */
									   /*  */
									   /*  */
#define CROBOT_MODEL_NULL                        ((size_t)0xE0060001L)

#define CROBOT_S3_MARSTER_DISABLE                ((size_t)0xE0060002L)

#define THE_SAME_POSITION                        ((size_t)0xE0060003L)

#define CROBOT_SYNC_NOT_CALIB_DISABLE            ((size_t)0xE0060004L)

#define IO_STATE_ERROR                           ((size_t)0xE0070001L)

#define REPRODUCING_OPERATION_ERROR              ((size_t)0xE0070002L)
#define Switch_On_Disabled                       ((size_t)0xE0070003L)

#define PLCOpen_State_Error                      ((size_t)0xE0070004L)
#define NO_MATCHED_PARAMETER                      ((size_t)0xE0070005L)
									   //直角坐标超限
#define RANGE_OVERLIMIT_X						((size_t)0xE0080001L)	//x轴超限；
#define RANGE_OVERLIMIT_Y						((size_t)0xE0080002L)	//y轴超限；
#define RANGE_OVERLIMIT_Z						((size_t)0xE0080003L)	//z轴超限；
#define RANGE_NOOVERLIMIT						((size_t)0xE0080004L)	//无超限

#define SPEED_NOOVERLIMIT                         ((size_t)0xE0080005L)	//无超速；
#define SPEED_OVERLIMIT                         ((size_t)0xE0080006L)	//超速；
									   //------------------------------ WARNING ------------------------------
