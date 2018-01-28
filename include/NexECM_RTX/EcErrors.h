#ifndef   _EC_ERRORS_H_
#define   _EC_ERRORS_H_


#define ECERR_SUCCESS                 ( 0)  // Success, no error.

/* -----------------------------------------------------------------------------
 * Library error.
 * -----------------------------------------------------------------------------*/

typedef struct
{
	int  errorCode;
	char errorString[32];
} ERROR_CODE_INFO;

#define ECERR_WIN32_ERROR                      (-1)  // Win32 API return error.
#define ECERR_NIC_INDEX                        (-2)  // NIC index invaild
#define ECERR_MASTER_ID                        (-3)  // MasterId invaild
#define ECERR_IPC_TIMEOUT                      (-4)  // IPC timeout
#define ECERR_CREATE_SERVICE                   (-5)  // Only first instance can create remote service
#define ECERR_CREATE_SOCKET                    (-6)  // Create socket failed.
#define ECERR_SERVER_CONNECT_FAILED            (-7)  // Could not establish connection
#define ECERR_DRIVER_RESPONSE_TIMEOUT          (-8)  // Driver response timeout. 
#define ECERR_START_NETWORK                    (-9)  // Start network error, check error message.
#define ECERR_FILE_NOT_FOUND                   (-10) // File not found or File open failed.
#define ECERR_DATA_BUFFER_SIZE                 (-11) // Data buffer size invaild (too big).
#define ECERR_ESC_EEPROM_TIMEOUT               (-12) // ESC EEPROM access timeout
#define ECERR_ESC_EEPROM_BUSY                  (-13) // ESC EEPROM busy.
#define ECERR_ESC_EEPROM_ERROR                 (-14) // ESC EEPROM error.
#define ECERR_SLAVE_COUNT                      (-15) // Slave count invaild, slave count must > 0
#define ECERR_DATA_LEN                         (-16) // Data length invaild, length must > 0 or data lenth over limitation
#define ECERR_LOAD_NETWORK_DESC_FILE           (-17) // Open/load network file failed. Please check the path of ENI file.
#define ECERR_MASTER_ALREADY_CREATED           (-18) // Already create master, close it first.
#define ECERR_TOO_MUCH_INIT_CMD                (-19) // To much master init commands. 
#define ECERR_INIT_CMD_DATA_SIZE               (-20) // INIT command size to large.
#define ECERR_FILE_NAME_LEN                    (-21) // Configuration file name' length too long.
#define ECERR_DRIVER_NOT_FOUND                 (-22) // 1.Hardware not install, 2.Hardware driver not install, 3.Realtime driver not found / open failed
#define ECERR_WAIT_STATE_TIMEOUT               (-23) // Wait operation state timeout,  please check slave's current state.
#define ECERR_PROC_IMG_DATA_SIZE               (-24) // Proceess image data command size 
#define ECERR_CYCLIC_CONFIG                    (-25) // Cyclic info error in configuration file or master not support
#define ECERR_RTX_LIB_NOT_FOUND                (-26) // RTX Library not exist 
#define ECERR_LOAD_RTX_LIB                     (-27) // Load RTX library failed
#define ECERR_READ_ENI_ERROR                   (-28) // Parser ENI failed, Please check ENI file
#define ECERR_EC_NIC_NOT_FOUND                 (-29) // Scan NIC for ESC not found.
#define ECERR_RT_DATA_BUFFER                   (-30) // Data length to large, data buferr overflow.
#define ECERR_MASTER_TYPE_CHECK                (-31) // This API doesn't support target master (runtime)type
#define ECERR_VERSION_NOT_SUPPORTED            (-32) // The version of target system is too old, please update the software to support this API
#define ECERR_INVALID_INPUT                    (-33) // Invalid input, please check input value.
#define ECERR_DATA_BUFFER_TOO_LARGE            (-34) // The size of data buffer which specified pointer to is too large

#define ECERR_COE_MOTION_BASE                  (-1000)
#define ECERR_COE_AXES_INEFFICENT              (  -1 + ECERR_COE_MOTION_BASE ) // No available CoE axis resource, or try NEC_CoEMotionReset()
#define ECERR_COE_AXES_ID                      (  -2 + ECERR_COE_MOTION_BASE ) // Axis not found, Axis not initial.
#define ECERR_COE_STATE_UNKNOWN                (  -3 + ECERR_COE_MOTION_BASE ) // Axis not state unknow.
#define ECERR_COE_TIMEOUT                      (  -4 + ECERR_COE_MOTION_BASE ) // Coe State change timeout, Timeout 
#define ECERR_COE_STATE_DENY                   (  -5 + ECERR_COE_MOTION_BASE ) // Coe State deny, Check CoE slave is in Fault state or not.
#define ECERR_COE_OBJECT_NOT_SUPPORT           (  -6 + ECERR_COE_MOTION_BASE ) // Device not support this object, Please check index, subIndex, size. offset ...etc.
#define ECERR_COE_ACCESS_MAP_TYPE_TXPDO        (  -7 + ECERR_COE_MOTION_BASE ) // TxPDO cannot be write. Please check Object map type
#define ECERR_COE_ACCESS_MAP_TYPE_RXPDO        (  -8 + ECERR_COE_MOTION_BASE ) // RxPDO cannot be read. Please check Object map type
#define ECERR_COE_STATUS_CHECK_TIMEOUT         (  -9 + ECERR_COE_MOTION_BASE ) // Check status word (0x6041) timeout
#define ECERR_COE_NOT_PDO_TYPE                 ( -10 + ECERR_COE_MOTION_BASE ) // This object is not PDO type. (return from only for callback type functions)
#define ECERR_COE_EXCEPTION                    ( -11 + ECERR_COE_MOTION_BASE ) // Exception, 1.Not CiA402 standard, 2.hardware problems etc. 3. Not support! 
#define ECERR_COE_PDO_ENTRY_FULL               ( -12 + ECERR_COE_MOTION_BASE ) // Cannot add PDO entry because of buffer is full
#define ECERR_COE_HALT                         ( -13 + ECERR_COE_MOTION_BASE ) // The axis is halt, clear halt bit first. (ctrlWord bit 8)
#define ECERR_COE_FAULT                        ( -14 + ECERR_COE_MOTION_BASE ) // The axis in fault state, clear fault first. (ctrlWord bit7: 0->1)
#define ECERR_COE_HOME_INTERRUPTED             ( -15 + ECERR_COE_MOTION_BASE ) // Home procedure is interrupted or not start
#define ECERR_COE_HOME_ERROR                   ( -16 + ECERR_COE_MOTION_BASE ) // Home procedure error
#define ECERR_COE_SLAVE_TYPE_NOT_DEFINE        ( -17 + ECERR_COE_MOTION_BASE ) // The value of parameter "TypeOfSlave" not defined.


/* -----------------------------------------------------------------------------
_* Realtime system error.
 * -----------------------------------------------------------------------------*/
#define  ECEER_RT_BASE                         ( -     10000         )
#define  ECEER_RT_WIN32			               ( - 1 + ECEER_RT_BASE )  // REALTIME API return error. (Fatal error)
#define  ECEER_RT_MAC_ADDRESS                  ( - 2 + ECEER_RT_BASE )  // MAC address not exist in system.
#define  ECEER_RT_RX_SOCKET                    ( - 3 + ECEER_RT_BASE )  // Socket error when received data.
#define  ECEER_RT_NIC_INDEX                    ( - 4 + ECEER_RT_BASE )  // NIC index invaild or not found.
#define  ECERR_RT_INIT_CMD_OVERFLOW            ( - 5 + ECEER_RT_BASE )  // Master/Slave initial commands too many.
#define  ECERR_RT_RECV_QUEUE_OVERFLOW          ( - 6 + ECEER_RT_BASE )  // Receiving frame queue full.
#define  ECERR_RT_SEND_INFO_QUEUE_OVERFLOW     ( - 7 + ECEER_RT_BASE )  // Sending info queue full.
#define  ECERR_RT_PARAMETER_UNKNOWN            ( - 8 + ECEER_RT_BASE )  // Invaild parameter number 
#define  ECERR_RT_MASTER_NOT_INIT              ( - 9 + ECEER_RT_BASE )  // Master not exist (Check MasterID) or not initialed( Initial master first)
#define  ECERR_RT_RECV_TIMEOUT                 ( -10 + ECEER_RT_BASE )  // Receive EC frame (datagram) timeout
#define  ECERR_RT_ASYNC_COMMU_TIMEOUT          ( -11 + ECEER_RT_BASE )  // Asynchronous communication timeout
#define  ECERR_RT_SEND_FRAME_SIZE              ( -12 + ECEER_RT_BASE )  // EC frame size invaild.
#define  ECERR_RT_PARAMETER_VALUE              ( -13 + ECEER_RT_BASE )  // Invaild parameter value. 
#define  ECERR_RT_MASTER_INIT_CMD_FULL         ( -14 + ECEER_RT_BASE )  // Master init command is full.
#define  ECERR_RT_MASTER_STATE                 ( -15 + ECEER_RT_BASE )  // Master state unknown or cannot be accepted trasition state..
#define  ECERR_RT_CREATE_SLAVE                 ( -16 + ECEER_RT_BASE )  // cleate slave failed.
#define  ECERR_RT_SLAVE_INDEX                  ( -17 + ECEER_RT_BASE )  // Slave index invaild. or Slave address not found.
#define  ECERR_RT_UNEXCEPTED                   ( -18 + ECEER_RT_BASE )  // Internal error, call Nexcom staff.
#define  ECERR_RT_MEM_ACCESS_VIOLATION         ( -19 + ECEER_RT_BASE )  // Process image accees violation
#define  ECERR_RT_SLAVE_TYPE                   ( -20 + ECEER_RT_BASE )  // This slave does not have the functions. 
#define  ECERR_RT_DIO_RANGE                    ( -21 + ECEER_RT_BASE )  // DIO access range.
#define  ECERR_RT_NIC_INIT                     ( -22 + ECEER_RT_BASE )  // Network interface initial failed. No connection detect, Please check the 1.Connection of NIC port. 2.NIC driver is installed?
#define  ECERR_RT_STATE_DENY                   ( -23 + ECEER_RT_BASE )  // This function cannot be perform at current state. check state (or link status) when you access this function. 
#define  ECERR_RT_CMD_RETRY_TIMEOUT            ( -24 + ECEER_RT_BASE )  // Master's (Initial) command retry timeout. Please check 1.ENI files is correct?, 2.Slave connections is correct?
#define  ECERR_RT_SLAVE_NO_RESPONSE            ( -25 + ECEER_RT_BASE )  // Slave no response(WKC error), please check link status. 
#define  ECERR_RT_NIC_SEND                     ( -26 + ECEER_RT_BASE )  // NIC sending error, check NIC driver.
#define  ECERR_RT_NIC_RECV                     ( -27 + ECEER_RT_BASE )  // NIC receive error, check NIC driver.
#define  ECERR_RT_CTRL_OWNER                   ( -28 + ECEER_RT_BASE )  // Master already be started and control by other process
#define  ECERR_RT_ECM_CYCLE_TIME               ( -29 + ECEER_RT_BASE )  // Master cycle time invaild
#define  ECERR_RT_CLIENT_FULL                  ( -30 + ECEER_RT_BASE )  // Client is full
#define  ECERR_RT_CLIENT_PARAM                 ( -31 + ECEER_RT_BASE )  // Client parameter error
#define  ECERR_RT_SLAVE_CMD_RETRY_TIMEOUT      ( -32 + ECEER_RT_BASE )  // Slave's (Initial) command retry timeout.
#define  ECERR_RT_NIC_VERSION                  ( -33 + ECEER_RT_BASE )  // NIC driver version too old.
#define  ECERR_RT_INSERT_DATAGRAM              ( -34 + ECEER_RT_BASE )  // Internal error, call Nexcom help.
#define  ECERR_RT_SDO_DOWNLOA                  ( -35 + ECEER_RT_BASE )  // Download SDO failed. Note, Not includeing Slave abort
#define  ECERR_RT_SDO_UPLOAD                   ( -36 + ECEER_RT_BASE )  // Upload SDO failed. Note, Not includeing Slave abort
#define  ECERR_RT_SDO_NOT_READY                ( -37 + ECEER_RT_BASE )  // SDO isn't ready now. 
#define  ECERR_RT_WAIT_TIMEOUT                 ( -38 + ECEER_RT_BASE )  // Wait event timeout, Wait Master state change timeout!, Master cannot change state to INIT.
#define  ECERR_RT_SDO_UPLOAD_ABORT             ( -39 + ECEER_RT_BASE )  // Upload SDO failed. 
#define  ECERR_RT_SYSTEM_STATE_DENY            ( -40 + ECEER_RT_BASE )  // Realtime system not ready or Master service already started.
#define  ECERR_RT_TOTAL_SLAVE_MISMATCH         ( -41 + ECEER_RT_BASE )  // Online slave counts != ENI files slave counts, check ENI files or check the connections.
#define  ECERR_RT_PROBE_NUMBER_ERROR           ( -42 + ECEER_RT_BASE )  // invalid probe number.
#define  ECERR_RT_SLAVE_CMD_VALIDATE_TIMEOUT   ( -43 + ECEER_RT_BASE )  // Slave's (Initial) command vaildate check retry timeout.
#define  ECERR_RT_SDO_DOWNLOA_ABORT            ( -44 + ECEER_RT_BASE )  // Download SDO failed. Note, Not includeing Slave abort
#define  ECERR_RT_SDO_DATA_SIZE_FAIL           ( -45 + ECEER_RT_BASE )  // Sdo data size over expected size. 256 bytes
#define  ECERR_RT_RUNTIME_LICENSE              ( -46 + ECEER_RT_BASE )  // License not active
#define  ECERR_RT_PROHIBIT_IN_CALLBACK         ( -47 + ECEER_RT_BASE )  // This API can't use in call back. Or, the parameters not mapped to PDO are prohibited in callback.
#define  ECERR_RT_SDO_REQ_FAIL                 ( -48 + ECEER_RT_BASE )  // Wait sdo's request time out.
#define  ECERR_RT_SDO_RES_FAIL                 ( -49 + ECEER_RT_BASE )  // Wait sdo's response time out.
#define  ECERR_RT_RW_EEPROM_INVALID_INPUT      ( -50 + ECEER_RT_BASE )  // Maximun is 16 bytes in once rw.
#define  ECERR_RT_RW_EEPROM_FAIL               ( -51 + ECEER_RT_BASE )  // access eeprom fail, the eeprom data of slave failed are all zero.
#define  ECERR_RT_MASTER_ALREADY_RUNNINGE      ( -52 + ECEER_RT_BASE )  // The master runtime is in running. Only one master can run in the same time.
#define  ECERR_RT_ACCESS_REGISTER_TIMEOUT      ( -53 + ECEER_RT_BASE )  // Access slave register timeout.
#define  ECERR_RT_ACCESS_REGISTER_FAIL         ( -54 + ECEER_RT_BASE )  // Access slave register failed. Return wkc isn't correct.
#define  ECERR_RT_DATA_POINTER_INVALID         ( -55 + ECEER_RT_BASE )  // The pointer of data is invalid.
#define  ECERR_RT_WAIT_MBX_WRITTEN_BIT_TMEOUT  ( -56 + ECEER_RT_BASE )  // Wait mailbox input written bit timeout.
#define  ECERR_RT_CYCLIC_FRAME_FORMAT_INVALID  ( -57 + ECEER_RT_BASE )  // EC cyclic frame format invalid.
#define  ECERR_RT_DC_TIMEOUE                   ( -58 + ECEER_RT_BASE )  // Wait DC timeout.
#define  ECERR_RT_TOTAL_FRAMES_INVALID         ( -59 + ECEER_RT_BASE )  // The total number of frames is invalid. Up the cycle time of master to avoid this error.

typedef struct
{
	int  errorCode;
	char errorString[48];
} ERROR_INFO;

#define __STR(s) #s
#define __ECERR(_c_)  { _c_, __STR(_c_##\0) }
#define DECLARE_ERR_INFO_TABLE()\
ERROR_INFO ECERR_INFO_TABLE[] = {\
	 __ECERR(ECERR_SUCCESS)\
	,__ECERR(ECERR_WIN32_ERROR)\
	,__ECERR(ECERR_NIC_INDEX)\
	,__ECERR(ECERR_MASTER_ID)\
	,__ECERR(ECERR_IPC_TIMEOUT)\
	,__ECERR(ECERR_CREATE_SERVICE)\
	,__ECERR(ECERR_CREATE_SOCKET)\
	,__ECERR(ECERR_SERVER_CONNECT_FAILED)\
\
    /* RT Error */\
	,__ECERR(ECEER_RT_WIN32)\
	,__ECERR(ECEER_RT_MAC_ADDRESS)\
	,__ECERR(ECEER_RT_RX_SOCKET)\
	,__ECERR(ECEER_RT_NIC_INDEX)\
	,__ECERR(ECERR_RT_INIT_CMD_OVERFLOW)\
	,__ECERR(ECERR_RT_RECV_QUEUE_OVERFLOW)\
	,__ECERR(ECERR_RT_SEND_INFO_QUEUE_OVERFLOW)\
	,__ECERR(ECERR_RT_PARAMETER_UNKNOWN)\
	,__ECERR(ECERR_RT_MASTER_NOT_INIT)\
	,__ECERR(ECERR_RT_RECV_TIMEOUT)\
	,__ECERR(ECERR_RT_ASYNC_COMMU_TIMEOUT)\
	,__ECERR(ECERR_RT_SEND_FRAME_SIZE)\
	,__ECERR(ECERR_RT_PARAMETER_VALUE)\
};

#define ERROR_TEXT_INSERT_MASTER_INIT_CMD_DATAGRAM        "Fatal error! Insert master init cmd datagram failed"
#define ERROR_TEXT_RESET_ESC_PROCESS                      "Reset ESC process failed"
#define ERROR_TEXT_CYCLIC_WKC                             "WKC of cyclic command error"
#define ERROR_TEXT_RECV_MASTER_INIT_CMD_TIMEOUT           "Receive master's init command timeout"
#define ERROR_TEXT_MASTER_STD_INIT_CMD                    "Master send std.initCmd failed"
#define ERROR_TEXT_SLAVE_STATE_ERROR                      "Slave state error."

#endif // _EC_ERRORS_H_