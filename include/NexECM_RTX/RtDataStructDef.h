#ifndef __RT_DATA_STRUCT_H
#define __RT_DATA_STRUCT_H

//
// Paramenter number definition 
// 
#define NEC_PARA_S_ECM_CYCLETIMEUS      (  0) // EcMaster TxRx cycletime. [1~1000000]
#define NEC_PARA_S_NIC_INDEX            (  1) // NicIndex. [0 ~ I32_T_MAX]
#define NEC_PARA_S_FRAME_TYPE           (  2) // FrameType [0:UDP, 1:ECAT_UDP, 2:ECAT] 
#define MAX_RT_PARAMETERS               (  3)

#define NEC_PARA_ECM_RECV_TIMEOUT_US    (100) // [ MasterCycleTime ~ 2000000 ], if value not in range, auto fit.
#define NEC_PARA_ECM_LINK_ERR_MODE      (101) // Link error mode. 0: Auto re-connect(default), 1:manual re-connect, 2:Stop network.
#define NEC_PARA_ECM_DC_CYC_TIME_MODE   (102) // Dc cycle time mode.[0:Follow master cycle time, 1:Use ENI value.], default:0
#define NEC_PARA_ECM_COE_INIT_CMD_FAIL_MODE (103) // Coe init command error mode. 0:bypass, 1:stop start network process.
//
// Parameter value definition
//

// For parameter: NEC_PARA_ECM_LINK_ERR_MODE
#define LINKERR_AUTO        (0)
#define LINKERR_MANUAL      (1)
#define LINKERR_STOP        (2)
//*******************************************************************************
#define MAX_NICI_STR  (128)
typedef struct
{ // Define nic information for UI
	char   Name[MAX_NICI_STR];
	char   Description[MAX_NICI_STR];
	U8_T   Mac[6];
} NICI;

typedef struct
{ //This structure read static information
	U32_T vendorId;
	U32_T productCode;
	U32_T revisionNo;
	U32_T serialNo;
	U8_T  escType;           // [out] esc type,     register offset 0x0000
    U8_T  escRevision;       // [out] esc revision, register offset 0x0001
    U16_T escBuild;          // [out] esc build,    register offset 0x0002:0x0003
	U8_T  escFmmuCnt;        // [out] esc build,    register offset 0x0004
	U8_T  escSmCnt;          // [out] esc build,    register offset 0x0005
	U8_T  escPrecessRamSize; // [out] esc build,    register offset 0x0006
	U8_T  escPortDesc;       // [out] esc build,    register offset 0x0007
} ECSI;

typedef struct
{
	U32_T FrameSendCount;       // How many frame been send when start network, will be reset when start.
	U32_T FrameRecvCount;       // How many frame recv when start network, will be reset when start.
	U32_T ErrFrameLostCount;    // When detect a packet lost will ++. Can be reset manually, be reset when start
	U32_T ErrRecvUnknownFrame;  // Receiving an unknow frame will ++. Can be reset manually, be reset when start
	U32_T ErrLogicRead;         // Counter incremantal when logical read command failed.(WKC error, FMMU read) 
}NETWORK_STATISTICS; // Low level network statistics.

typedef struct
{
	//EtherCAT Slave info.
	U16_T autoIncAddr;
	U16_T configAddr;
	U32_T vendorId;
	U32_T productCode;
	U32_T revisionNo;
	U32_T serialNo;

	U16_T DL_Status; // Will be initial in trans. state IP (offset:0x0110)
	U16_T res;
} SLAVE_INFO;

typedef struct  // Get all infomation of slave at once.
{
	SLAVE_INFO    info;
	U16_T         appType;  
	U16_T         totalInitCmdCnt;
	
	// State info
	U16_T   currState;
	U16_T   targetState;
	U16_T   subState;
	U16_T   __SlaveIndex;   //<== [i] This var is input. To get which slave's info

	// ProcessImage info
	U16_T SendByteMasterProcImgOfs;
	U16_T SendByteLen;
	U16_T RecvByteMasterProcImgOfs;
	U16_T RecvByteLen;

} SLAVE_DETAIL;

typedef struct  // Get all infomation of master at once.
{
	U16_T  totalSlaves;
	U16_T  totalInitCmds;
} MASTER_DETAIL;

typedef enum
{
	  RT_INFO_SLAVE  = 0
	, RT_INFO_MASTER
	, RT_INFO_NETWORK_STATISTICS
} RT_INFO_TYPE;

typedef union
{
	SLAVE_DETAIL       slave;
	MASTER_DETAIL      master;
	NETWORK_STATISTICS nws;
} RT_INFOMATIONS;

//
// definition of SDO command status.
#define SDO_INIT          (0) // SDO in communication
#define SDO_SUCCESS       (1) // SDO communication success
#define SDO_FAIL          (2) // Master error
#define SDO_ABORT         (3) // Slave abort SDO process 
#define SDO_EMCY          (4) // Slave emcy SDO process
#define SDO_ERROR         (5) // Slave return unexpected data.

typedef struct
{
	U16_T  index;        // [i]   Object index
	U8_T   subIndex;     // [i]   Objec sub-index
	U8_T   ctrlFlag;     // [i]   Control flag, bit 0: Complete access
	U8_T  *dataPtr;      // [i/o] point ot input or output data.
	U16_T  dataLenByte;  // [i/o] Lengh of input or output data. 
	U16_T  status;       // [o]   For RT user: SDO_INIT -> SDO_SUCCESS or SDO_FAIL, For Win32:Reserved
	I32_T  abortCode;    // [o]   Return SDO abort code or master error code. 
} TSDO;

typedef struct
{
	U16_T numOfAllObj;		 // [o] All objects in slave.
	U16_T numOfRxPdoObj;	 // [o] Mappable RxPDO objects in slave.
	U16_T numOfTxPdoObj;     // [o] Mappable TxPDO objects in slave.
	U16_T numOfBackupObj;    // [o] Objects can be stored for device replacement.
	U16_T numOfStartObj;     // [o] Objects be used in startup state.
	U16_T status;			 // [o] For RT user: SDO_INIT -> SDO_SUCCESS or SDO_FAIL, For Win32:Reserved
	I32_T abortCode;         // [o] return abort code, if CoE request is failed.
}TCoEODListCount;


#define LIST_TYPE_DELY_ALL_OD_LIST		(1) //all objects of the object dictionary shall be delivered in the response
#define LIST_TYPE_DELY_RPDO_OD_LIST		(2) //only objects which are mappable in a RxPDO shall be delivered in the response
#define LIST_TYPE_DELY_TPDO_OD_LIST		(3) //only objects which are mappable in a TxPDO shall be delivered in the response
#define LIST_TYPE_DELY_DEV_REPL_OD_LIST	(4) //only objects which has to stored for a device replacement shall be delivered in the response
#define LIST_TYPE_DELY_STARTUP_OD_LIST  (5) //only objects which can be used as startup parameter shall be delivered in the response.
typedef struct
{
	U16_T listType;		 // [i] Object list type
	U16_T lenOfList;	 // [i/o] Length of listData array
	U16_T *plistData;    // [o] Pointer to list data array
	U16_T status;	     // [o] For RT user: SDO_INIT -> SDO_SUCCESS or SDO_FAIL, For Win32:Reserved
    U16_T res;           // [o] reserved.
	I32_T abortCode;     // [o] return abort code, if CoE request is failed.
}TCoEODList;

typedef struct
{
	U16_T  index;				// [i] Object index
	U16_T  dataType;			// [o] return data type
	U8_T   maxNumOfSubIndex;    // [o] return maximum of sub-index
	U8_T   objectCode;			// [o] return object code
    U16_T  sizeOfName;          // [i/o] return length size of name 
	U8_T   *pName;              // [o] return name of object
	U16_T  reserved;            // [o] Reserved, set zero
	U16_T  status;              // [o] For RT user: SDO_INIT -> SDO_SUCCESS or SDO_FAIL, For Win32:Reserved
	I32_T  abortCode;           // [o] return abort code, if CoE request is failed.
}TCoEObjDesc;

typedef struct
{
	U16_T  index;			// [i] Object index
	U8_T   subIndex;		// [i] Object sub index
	U8_T   valueInfo;		// [i] Bit mask, the value info includes
	U16_T  dataType;		// [o] index of the data type of the object
    U16_T  bitLength;       // [o] return bit length of the object
	U16_T  objectAccess;    // [o] return object's rw attribute
	U16_T  sizeOfData;      // [i/o] Length of pData array
	U8_T   *pData;          // [o] Entry description
	U16_T  reserved;        // [o] Reserved, set zero
	U16_T  status;          // [o] For RT user: SDO_INIT -> SDO_SUCCESS or SDO_FAIL, For Win32:Reserved
	I32_T  abortCode;       // [o] return abort code, if CoE request is failed.
}TCoEEntryDesc;

typedef struct
{
	U16_T errorCode;				// [o] Emergency error code. This value equal to 0x603F
	U8_T  errorRegister;			// [o] Emergency error register. This value equal to 0x1001
	U8_T  data[5];			        // [o] Emergency data.
}TCoEEmgMsg;

typedef struct
{
	U16_T lenOfData;
	U16_T res;
	U16_T *pSlaveAddrDataArr;
	TCoEEmgMsg *pEmgMsgDataArr;
}TEmgData;

typedef struct
{
	U16_T index;
	U8_T  subIndex;
	U8_T  res;
	U32_T abortCode;
}TCoEAbtMsg;

typedef struct
{
	U16_T lenOfData;
	U8_T  moreData;
	U8_T  res;
	U16_T *pSlaveAddrDataArr;
	TCoEAbtMsg *pAbtMsgDataArr;
}TAbtData;

typedef struct
{
	U16_T offsetByWord;
	U16_T dataLenByte;
	U16_T status;
	U16_T failInfo;
	U8_T  *dataPtr;
}TRWEEPROM;

#define OBJ_DATA_TYPE_BIT_T		(0x01)
#define OBJ_DATA_TYPE_I8_T		(0x02)
#define OBJ_DATA_TYPE_I16_T		(0x03)
#define OBJ_DATA_TYPE_I32_T		(0x04)
#define OBJ_DATA_TYPE_U8_T		(0x05)
#define OBJ_DATA_TYPE_U16_T		(0x06)
#define OBJ_DATA_TYPE_U32_T		(0x07)
#define OBJ_DATA_TYPE_CHAR_T    (0x09)
#define OBJ_DATA_TYPE_BINARY_T  (0x0a)

#define OBJ_ACCESS_TYPE_RO		(0) 
#define OBJ_ACCESS_TYPE_RW		(1)


#define MAX_OBJ_NAME			(64)


typedef struct
{
	U16_T  index;
	U8_T   subIndex;
	U8_T   accessType;
	U16_T  dataType;
	U16_T  bitLength;
	U8_T   name[MAX_OBJ_NAME];
}CoEObjInfo;

// 
// For RT application
// Realtime callback functions define
typedef void (*NEC_RtCyclicCallback)( void *UserDataPtr );	                // For cyclic process. 
typedef void (*NEC_RtEventCallback) ( void *UserDataPtr, U32_T EventCode ); // State change notification
typedef void (*NEC_RtErrorCallback) ( void *UserDataPtr, I32_T ErrorCode ); // Error notification

// EventCallback Event code
#define EVENT_ECM_STATE_CHANGE  (0)
#define EVENT_ECM_CNECT_FINISH  (1) //when the process of connection is done( all slave is online).  
#define EVENT_ECM_EMG_OCCUR     (2)

//
// For RT application
// NEC_RtRegisterClient() / NEC_RtUnregisterClient()
typedef struct
{
	U32_T                version;        // [i] Assign version using NEC_RtRetVer()
	void                 *userDataPtr;   // [i] Assign data pointer to master. Master pass data pointer to callback functions. Can be null (set 0)
	NEC_RtCyclicCallback cyclicCallback; // [i] cyclic callback, can be null (set 0)
	NEC_RtEventCallback  eventCallback;  // [i] event callback, can be null (set 0)
	NEC_RtErrorCallback  errorCallback;  // [i] error callback, can be null (set 0)
	U16_T                clientID;       // [o] return client identification.
} TClintParam;
	



#endif
