/******************************************************************************

	Building The Digital Infrastructure 
	(C) 2012 NEXCOM International Co., Ltd. All Rights Reserved.
*******************************************************************************/
#ifndef __NEXECM_H
#define __NEXECM_H

#define FNTYPE  __stdcall

#ifdef __cplusplus
  extern "C" {
#endif 

#include "nex_type.h"
#include "RtDataStructDef.h"

//*** StartNetworkEx() Option define.	  
#define START_NETWORK_OPT_NIC_PORT_USING_PARAMETER (1)
// #define START_NETWORK_OPT_MASK_CYCLETIME  (2)   
#define MAX_CONFIG_FILE_NAME_LEN                   (248)

// Master type
#define  MASTER_TYPE_WIN32  (0)
#define  MASTER_TYPE_RTX    (1)
#define  MASTER_TYPE_CARD   (2)

//*** Network state definition ***
#define STATE_STOPPED      (0) // Networking is stopped.
#define STATE_OPERATIONAL  (1) // Networking is in operation state.(EtherCAT in OP state)
#define STATE_ERROR        (2) // Networking / slaves errors, and stopped.
#define STATE_SLAVE_RETRY  (3) // Networking / slaves errors, and try to re-connect.

//***********************
//*** API list below ***
//***********************
U32_T   FNTYPE NEC_GetVersion();  // Get NexECM DLL version
RTN_ERR FNTYPE NEC_StartDriver();
void    FNTYPE NEC_StopDriver();
U16_T   FNTYPE NEC_GetAvailableDriverCount();
U16_T   FNTYPE NEC_GetMasterType( U16_T MasterId, U16_T *Type );

RTN_ERR FNTYPE NEC_SetParameter( U16_T MasterId, U16_T ParaNum, I32_T  ParaData );
RTN_ERR FNTYPE NEC_GetParameter( U16_T MasterId, U16_T ParaNum, I32_T *ParaData );

 // TimeoutMs: wait state to operation state. if value = -1 no wait, just start communication and you can check state by call NEC_GetNetworkState()
RTN_ERR FNTYPE NEC_StartNetwork( U16_T MasterId, const char *ConfigurationFile, I32_T TimeoutMs );  
RTN_ERR FNTYPE NEC_StartNetworkEx( U16_T MasterId, const char *ConfigurationFile, U32_T Option, I32_T TimeoutMs );  
RTN_ERR FNTYPE NEC_StopNetwork( U16_T MasterId, I32_T TimeoutMs );
RTN_ERR FNTYPE NEC_GetSlaveCount( U16_T MasterId, U32_T *Count ); // be called after StartNetwork()
RTN_ERR FNTYPE NEC_GetNetworkState( U16_T MasterId, U16_T *State );
RTN_ERR FNTYPE NEC_GetSlaveState( U16_T MasterId, U16_T SlaveIndex, U8_T *StateArr, U16_T *ArrLen );
RTN_ERR FNTYPE NEC_GetStateError( U16_T MasterId, I32_T *Code  );       // Return error code.
RTN_ERR FNTYPE NEC_GetErrorMsg( U16_T MasterId, char *ErrMsg_128_len ); // Return error message when state error.

//*** Application functions ****
  // <<< non-synchronized DIO functions, set DO value to memory only. >>> 
RTN_ERR FNTYPE NEC_SetDo( U16_T MasterId, U16_T SlaveAddr, U16_T Offset, U16_T SizeByte, const U8_T *DoData );
RTN_ERR FNTYPE NEC_GetDo( U16_T MasterId, U16_T SlaveAddr, U16_T Offset, U16_T SizeByte, U8_T *DoData );
RTN_ERR FNTYPE NEC_GetDi( U16_T MasterId, U16_T SlaveAddr, U16_T Offset, U16_T SizeByte, U8_T *DiData );
  // <<< synchronized DIO functions, set DO value to memory and wait master accepted. >>>
RTN_ERR FNTYPE NEC_SetDo_s( U16_T MasterId, U16_T SlaveAddr, U16_T Offset, U16_T SizeByte, const U8_T *DoData );

RTN_ERR FNTYPE NEC_SDODownload( U16_T MasterId, U16_T SlaveAddr, TSDO *HSdo );
RTN_ERR FNTYPE NEC_SDOUpload( U16_T MasterId, U16_T SlaveAddr, TSDO *HSdo );

RTN_ERR FNTYPE NEC_SDODownloadEx( U16_T MasterId, U16_T SlaveAddr
	, U16_T  Index        // [i]   Object index
	, U8_T   SubIndex     // [i]   Objec sub-index
	, U8_T   CtrlFlag     // [i]   Control flag, bit 0: Complete access
	, U32_T  DataLenByte  // [i/o] Lengh of input or output data. 
	, U8_T  *DataPtr      // [i/o] point ot input or output data.
	, U32_T *AbortCode    // [o]   SDO abort code
	);

RTN_ERR FNTYPE NEC_SDOUploadEx( U16_T MasterId, U16_T SlaveAddr 
	, U16_T  Index        // [i]   Object index
	, U8_T   SubIndex     // [i]   Objec sub-index
	, U8_T   CtrlFlag     // [i]   Control flag, bit 0: Complete access
	, U32_T  DataLenByte  // [i/o] Lengh of input or output data. 
	, U8_T  *DataPtr      // [i/o] point ot input or output data.
	, U32_T *AbortCode    // [o]   SDO abort code
	);

RTN_ERR FNTYPE NEC_GetODListCount( U16_T MasterId, U16_T SlaveAddr, TCoEODListCount *pCoeOdListCount );
RTN_ERR FNTYPE NEC_GetODList( U16_T MasterId, U16_T SlaveAddr, TCoEODList *pCoeOdList );
RTN_ERR FNTYPE NEC_GetObjDesc( U16_T MasterId, U16_T SlaveAddr, TCoEObjDesc *pCoeObjDesc );
RTN_ERR FNTYPE NEC_GetEntryDesc( U16_T MasterId, U16_T SlaveAddr, TCoEEntryDesc *pCoeEntryDesc );
RTN_ERR FNTYPE NEC_GetEmgDataCount( U16_T MasterId, U16_T *pEmgDataCount );
RTN_ERR FNTYPE NEC_GetEmgData( U16_T MasterId, TEmgData *pEmgData );

#define READ_PI  (0)   // Read ProcessImage (PDI)
#define WRITE_PI (1)   // Write ProcessImage (PDO)
#define READ_PDO  (2)  // Read ProcessImage (PDO )
RTN_ERR FNTYPE NEC_RWProcessImage( U16_T MasterId, U16_T RW, U16_T Offset, U8_T *Data, U16_T Size );
RTN_ERR FNTYPE NEC_GetProcessImageSize( U16_T MasterId, U16_T *SizeOfInputByte, U16_T *SizeOfOutputByte );
RTN_ERR FNTYPE NEC_RWSlaveProcessImage( U16_T MasterId, U16_T SlaveAddr, U16_T RW, U16_T Offset, U8_T *Data, U16_T Size );

//*************************************
//*** API for realtime applications ***
//*************************************
RTN_ERR FNTYPE NEC_GetRtMasterId( U16_T *pMasterId ); // This api is used to get First RT(RTX) master's ID
RTN_ERR FNTYPE NEC_ResetEcMaster( U16_T MasterId );  // Reset EtherCAT Master, execute only when ECM is stopped or return error 

#define MAX_RTX_FILE_NAME  256
RTN_ERR FNTYPE NEC_LoadRtxApp( const char* RtxFileName ); 
RTN_ERR FNTYPE NEC_KillRtxApp( U32_T ProcessId ); // Force kill RTX Application

#define MAX_ENI_FILE_NAME  248
RTN_ERR FNTYPE NEC_LoadNetworkConfig( U16_T MasterId, const char *ConfigurationFile, U32_T Option ); // Load ENI file 

RTN_ERR FNTYPE NEC_GetRtMasterPorcessId( U16_T MasterId, U32_T *pProcessID ); // This api is used to get RT(RTX) master's Process ID
RTN_ERR FNTYPE NEC_GetRtMasterVersion( U16_T MasterId, U32_T *pVersion );   // This api is used to get RT(RTX) Master's Runtime version

RTN_ERR FNTYPE NEC_KillNexECMRtxRuntime();

RTN_ERR FNTYPE NEC_GetConfiguredAddress( U16_T MasterId, U16_T SlaveAddr, U16_T *pConfigAddr ); // This api is used to get slave's config address.
RTN_ERR FNTYPE NEC_GetAliasAddress( U16_T MasterId, U16_T SlaveAddr, U16_T *pAliasAddr );      // This api is used to get slave's alias address.

RTN_ERR FNTYPE NEC_GetSlaveCoeProfileNum( U16_T MasterId, U16_T SlaveAddr, U32_T *pCoeProfileNum );

//
// Obselate APIs
//
#define START_NETWORK_OPT_MASK_NIC_PORT  (1)  //Option define.

#ifdef __cplusplus
}
#endif 

#endif
