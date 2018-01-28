#ifndef __NEXECM_RTX_H
#define __NEXECM_RTX_H

#ifdef __cplusplus
  extern "C" {
#endif 

#ifdef NEXECMRTX_EXPORT
#define FNTYPE __declspec(dllexport)
#else
#define FNTYPE __declspec(dllimport)
#endif

//*** Network state definition ***
#define STATE_STOPPED      (0) // Networking is stopped.
#define STATE_OPERATIONAL  (1) // Networking is in operation state.(EtherCAT in OP state)
#define STATE_ERROR        (2) // Networking / slaves errors, and stopped.
#define STATE_SLAVE_RETRY  (3) // Networking / slaves errors, and try to re-connect.

#include "nex_type.h"
#include "RtDataStructDef.h"

// General APIs
RTN_ERR FNTYPE NEC_RtGetVersion( U32_T *Version );
U32_T   FNTYPE NEC_RtRetVer(); 
RTN_ERR FNTYPE NEC_RtInitMaster( U16_T MasterId );   // Resource initialization
RTN_ERR FNTYPE NEC_RtCloseMaster( U16_T MasterId );  // Resource release, it will also stop master.
RTN_ERR FNTYPE NEC_RtCheckLicense();  //Return sucess  mean license is active.

RTN_ERR FNTYPE NEC_RtSetParameter( U16_T MasterId, U16_T ParaNum, I32_T  ParaData );// CANNOT use this function in callback
RTN_ERR FNTYPE NEC_RtGetParameter( U16_T MasterId, U16_T ParaNum, I32_T *ParaData );// CANNOT use this function in callback

RTN_ERR FNTYPE NEC_RtRegisterClient( U16_T MasterId, TClintParam *ClientParam );// CANNOT use this function in callback
RTN_ERR FNTYPE NEC_RtUnregisterClient( U16_T MasterId, TClintParam *ClientParam );// CANNOT use this function in callback
RTN_ERR FNTYPE NEC_RtStartMaster( U16_T MasterId );    // CANNOT use this function in callback
RTN_ERR FNTYPE NEC_RtStopMaster( U16_T MasterId );     // CANNOT use this function in callback
RTN_ERR FNTYPE NEC_RtStopMasterCb( U16_T MasterId );   // Stop master in callback, Actual stop in wait stop function
RTN_ERR FNTYPE NEC_RtWaitMasterStop( U16_T MasterId ); // CANNOT use this function in callback
RTN_ERR FNTYPE NEC_RtLoadNetworkConfig(U16_T MasterId, const char *ConfigurationFile, U32_T Option); // Just support NexECMRtx2014

#define ECM_STA_INIT        (1)
#define ECM_STA_PREOP       (2)
#define ECM_STA_SAFEOP      (3)
#define ECM_STA_OPERATION   (4)
#define ECM_STA_ERROR       (6)
RTN_ERR FNTYPE NEC_RtGetMasterState( U16_T MasterId, U16_T *State );
RTN_ERR FNTYPE NEC_RtSetMasterState( U16_T MasterId, U16_T  State );
RTN_ERR FNTYPE NEC_RtGetStateError( U16_T MasterId, I32_T *Code  );
RTN_ERR FNTYPE NEC_RtSetMasterStateWait( U16_T MasterId, U16_T  State, I32_T TimeoutMs  ); // CANNOT use this function in callback
RTN_ERR FNTYPE NEC_RtChangeStateToOP( U16_T MasterId, I32_T TimeoutMs  ); // CANNOT use this function in callback

// Process data access related functions
RTN_ERR FNTYPE NEC_RtGetProcessDataPtr( U16_T MasterId, U8_T **InputProcessDataPtr, U32_T *InPDSizeInByte, U8_T **OutputProcessDataPtr, U32_T *OutPDSizeInByte );
RTN_ERR FNTYPE NEC_RtSetProcessDataPtrSource( U16_T MasterId, void *InputProcessDataPtrSource, U32_T InPDSizeInByte, void *OutputProcessDataPtrSource, U32_T OutPDSizeInByte );
RTN_ERR FNTYPE NEC_RtGetSlaveProcessDataPtr( U16_T MasterId, U16_T SlaveAddr, U8_T **InputProcessDataPtr, U32_T *InPDSizeInByte, U8_T **OutputProcessDataPtr, U32_T *OutPDSizeInByte );

RTN_ERR FNTYPE NEC_RtSetProcessDataOutput( U16_T MasterId, U32_T PDOAddr, U32_T LengthOfData, U8_T *SetData );
RTN_ERR FNTYPE NEC_RtGetProcessDataOutput( U16_T MasterId, U32_T PDOAddr, U32_T LengthOfData, U8_T *RetData );
RTN_ERR FNTYPE NEC_RtGetProcessDataInput(  U16_T MasterId, U32_T PDIAddr, U32_T LengthOfData, U8_T *RetData );

RTN_ERR FNTYPE NEC_RtSetSlaveProcessDataOutput( U16_T MasterId, U16_T SlaveAddr, U16_T Offset, U8_T *Data, U32_T Size );
RTN_ERR FNTYPE NEC_RtGetSlaveProcessDataOutput( U16_T MasterId, U16_T SlaveAddr, U16_T Offset, U8_T *Data, U32_T Size );
RTN_ERR FNTYPE NEC_RtGetSlaveProcessDataInput( U16_T MasterId, U16_T SlaveAddr, U16_T Offset, U8_T *Data, U32_T Size );

// CoE SDO functions
RTN_ERR FNTYPE NEC_RtStartSDODownload( U16_T MasterId, U16_T SlaveAddr, TSDO *HSdo );
RTN_ERR FNTYPE NEC_RtStartSDOUpload( U16_T MasterId, U16_T SlaveAddr, TSDO *HSdo );

RTN_ERR FNTYPE NEC_RtSDODownload( U16_T MasterId, U16_T SlaveAddr, TSDO *HSdo ); // CANNOT use in callback
RTN_ERR FNTYPE NEC_RtSDOUpload( U16_T MasterId, U16_T SlaveAddr, TSDO *HSdo ); // CANNOT use in callback

RTN_ERR FNTYPE NEC_RtStartGetODListCount( U16_T MasterId, U16_T SlaveAddr, TCoEODListCount *pCoeOdListCount );
RTN_ERR FNTYPE NEC_RtStartGetODList( U16_T MasterId, U16_T SlaveAddr, TCoEODList *pCoeOdList );
RTN_ERR FNTYPE NEC_RtStartGetObjDesc( U16_T MasterId, U16_T SlaveAddr, TCoEObjDesc *pCoeObjDesc );
RTN_ERR FNTYPE NEC_RtStartGetEntryDesc( U16_T MasterId, U16_T SlaveAddr, TCoEEntryDesc *pCoeEntryDesc );
RTN_ERR FNTYPE NEC_RtGetEmgDataCount( U16_T MasterId, U16_T *pEmgDataCount );
RTN_ERR FNTYPE NEC_RtGetEmgData( U16_T MasterId, TEmgData *pEmgData );

RTN_ERR FNTYPE NEC_RtSDODownloadEx( U16_T MasterId, U16_T SlaveAddr   // CANNOT use in callback
	, U16_T  Index        // [i]   Object index
	, U8_T   SubIndex     // [i]   Objec sub-index
	, U8_T   CtrlFlag     // [i]   Control flag, bit 0: Complete access
	, U32_T  DataLenByte  // [i]   Lengh of input or output data. 
	, U8_T  *DataPtr      // [i]   point ot input or output data.
	, I32_T *AbortCode    // [o]   SDO abort code, Set 0 to ignore
); // CANNOT use in callback

RTN_ERR FNTYPE NEC_RtSDOUploadEx( U16_T MasterId, U16_T SlaveAddr  // CANNOT use in callback
	, U16_T  Index        // [i]   Object index
	, U8_T   SubIndex     // [i]   Objec sub-index
	, U8_T   CtrlFlag     // [i]   Control flag, bit 0: Complete access
	, U32_T  DataLenByte  // [i]   Lengh of input or output data. 
	, U8_T  *DataPtr      // [o]   point ot input or output data.
	, I32_T *AbortCode    // [o]   SDO abort code, Set 0 to ignore
); // CANNOT use in callback

RTN_ERR FNTYPE NEC_RtGetSlaveCount( U16_T MasterId, U16_T *Count ); // from static data (ENI)
RTN_ERR FNTYPE NEC_RtGetSlaveInfomation( U16_T MasterId, U16_T SlaveAddr, SLAVE_INFO *pSlaveInfo );
RTN_ERR FNTYPE NEC_RtGetSlaveState( U16_T MasterId, U16_T SlaveIndex, U8_T *StateArr, U16_T *ArrLen );

RTN_ERR FNTYPE NEC_RtGetConfiguredAddress(U16_T MasterId, U16_T SlaveAddr, U16_T *pConfigAddr); // CANNOT use in callback
RTN_ERR FNTYPE NEC_RtGetAliasAddress(U16_T MasterId, U16_T SlaveAddr, U16_T *pAliasAddr);      // CANNOT use in callback

RTN_ERR FNTYPE NEC_RtGetSlaveCoeProfileNum(U16_T MasterId, U16_T SlaveAddr, U32_T *pCoeProfileNum); 

#ifdef __cplusplus
}
#endif 



#endif
