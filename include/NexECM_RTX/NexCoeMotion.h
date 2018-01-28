#ifndef NEX_COE_MOTION_H
#define NEX_COE_MOTION_H

#ifdef __cplusplus
  extern "C" {
#endif 

#include "nex_type.h"

typedef void* CANAxis_T; 
#define CAN_ALL_AXES     ((CANAxis_T)(-1))


#ifdef IZ_RTX
  #ifdef NEXECMRTX_EXPORT
    #define FNTYPE __declspec(dllexport)
  #else
    #define FNTYPE __declspec(dllimport)
  #endif
#else
  #ifdef UNDER_RTSS
    #define FNTYPE __declspec(dllimport)
  #else
    #define FNTYPE __stdcall
  #endif
#endif

#ifdef MS_WIN32
#ifndef FNTYPE
#define FNTYPE __stdcall
#endif
#endif


//
// CoE CANOpen Profile 402 for servo control
//


// CiA402 object index
#define INDEX_MODE_OF_OPERATION                   (0x6060) 
#define INDEX_MODE_OF_OPERATION_DISPLAY           (0x6061)
#define INDEX_MAX_PROFILE_VELOCITY                (0x607F) // U32_T, PP, PV
#define INDEX_PROFILE_VELOCITY                    (0x6081) // U32_T, PP
#define INDEX_PROFILE_ACCELERATION                (0x6083) // U32_T, PP, PV 
#define INDEX_PROFILE_DECELERATION                (0x6084) // U32_T, PP, PV 
#define INDEX_QUICK_STOP_DEC                      (0x6085) // U32_T, PP, PV
#define INDEX_TARGET_VELOCITY                     (0x60FF) // I32_T, PV

#define INDEX_HOME_OFFSET                         (0x607C)
#define INDEX_HOME_METHOD                         (0x6098)
#define INDEX_HOME_SPEED                          (0x6099) // SubIndex = 1
#define INDEX_HOME_ZERO_SPEED                     (0x6099) // SubIndex = 2
#define INDEX_HOME_ACC                            (0x609A) 

//PDO Type
#define TYPE_RXPDO            (0)  // Type of RxPDO
#define TYPE_TXPDO            (1)  // Type of TxPDO

// Type of CiA402 Axis
#define EC_CIA402_SERVO_DRIVE     (0)  // For Most standard EtherCAT CoE servo drive, Only ref SlaveAddr, 
#define EC_MULTIPLE_CIA402_SLAVE  (1)  // For multiple slots/channel EtherCAT slave 

// CoE axis PDO mapping information
#pragma pack(push, 1)
typedef struct 
{
    U16_T TypeOfSlave;  // Refer to  EC_CIA402_SERVO_DRIVE ... define
    U16_T SlaveAddr;

    U16_T SlaveSlotNum;
    U16_T Reserved;        // Internal used set zero.
    U16_T CoeObjectOffset; // ex. 0x0800
    U16_T pdoMapOffset;    // ex. 0x0010

    U16_T MinRxPdoIndex;   // ex. 0x1600
    U16_T MaxRxPdoIndex;   // ex. 0x1603
    U16_T MinTxPdoIndex;   // ex. 0x1A00
    U16_T MaxTxPdoIndex;   // ex. 0x1A03
}EcCiA402AxisMapInfo_T;
#pragma pack(pop)


RTN_ERR FNTYPE NEC_CoE402Reset();
RTN_ERR FNTYPE NEC_CoE402Close();
RTN_ERR FNTYPE NEC_CoE402GetAxisId( U16_T MasterId, U16_T SlaveAddr, CANAxis_T *pAxis );  //This API only for CoE EtherCAT servo drive
RTN_ERR FNTYPE NEC_CoE402GetDefaultMapInfo( U16_T TypeOfSlave, U16_T SlaveAddr, EcCiA402AxisMapInfo_T *pMapInfo );
RTN_ERR FNTYPE NEC_CoE402GetAxisIdEx( U16_T MasterId, EcCiA402AxisMapInfo_T *pInfo, CANAxis_T *pAxis );
RTN_ERR FNTYPE NEC_CoE402ResetPdoMapping( CANAxis_T Axis );  // support CAN_ALL_AXES
RTN_ERR FNTYPE NEC_CoE402UpdatePdoMapping( CANAxis_T Axis );  // support CAN_ALL_AXES

// State control functions
RTN_ERR FNTYPE NEC_CoE402CyclicProcess(); // Callback 
RTN_ERR FNTYPE NEC_CoE402SetSlaveProcessCountPreCycle( U16_T AxisCount );    // Callback ok
RTN_ERR FNTYPE NEC_CoE402SetCtrlWord( CANAxis_T Axis, U16_T CtrlInBit );      // Callback ok (PDO mapped only)
RTN_ERR FNTYPE NEC_CoE402GetCtrlWord( CANAxis_T Axis, U16_T *CtrlInBit );     // Callback ok (PDO mapped only)
RTN_ERR FNTYPE NEC_CoE402GetStatusWord( CANAxis_T Axis, U16_T *StatusInBit ); // Callback ok (PDO mapped only)

// State for CiA402 
#define COE_STA_DISABLE                (0)  // ServoOff
#define COE_STA_READY_TO_SWITCH_ON     (1)
#define COE_STA_SWITCH_ON              (2)
#define COE_STA_OPERATION_ENABLE       (3)  // ServoOn
#define COE_STA_QUICK_STOP_ACTIVE      (4)  
#define COE_STA_FAULT                  (5)  // Error, Servo Internal
#define COE_STA_FAULT_REACTION_ACTIVE  (6)  // Error, Servo Internal
#define MAX_COE_STA                    (7)
#define COE_STA_UNKNOWN                (0xFF)
RTN_ERR FNTYPE NEC_CoE402ChangeState( CANAxis_T Axis, U16_T TargetState, I32_T TimeoutMs ); // CAN_ALL_AXES support
RTN_ERR FNTYPE NEC_CoE402SetState( CANAxis_T Axis, U16_T State );                           // Callback ok, CAN_ALL_AXES support
RTN_ERR FNTYPE NEC_CoE402GetState( CANAxis_T Axis, U16_T *State );                          // Callback ok
RTN_ERR FNTYPE NEC_CoE402FaultReset( CANAxis_T Axis,  I32_T TimeoutMs );                    // Callback ok, CAN_ALL_AXES support
RTN_ERR FNTYPE NEC_CoE402ServoOn( CANAxis_T Axis, U16_T OnOff );                            // 0:ServoOff, 1:ServoOn; CAN_ALL_AXES support

// CoE Mode of operation
#define CiA402_OP_MODE_PROFILE_POSITION       (1)
#define CiA402_OP_MODE_PROFILE_VELOCITY       (3)
#define CiA402_OP_MODE_TORQUE_PROFILE         (4)
#define CiA402_OP_MODE_HOMING                 (6)
#define CiA402_OP_MODE_INTERPOLATED_POSITION  (7)
#define CiA402_OP_MODE_CYCLIC_POSITION        (8)
#define CiA402_OP_MODE_CYCLIC_VELOCITY        (9)
#define CiA402_OP_MODE_CYCLIC_TORQUE          (10)
RTN_ERR FNTYPE NEC_CoE402SetOperationMode( CANAxis_T Axis, I8_T  MotionMode, I32_T CheckTimeoutMs );
RTN_ERR FNTYPE NEC_CoE402GetOperationModeDisplay( CANAxis_T Axis, I8_T *MotionMode );

RTN_ERR FNTYPE NEC_CoE402SetParameter( CANAxis_T Axis, U16_T Index, U8_T SubIndex, U8_T LenOfByte, I32_T Value     ); 
RTN_ERR FNTYPE NEC_CoE402GetParameter( CANAxis_T Axis, U16_T Index, U8_T SubIndex, U8_T LenOfByte, void *pRetValue );

// For CSP operation mode
RTN_ERR FNTYPE NEC_CoE402GetActualPosition( CANAxis_T Axis, I32_T *Position );  // Callback ok, Return motor actual position (0x6064)
RTN_ERR FNTYPE NEC_CoE402SetTargetPosition( CANAxis_T Axis, I32_T TargetPos );  // Callback ok, Write data to process data   (0x607A)
RTN_ERR FNTYPE NEC_CoE402GetTargetPosition( CANAxis_T Axis, I32_T *TargetPos ); // Callback ok, Return motor target position (0x607A)

// For CSV operation mode
RTN_ERR FNTYPE NEC_CoE402SetTargetVelocity( CANAxis_T Axis, I32_T TargetVel );   // Callback ok (PDO mapped only) (0x60FF)
RTN_ERR FNTYPE NEC_CoE402GetActualVelocity( CANAxis_T Axis, I32_T *ActualVel);   // Callback ok (PDO mapped only) (0x606C)
	
RTN_ERR FNTYPE NEC_CoE402SetQuickStopDec( CANAxis_T Axis, U32_T QuickStopDec );    // (0x6085)
RTN_ERR FNTYPE NEC_CoE402SetSoftPosLimit( CANAxis_T Axis, I32_T MinPositionLimit, I32_T MaxPositionLimit ); // (0x607D) 
RTN_ERR FNTYPE NEC_CoE402SetMaxVelLimit( CANAxis_T Axis, U32_T MaxVelocityLimit ); // (0x607F)

// Operation mode: Profile Position mode
#define OPT_ABS 0x00000000 // Absolution position
#define OPT_REL 0x00000040 // Releative distance
#define OPT_WMC 0x00010000 // Wait move complete
#define OPT_IMV 0x10000000 // Ignore MaxVel
#define OPT_IAC 0x20000000 // Ignore Acc
#define OPT_IDC 0x40000000 // Ignore Dec
RTN_ERR FNTYPE NEC_CoE402Ptp(  CANAxis_T Axis, U32_T Option, I32_T TargetPos );
RTN_ERR FNTYPE NEC_CoE402PtpV( CANAxis_T Axis, U32_T Option, I32_T TargetPos, U32_T MaxVel );
RTN_ERR FNTYPE NEC_CoE402PtpA( CANAxis_T Axis, U32_T Option, I32_T TargetPos, U32_T MaxVel, U32_T Acc, U32_T Dec );
RTN_ERR FNTYPE NEC_CoE402WaitTargetReached( CANAxis_T Axis );


// Operation mode: Profile Velocity mode
RTN_ERR FNTYPE NEC_CoE402Jog(  CANAxis_T Axis, U32_T Option, I32_T MaxVel );
RTN_ERR FNTYPE NEC_CoE402JogA( CANAxis_T Axis, U32_T Option, I32_T MaxVel, U32_T Acc, U32_T Dec );
RTN_ERR FNTYPE NEC_CoE402Halt( CANAxis_T Axis, U32_T OnOff  );

// Operation mode: Homing
#define OPT_MTD 0x08000000  // Ignore Method
// #define OPT_IMV 0x10000000 // Ignore MaxVel
// #define OPT_IAC 0x20000000 // Ignore Acc
#define OPT_IZV 0x40000000  // Ignore ZeroVel
#define OPT_IOF 0x80000000  // Ignore Offset
RTN_ERR FNTYPE NEC_CoE402Home( CANAxis_T Axis, U32_T Option ); // Start home move, this API will set the "home start bit"
RTN_ERR FNTYPE NEC_CoE402HomeEx( CANAxis_T Axis, U32_T Option, I8_T Method, I32_T Offset, U32_T MaxVel, U32_T ZeroVel, U32_T Acc ); // Start home move, this API will set the "home start bit"
RTN_ERR FNTYPE NEC_CoE402WaitHomeFinished( CANAxis_T Axis ); //Wait until homing end and clear the "home start bit".
RTN_ERR FNTYPE NEC_CoE402ClearHomeStartBit( CANAxis_T Axis ); // Clear "home start bit"

// State for homing
#define HOMING_IN_PROGRESS      (0)
#define HOMING_TARGET_NOT_REACH (1)
#define HOMING_COMPLETE         (2)
#define HOMING_INTERRUPTED      (3)
#define HOMING_ERR_VEL_ZERO     (4)
#define HOMING_ERR_VEL_NON_ZERO (5)

RTN_ERR FNTYPE NEC_CoE402CheckHomeStatus( CANAxis_T Axis, U16_T *pStatus ); // Return status of homing. This API will clear "home start bit" if the homing is stoped, no matter homing is failed or successful.

// Operation mode: Profile Torque Mode /Cyclic Sync Torque mode, 
RTN_ERR FNTYPE NEC_CoE402SetTargetTorque( CANAxis_T Axis, I16_T TargetTorque );  // Callback ok, (0x6071)
RTN_ERR FNTYPE NEC_CoE402GetTargetTorque( CANAxis_T Axis, I16_T *TargetTorque ); // Callback ok, (0x6071)
RTN_ERR FNTYPE NEC_CoE402GetActualTorque( CANAxis_T Axis, I16_T *TorqueActualValue ); // Callback ok, (0x6077)

// Operation mode: Profile Torque Mode
RTN_ERR FNTYPE NEC_CoE402SetTorqueProfile( CANAxis_T Axis, I16_T TargetTorque, U32_T TorqueSlope );  //For profile torque mode 

//*******************************************************************************
// Reserved functions.
//********************************************************************************
#define CW_HALT                  0x0100  // Halt
// Profile position move
#define CW_PP_ABSOLUTE           0x0000
#define CW_PP_RELATIVE           0x0040
#define CW_PP_START_IMMEDIATELY  0x0020
#define O_WTR  0x10000 // Wait target reach
RTN_ERR FNTYPE NEC_CoE402ProfilePositionMove( CANAxis_T Axis, I32_T TargetPos, U16_T CtrlWord_B4_15 );
RTN_ERR FNTYPE NEC_CoE402ProfileVelocityMove( CANAxis_T Axis, I32_T TargetVel, U16_T CtrlWord_B4_15 );
RTN_ERR FNTYPE NEC_CoE402ProfileVelocityMoveAcc( CANAxis_T Axis, I32_T TargetVel, U32_T Acceleration, U32_T Deceleration, U16_T CtrlWord_B4_15 );
RTN_ERR FNTYPE NEC_CoE402Homing( CANAxis_T Axis, U16_T CtrlWord_B4_15 );

//RTN_ERR FNTYPE NEC_CoEGetActualVelocity( CANAxis_T Axis, I32_T *Position );   // Return motor actual velocity 
//RTN_ERR FNTYPE NEC_CoEGetActualTorque( CANAxis_T Axis, I32_T *Position );     // Return motor actual torque 
//RTN_ERR FNTYPE NEC_CoESetTargetVelocity( CANAxis_T Axis, I32_T TargetVel );
//RTN_ERR FNTYPE NEC_CoESetTargetTorque( CANAxis_T Axis, I32_T TargetTrq );
//RTN_ERR FNTYPE NEC_CoE402AddPdoMapping( CANAxis_T Axis, U16_T Type, U16_T CANIndex, U8_T CANSubIndex, U8_T OffsetOfByte, U8_T LenOfByte );  // obsloate, support CAN_ALL_AXES

#ifdef __cplusplus
  }
#endif 




#endif