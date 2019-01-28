/*    ========================================================================
    =                   Doosan Robot Framework Structure                      =
    =                   Copyright (c) Doosan Robotics.                        =   
    =_______________________________________________________________________  =
    = Title             : Doosan Robot Framwork Structure                     =
    = Author            : Lee Jeong-Woo<jeongwoo1.lee@doosan.com>             =
    = Description       : -                                                   =
    = Version           : 1.0 (GL010105)                                              =
    =                     0.                                                  =
    ======================================================================== */


#pragma once
#include "DRFC.h"

#ifdef __cplusplus
#include <string>
#include <list>
using namespace std;
#endif

#pragma pack(1)
typedef struct _SYSTEM_VERSION
{
    /* smarttp version */
    char                        _szSmartTp[MAX_SYMBOL_SIZE];
    /* controller version */
    char                        _szController[MAX_SYMBOL_SIZE];
    /* interpreter version */
    char                        _szInterpreter[MAX_SYMBOL_SIZE];
    /* inverter version */
    char                        _szInverter[MAX_SYMBOL_SIZE];
    /* SafetyBoard version */
    char                        _szSafetyBoard[MAX_SYMBOL_SIZE];
    /* robot serial number */
    char                        _szRobotSerial[MAX_SYMBOL_SIZE];
    /* robot model number*/
    char                        _szRobotModel[MAX_SYMBOL_SIZE];
    /* jts board version */
    char                        _szJTSBoard[MAX_SYMBOL_SIZE];
    /* flange board version */
    char                        _szFlangeBoard[MAX_SYMBOL_SIZE];

} SYSTEM_VERSION, *LPSYSTEM_VERSION;

typedef struct _ROBOT_MONITORING_JOINT 
{
    /* Position Actual Value in INC */
    float                       _fActualPos[NUM_JOINT];
    /* Position Actual Value in ABS */
    float                       _fActualAbs[NUM_JOINT];
    /* Velocity Actual Value */
    float                       _fActualVel[NUM_JOINT];
    /* Joint Error */
    float                       _fActualErr[NUM_JOINT];
    /* Target Position */
    float                       _fTargetPos[NUM_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUM_JOINT];

} ROBOT_MONITORING_JOINT, *LPROBOT_MONITORING_JOINT;

typedef struct _ROBOT_MONITORING_TOOL 
{
    /* Position Actual Value(0: tool, 1: flange) */
    float                       _fActualPos[2][NUM_TASK]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUM_TASK];
    /* Task Error */
    float                       _fActualErr[NUM_TASK];
    /* Target Position */
    float                       _fTargetPos[NUM_TASK];
    /* Target Velocity */
    float                       _fTargetVel[NUM_TASK];
    /* Solution Space */
    unsigned char               _iSolutionSpace;
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_TOOL, *LPROBOT_MONITORING_TOOL;

typedef struct _ROBOT_MONITORING_TORQUE 
{
    /* Dynamics Torque */
    float                       _fDynamicTor[NUM_JOINT];
    /* Joint Torque Sensor Value */
    float                       _fActualJTS[NUM_JOINT];
    /* External Joint Torque */
    float                       _fActualEJT[NUM_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUM_JOINT];

} ROBOT_MONITORING_TORQUE, *LPROBOT_MONITORING_TORQUE;

typedef struct _ROBOT_MONITORING_STATE 
{
    /* position control: 0, torque control: 1*/
    unsigned char               _iActualMode;
    /*joint space: 0, task space: 1*/
    unsigned char               _iActualSpace;

} ROBOT_MONITORING_STATE, *LPROBOT_MONITORING_STATE;

typedef struct _ROBOT_MONITORING_DATA
{
    ROBOT_MONITORING_STATE      _tState;
    /* joint */
    ROBOT_MONITORING_JOINT      _tJoint;
    /* task */
    ROBOT_MONITORING_TOOL       _tTool;
    /* torque */
    ROBOT_MONITORING_TORQUE     _tTorque;  

} MONITORING_CONTROL, *LPMONITORING_CONTROL;

typedef struct _MONITORING_MISC
{
    /* inner clock counter */
    double                       _dSyncTime;
    /* Digtal Input data */
    unsigned char                _iActualDI[NUM_FLANGE_IO];
    /* Digtal output data */
    unsigned char                _iActualDO[NUM_FLANGE_IO];
    /* brake state */
    unsigned char                _iActualBK[NUM_JOINT];
    /* robot button state */
    unsigned int                 _iActualBT[NUM_BUTTON];
    /* motor input current */
    float                        _fActualMC[NUM_JOINT];
    /* motro current temperature */
    float                        _fActualMT[NUM_JOINT];
} MONITORING_MISC, *LPMONITORING_MISC;

typedef struct _MONITORING_DATA
{
    MONITORING_CONTROL          _tCtrl;
    /* misc. */
    MONITORING_MISC             _tMisc;

} MONITORING_DATA, *LPMONITORING_DATA;

typedef struct _READ_CTRLIO_INPUT
{
    /* Digtal Input data */
    unsigned char               _iActualDI[NUM_DIGITAL];
#if 0
    /*  Analog Input type */
    unsigned char               _iActualAT[NUM_ANALOG];
#endif
    /* Analog Input data */
    float                       _fActualAI[NUM_ANALOG];
    /* switch input data */
    unsigned char               _iActualSW[NUM_SWITCH];
    /* Safety Input data */
    unsigned char               _iActualSI[NUM_SAFETY_IN];
    /* Encorder Input data */
    unsigned char               _iActualEI[NUM_ENCORDER];
    /* Encorder raw data */
    unsigned int                _iAcutualED[NUM_ENCORDER];

} READ_CTRLIO_INPUT, *LPREAD_CTRLIO_INPUT;

typedef struct _READ_CTRLIO_OUTPUT
{
    /* Digital Output data */
    unsigned char               _iTargetDO[NUM_DIGITAL];
#if 0
    /*  Analog Output type */
    unsigned char               _fTargetAT[NUM_ANALOG];
#endif    
    /* Analog Output data */
    float                       _fTargetAO[NUM_ANALOG];

} READ_CTRLIO_OUTPUT, *LPREAD_CTRLIO_OUTPUT;

typedef struct _MONITORING_CTRLIO
{
    /* input data */
    READ_CTRLIO_INPUT           _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT          _tOutput;
} MONITORING_CTRLIO, *LPMONITORING_CTRLIO;

typedef struct _MODBUS_REGISTER
{
    /* modbus i/o name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* modbus i/o  value */
    unsigned short              _iValue;

} MODBUS_REGISTER, *LPMODBUS_REGISTER;

typedef struct _MONITORING_MODBUS
{
    /* modbus i/o count */
    unsigned short              _iRegCount;
    /* modbus i/o values */
    MODBUS_REGISTER             _tRegister[MAX_MODBUS_TOTAL_REGISTERS];

} MONITORING_MODBUS, *LPMONITORING_MODBUS;

typedef struct _LOG_ALARM
{
    /* message level */
    unsigned char               _iLevel;
    /* group no */
    unsigned char               _iGroup;
    /* message no */
    unsigned int                _iIndex;
    /* message param */
    char                        _szParam[3][MAX_STRING_SIZE];
} LOG_ALARM, *LPLOG_ALARM;

typedef LOG_ALARM MONITORING_ALARM;

typedef struct _MOVE_POSB 
{
    /*  q               */
    float                       _fTargetPos[2][NUM_TASK];      
    /* blending motion type (line: 0, circle: 1) */
    unsigned char               _iBlendType;    
    /* blending radius  */
    float                       _fBlendRad;

} MOVE_POSB, *LPMOVE_POSB;

typedef struct _ROBOT_POSE
{
    /* current pose */
    float                       _fPosition[NUM_JOINT];

} ROBOT_POSE, *LPROBOT_POSE;

#pragma pack()
