/*    ========================================================================
    =                   Doosan Robot Framework Structure                      =
    =                   Copyright (c) Doosan Robotics.                        =   
    =_______________________________________________________________________  =
    = Title             : Doosan Robot Framwork Structure                     =
    = Author            : Lee Jeong-Woo<jeongwoo1.lee@doosan.com>             =
    = Description       : -                                                   =
    = Version           : 1.0 (GL010105) first release                        =
    =                     1.1 (GF020300) add force control                    =
    =                                    add coordinate sytem control function      =
    =                                    fix GetCurrentTool, GetCurrentTCP function = 
    ======================================================================== */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Doosan Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

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

typedef struct _ROBOT_MONITORING_TASK 
{
    /* Position Actual Value(0: tool, 1: flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* Task Error */
    float                       _fActualErr[NUMBER_OF_JOINT];
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];
    /* Solution Space */
    unsigned char               _iSolutionSpace;
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_TASK, *LPROBOT_MONITORING_TASK;

typedef struct _ROBOT_MONITORING_WORLD
{
    /* world to base relation */
    float                       _fActualW2B[NUMBER_OF_JOINT];
    /* Position Actual Value  (0:tool, 1:flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUMBER_OF_JOINT];    
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];   
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_WORLD, *LPROBOT_MONITORING_WORLD;

typedef struct _ROBOT_MONITORING_USER
{
    /* actual user coord no */
    unsigned char               _iActualUCN;
    /* base: 0 world: 2 */
    unsigned char               _iParent;
    /* Position Actual Value  (0:tool, 1:flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUMBER_OF_JOINT];       
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_USER, *LPROBOT_MONITORING_USER;


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
    ROBOT_MONITORING_TASK       _tTask;
    /* torque */
    ROBOT_MONITORING_TORQUE     _tTorque;  

} MONITORING_CONTROL, *LPMONITORING_CONTROL;

typedef struct _ROBOT_MONITORING_DATA_EX
{
    ROBOT_MONITORING_STATE      _tState;
    /* joint */
    ROBOT_MONITORING_JOINT      _tJoint;
    /* task */
    ROBOT_MONITORING_TASK       _tTask;
    /* torque */
    ROBOT_MONITORING_TORQUE     _tTorque;
    /* world */
    ROBOT_MONITORING_WORLD      _tWorld;
    /*user */
    ROBOT_MONITORING_USER       _tUser;
} ROBOT_MONITORING_DATA_EX, *LPROBOT_MONITORING_DATA_EX;

typedef ROBOT_MONITORING_DATA_EX
    MONITORING_CONTROL_EX, *LPMONITORING_CONTROL_EX;

typedef struct _MONITORING_MISC
{
    /* inner clock counter */
    double                       _dSyncTime;
    /* Flange Digtal Input data */
    unsigned char                _iActualDI[NUM_FLANGE_IO];
    /* Flange Digtal output data */
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

typedef struct _MONITORING_DATA_EX
{
    MONITORING_CONTROL_EX       _tCtrl;
    /* misc. */
    MONITORING_MISC             _tMisc;

} MONITORING_DATA_EX, *LPMONITORING_DATA_EX;

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

typedef struct _READ_CTRLIO_INPUT_EX
{
    /* Digtal Input data */
    unsigned char               _iActualDI[NUM_DIGITAL];
    /* Analog Input data */
    float                       _fActualAI[NUM_ANALOG];
    /* switch input data */
    unsigned char               _iActualSW[NUM_SWITCH];
    /* Safety Input data */
    unsigned char               _iActualSI[NUM_SAFETY_IN];
    /*  Analog Input type */
    unsigned char               _iActualAT[NUM_ANALOG];
    
} READ_CTRLIO_INPUT_EX, *LPREAD_CTRLIO_INPUT_EX;

typedef struct _READ_ENCODER_INPUT
{
    /* Encorder strove signal */
    unsigned char               _iActualES[NUM_ENCORDER];
    /* Encorder raw data */
    unsigned int                _iActualED[NUM_ENCORDER];
    /* Encorder Reset signal */
    unsigned char               _iActualER[NUM_ENCORDER];

} READ_ENCODER_INPUT, *LPREAD_ENCODER_INPUT;

typedef struct _READ_CTRLIO_OUTPUT_EX
{
    /* Digital Output data */
    unsigned char               _iTargetDO[NUM_DIGITAL];
    /* Analog Output data */
    float                       _fTargetAO[NUM_ANALOG];
    /*  Analog Output type */
    unsigned char               _iTargetAT[NUM_ANALOG];

} READ_CTRLIO_OUTPUT_EX, *LPREAD_CTRLIO_OUTPUT_EX;

typedef struct _READ_PROCESS_INPUT
{
    /* Digtal Input data */
    unsigned char               _iActualDI[4];

} READ_PROCESS_INPUT, *LPREAD_PROCESS_INPUT;

typedef struct _MONITORING_CTRLIO
{
    /* input data */
    READ_CTRLIO_INPUT           _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT          _tOutput;
} MONITORING_CTRLIO, *LPMONITORING_CTRLIO;


typedef struct _MONITORING_CTRLIO_EX
{
    /* input data */
    READ_CTRLIO_INPUT_EX        _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT_EX       _tOutput;
    /* input encoder data*/
    READ_ENCODER_INPUT          _tEncoder;
    /* reserved data */
    unsigned char               _szReserved[24];
} MONITORING_CTRLIO_EX, *LPMONITORING_CTRLIO_EX;

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

typedef struct _MESSAGE_PROGRESS
{
    /* current step */
    unsigned char               _iCurrentCount;
    /*  total step  */
    unsigned char               _iTotalCount;
} MESSAGE_PROGRESS, *LPMESSAGE_PROGRESS;

typedef struct _MESSAGE_POPUP
{
    /* message string */
    char                        _szText[MAX_STRING_SIZE];
    /*  Message: 0, Warning: 1, Alarm: 2 */
    unsigned char               _iLevel;
    /*  resuem and stop : 0, ok : 1 */
    unsigned char               _iBtnType;

} MESSAGE_POPUP, *LPMESSAGE_POPUP;

typedef MESSAGE_POPUP MONITORING_POPUP;

typedef struct _MESSAGE_INPUT
{
    /* message string */
    char                        _szText[MAX_STRING_SIZE];
    /*  int: 0, float: 1, string: 2 , bool: 3*/
    unsigned char               _iType;
} MESSAGE_INPUT, *LPMESSAGE_INPUT;

typedef MESSAGE_INPUT MONITORING_INPUT;

typedef struct _CONTROL_BRAKE
{
    /* joint: 0~5 , all joint: 6 */
    unsigned char               _iTargetAxs;
    /*  on: 1 off: 0 */
    unsigned char               _bValue;
} CONTROL_BRAKE, *LPCONTROL_BRAKE;

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

typedef struct _ROBOT_VEL
{
    /* current velocity */
    float                       _fVelocity[NUM_JOINT];
} ROBOT_VEL, *LPROBOT_VEL;

typedef struct _ROBOT_FORCE
{
    /* current force */
    float                       _fForce[NUM_JOINT];
} ROBOT_FORCE, *LPROBOT_FORCE;

typedef struct _ROBOT_TASK_POSE
{
    /* target pose */
    float                       _fTargetPos[NUM_TASK];
    /*  solution space: 0 ~ 7 */
    unsigned char               _iTargetSol;

} ROBOT_TASK_POSE, *LPROBOT_TASK_POSE;

typedef struct _USER_COORDINATE
{
    unsigned char _iReqId;
    unsigned char _iTargetRef;
    float _fTargetPos[NUM_TASK];
} USER_COORDINATE, *LPUSER_COORDINATE;

typedef struct _MEASURE_TOOL_RESPONSE
{
    /* mass(kg) */
    float                       _fWeight;
    /* center of mass */
    float                       _fXYZ[3];
    
} MEASURE_TOOL_RESPONSE, *LPMEASURE_TOOL_RESPONSE;

typedef struct _CONFIG_TCP
{
    /* target pose */
    float                       _fTargetPos[NUM_JOINT];

} CONFIG_TCP, *LPCONFIG_TCP;

typedef struct _CONFIG_TOOL
{
    /* mass(kg) */
    float                       _fWeight;
    /* center of mass */
    float                       _fXYZ[3];
    /* inertia */
    float                       _fInertia[NUMBER_OF_JOINT];    

} CONFIG_TOOL, *LPCONFIG_TOOL;

typedef struct _MEASURE_TCP_RESPONSE
{
    /* mesure pose  */
    CONFIG_TCP                  _tTCP;
    /* mesure error */
    float                       _fError;

} MEASURE_TCP_RESPONSE, *LPMEASURE_TCP_RESPONSE;

typedef struct _FLANGE_SERIAL_DATA
{
    unsigned char              _iCommand;       // 0 : OPEN, 1: CLOSE
                                                // 2 : SEND, 3: RECV
    union {
        struct {
            /* Baudrate */
            unsigned char               _szBaudrate[7]; // "0115200"(ASCII)
            /* Data Length */
            unsigned char               _szDataLength;  // 0: 1bit, 7: 8bit
            /* Parity */
            unsigned char               _szParity;      // 0x00 : Non-parity, 0x01 : Odd, 0x02 : Even
            /* Stop Bit */
            unsigned char               _szStopBit;     // 0x01 : One stop bit, 0x02 : Two Stop bit

        } _tConfig;

        //unsigned char                   _szConfigData[10];

        struct {
            unsigned short             _iLength;

            unsigned char              _szValue[MAX_SYMBOL_SIZE];

        } _tValue;
    } _tData;
} FLANGE_SERIAL_DATA, *LPFLANGE_SERIAL_DATA;

typedef struct _FLANGE_SER_RXD_INFO
{
    /* size of serial data */
    short                               _iSize;      //max 256bytes
    /* raw serial data */
    unsigned char                       _cRxd[256];

} FLANGE_SER_RXD_INFO, *LPFLANGE_SER_RXD_INFO;

typedef struct _READ_FLANGE_SERIAL
{
    // check ready to read
    unsigned char               _bRecvFlag;    // 0: non-receive, 1: received
} READ_FLANGE_SERIAL, *LPREAD_FLANGE_SERIAL;

typedef struct _INVERSE_KINEMATIC_RESPONSE
{
    /* target pose */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* status */
    int                         _iStatus;
} INVERSE_KINEMATIC_RESPONSE, *LPINVERSE_KINEMATIC_RESPONSE;

typedef struct _RT_INPUT_DATA_LIST
{
    /* External Force Torque */
    float                       _fExternalForceTorque[NUM_JOINT];
    /* External Digital Input */
    unsigned short              _iExternalDI;
    /* External Digital Output */
    unsigned short              _iExternalDO;
    /* external analog input(6 channel) */
    float                       _fExternalAnalogInput[6];
    /* external analog output(6 channel) */
    float                       _fExternalAnalogOutput[6];
    /* Reserved */
    unsigned char               _iReserved[256];
} RT_INPUT_DATA_LIST, *LPRT_INPUT_DATA_LIST;

typedef struct _RT_OUTPUT_DATA_LIST
{
    /* timestamp at the data of data acquisition */
    double                      time_stamp;
    /* actual joint position from incremental encoder at motor side(used for control) [deg] */
    float                       actual_joint_position[NUMBER_OF_JOINT];
    /* actual joint position from absolute encoder at link side (used for exact link position) [deg] */
    float                       actual_joint_position_abs[NUMBER_OF_JOINT];
    /* actual joint velocity from incremental encoder at motor side [deg/s] */
    float                       actual_joint_velocity[NUMBER_OF_JOINT];
    /* actual joint velocity from absolute encoder at link side [deg/s] */
    float                       actual_joint_velocity_abs[NUMBER_OF_JOINT];
    /* actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       actual_tcp_position[NUM_TASK];
    /* actual robot tcp velocity w.r.t. base coordinates [mm, deg/s] */
    float                       actual_tcp_velocity[NUMBER_OF_TASK];
    /* actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       actual_flange_position[NUMBER_OF_TASK];
    /* robot flange velocity w.r.t. base coordinates [mm, deg/s] */
    float                       actual_flange_velocity[NUMBER_OF_TASK];
    /* actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm] */
    float                       actual_motor_torque[NUMBER_OF_JOINT];
    /* estimated joint torque by robot controller [Nm] */
    float                       actual_joint_torque[NUMBER_OF_JOINT];
    /* calibrated joint torque sensor data [Nm] */
    float                       raw_joint_torque[NUMBER_OF_JOINT];
    /* calibrated force torque sensor data w.r.t. flange coordinates [N, Nm] */
    float                       raw_force_torque[NUMBER_OF_JOINT];
    /* estimated external joint torque [Nm] */
    float                       external_joint_torque[NUMBER_OF_JOINT];
    /* estimated tcp force w.r.t. base coordinates [N, Nm] */
    float                       external_tcp_force[NUMBER_OF_TASK];
    /* target joint position [deg] */
    float                       target_joint_position[NUMBER_OF_JOINT];
    /* target joint velocity [deg/s] */
    float                       target_joint_velocity[NUMBER_OF_JOINT];
    /* target joint acceleration [deg/s^2] */
    float                       target_joint_acceleration[NUMBER_OF_JOINT];
    /* target motor torque [Nm] */
    float                       target_motor_torque[NUMBER_OF_JOINT];
    /* target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       target_tcp_position[NUMBER_OF_TASK];
    /* target tcp velocity w.r.t. base coordinates [mm, deg/s] */
    float                       target_tcp_velocity[NUMBER_OF_TASK];
    /* jacobian matrix=J(q) w.r.t. base coordinates */
    float                       jacobian_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* gravity torque=g(q) [Nm] */
    float                       gravity_torque[NUMBER_OF_JOINT];
    /* coriolis matrix=C(q,q_dot)  */
    float                       coriolis_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* mass matrix=M(q) */
    float                       mass_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* robot configuration */
    unsigned short              solution_space;
    /* minimum singular value */
    float                       singularity;
    /* current operation speed rate(1~100 %) */
    float                       operation_speed_rate;
    /* joint temperature(celsius) */
    float                       joint_temperature[NUMBER_OF_JOINT];
    /* controller digital input(16 channel) */
    unsigned short              controller_digital_input;
    /* controller digital output(16 channel) */
    unsigned short              controller_digital_output;
    /* controller analog input type(2 channel) */
    unsigned char               controller_analog_input_type[2];
    /* controller analog input(2 channel) */
    float                       controller_analog_input[2];
    /* controller analog output type(2 channel) */
    unsigned char               controller_analog_output_type[2];
    /* controller analog output(2 channel) */
    float                       controller_analog_output[2];
    /* flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) */
    unsigned char               flange_digital_input;
    /* flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) */
    unsigned char               flange_digital_output;
    /* flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) */
    float                       flange_analog_input[4];
    /* strobe count(increased by 1 when detecting setting edge) */
    unsigned char               external_encoder_strobe_count[2];
    /* external encoder count */
    unsigned int                external_encoder_count[2];
    /* final goal joint position (reserved) */
    float                       goal_joint_position[NUMBER_OF_JOINT];
    /* final goal tcp position (reserved) */
    float                       goal_tcp_position[NUMBER_OF_TASK];
    /* ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2) */
    unsigned char               robot_mode;
    /* STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, */
    unsigned char               robot_state;
    /* position control mode, torque mode */
    unsigned short              control_mode;
    /* Reserved */
    unsigned char               reserved[256];
    
} RT_OUTPUT_DATA_LIST, *LPRT_OUTPUT_DATA_LIST;

typedef struct _UPDATE_SW_MODULE_RESPONSE{
    unsigned char               _bStatus;
    char                        _szModuleInform[2048];
} UPDATE_SW_MODULE_RESPONSE, *LPUPDATE_SW_MODULE_RESPONSE;

typedef struct _JOINT_RANGE
{
    // max velocity
    float                       _fMaxVelocity[NUMBER_OF_JOINT];
    // max range
    float                       _fMaxRange[NUMBER_OF_JOINT];
    // min range
    float                       _fMinRange[NUMBER_OF_JOINT];
} JOINT_RANGE, *LPJOINT_RANGE;

typedef struct _CONFIG_JOINT_RANGE
{
    /* noraml mode */
    JOINT_RANGE                 _Normal;
    /* reduced mode */
    JOINT_RANGE                 _Reduced;

} CONFIG_JOINT_RANGE, *LPCONFIG_JOINT_RANGE;

typedef struct _GENERAL_RANGE
{
    // max force
    float                       _fMaxForce;
    // max power
    float                       _fMaxPower;
    // max speed
    float                       _fMaxSpeed;
    // max momenturm
    float                       _fMaxMomentum;
} GENERAL_RANGE, *LPGENERAL_RANGE;

typedef struct _CONFIG_GENERAL_RANGE
{
    /* noraml mode */
    GENERAL_RANGE               _Normal;
    /* reduced mode */
    GENERAL_RANGE               _Reduced;

} CONFIG_GENERAL_RANGE, *LPCONFIG_GENERAL_RANGE;

typedef struct _POINT_2D
{
    float                         _fXPos;
    float                         _fYPos;
} POINT_2D, *LPPOINT_2D;

typedef struct _POINT_3D
{
    float           _fXPos;
    float           _fYPos;
    float           _fZPos;
} POINT_3D, *LPPOINT_3D;

typedef struct _LINE
{
    POINT_2D            _tFromPoint;
    POINT_2D            _tToPoint;
} LINE, *LPLINE;

typedef union _CONFIG_SAFETY_FUNCTION
{
    /* SF05. Emergency Stop */
    /* SF06. Proctective Stop */
    /* SF07. StadnStill Monitoring */
    /* SF08. Joint Angle Monitoring */
    /* SF09. Joint Speed Monitoring */
    /* SF10. Joint Torque Monitoring*/
    /* SF11. Collisoin Detection */
    /* SF12. TCP Posiiont Monitoring */
    /* SF13. TCP Orientation Monitoring */
    /* SF14. TCP Speed Monitoring */
    /* SF15. TCP Force Monitoring */
    /* SF16. Momentum Monitoring */
    /* SF17. Power Mon. */

    struct {
        /* Standalone Workspace */
        unsigned char           _iStdWorkSpace:4;
        /* Collaborative Workspace */
        unsigned char           _iColWorkSpace:4;
    } _tStopCode[SAFETY_FUNC_LAST];

    unsigned char               _iStopCode[SAFETY_FUNC_LAST];

} CONFIG_SAFETY_FUNCTION, *LPCONFIG_SAFETY_FUNCTION;

typedef struct _CONFIG_INSTALL_POSE
{
    /* robot gradinet on ground  */
    float                       _fGradient;
    /* robot rotation angle */
    float                       _fRotation;

} CONFIG_INSTALL_POSE, *LPCONFIG_INSTALL_POSE;

typedef struct _CONFIG_SAFETY_IO
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_SAFETY];

} CONFIG_SAFETY_IO, *LPCONFIG_SAFETY_IO;

typedef struct _CONFIG_SAFETY_IO_EX
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_SAFETY];
    /* trigger level */
    unsigned char               _bLevel[TYPE_LAST][NUM_SAFETY];

} CONFIG_SAFETY_IO_EX, *LPCONFIG_SAFETY_IO_EX;

typedef union _VIRTUAL_FENCE_OBJECT
{
    struct _CUBE {
        float                   _fXLoLimit;
        float                   _fXUpLimit;
        float                   _fYLoLimit;
        float                   _fYUpLimit;
        float                   _fZLoLimit;
        float                   _fZUpLimit;
    } _tCube;

    struct _POLYGON {
        unsigned char           _iLineCount;
        LINE                    _tLine[6];
        float                   _fZLoLimit;
        float                   _fZUpLimit;
    } _tPolygon;

    struct _CYLINDER {
        float                   _fRadius;
        float                   _fZLoLimit;
        float                   _fZUpLimit;
    } _tCylinder;

    unsigned char               _iBuffer[110];
} VIRTUAL_FENCE_OBJECT, *LPVIRTUAL_FENCE_OBJECT;

typedef struct _CONFIG_VIRTUAL_FENCE
{
    unsigned char               _iTargetRef;
    /* fence type - cube: 0, polygon: 1, cylinder: 2 */
    unsigned char               _iFenceType;
    /* fence objec */
    VIRTUAL_FENCE_OBJECT        _tFenceObject;
} CONFIG_VIRTUAL_FENCE, *LPCONFIG_VIRTUAL_FENCE;

typedef struct _CONFIG_SAFE_ZONE
{
    unsigned char               _iTargetRef;
    LINE                        _tLine[2];
    POINT_2D                    _tPoint[3];
} CONFIG_SAFE_ZONE, *LPCONFIG_SAFE_ZONE;

typedef struct _ENABLE_SAFE_ZONE
{
    /* region enable */
    unsigned char                 _iRegion[3];
} ENABLE_SAFE_ZONE, *LPENABLE_SAFE_ZONE;

typedef struct _SAFETY_OBJECT_SPHERE
{
    /* radius */
    float                   _fRadius;
    /* vertex: center */
    POINT_3D                _tTargetPos;
} SAFETY_OBJECT_SPHERE, *LPSAFETY_OBJECT_SPHERE;

typedef struct _SAFETY_OBJECT_CAPSULE
{
    /* radius */
    float                   _fRadius;
    /* vertex: low(0), high(1) */
    POINT_3D                _tTargetPos[2];

} SAFETY_OBJECT_CAPSULE, *LPSAFETY_OBJECT_CAPSULE;

typedef struct _SAFETY_OBJECT_CUBE
{
    /*  vertex: low(0), high(1)  */
    POINT_3D                _tTargetPos[2];

} SAFETY_OBJECT_CUBE, *LPSAFETY_OBJECT_CUBE;

typedef struct _SAFETY_OBJECT_OBB
{
    /* vetext: std(0), x-end(1), y-end(2), z-end(3) */
    POINT_3D                _tTargetPos[4];

} SAFETY_OBJECT_OBB, *LPSAFETY_OBJECT_OBB;

typedef struct _SAFETY_OBJECT_POLYPRISM
{
    unsigned char           _iPointCount;
    /* x,y coordinates of polygon vertices */
    POINT_2D                _tPoint[10];
    /* min z */
    float                   _fZLoLimit;
    /* max z */
    float                   _fZUpLimit;

} SAFETY_OBJECT_POLYPRISM, *LPSAFETY_OBJECT_POLYPRISM;

typedef union _SAFETY_OBJECT_DATA
{
    SAFETY_OBJECT_SPHERE        _tSphere;
    SAFETY_OBJECT_CAPSULE       _tCapsule;
    SAFETY_OBJECT_CUBE          _tCube;
    SAFETY_OBJECT_OBB           _tOBB;
    SAFETY_OBJECT_POLYPRISM     _tPolyPrism;
    unsigned char               _iBuffer[100];
} SAFETY_OBJECT_DATA, *LPSAFETY_OBJECT_DATA;

typedef struct _SAFETY_OBJECT
{
    unsigned char               _iTargetRef;
    /*
        geometry object type :
        0(Sphere), 1(Capsule), 2(Cube), 3(Oriented Box), 4(Polygon-Prism)
    */
    unsigned char                   _iObjectType;
    /* geometry object  data */
    SAFETY_OBJECT_DATA              _tObject;

} SAFETY_OBJECT, *LPSAFETY_OBJECT;

typedef struct _CONFIG_PROTECTED_ZONE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[10];
    /* safety object */
    SAFETY_OBJECT                   _tZone[10];

} CONFIG_PROTECTED_ZONE, *LPCONFIG_PROTECTED_ZONE;

typedef struct _CONFIG_COLLISION_MUTE_ZONE_PROPERTY
{
    char                            _szIdentifier[32];
    unsigned char                   _iOnOff;
#ifdef INFRACORE_PATCH        
    unsigned char                   _iSafetyIO;
#endif
    float                           _fSensitivity;
    SAFETY_OBJECT                   _tZone;
} CONFIG_COLLISION_MUTE_ZONE_PROPERTY, *LPCONFIG_COLLISION_MUTE_ZONE_PROPERTY;

typedef struct _CONFIG_COLLISION_MUTE_ZONE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[10];
    /* safety object */
    CONFIG_COLLISION_MUTE_ZONE_PROPERTY _tProperty[10];

} CONFIG_COLLISION_MUTE_ZONE, *LPCONFIG_COLLISION_MUTE_ZONE;

typedef struct _SAFETY_TOOL_ORIENTATION_LIMIT
{
    /* direction */
    POINT_3D                        _tTargetDir;
    /* angle (degree) */
    float                           _fTargetAng;

} SAFETY_TOOL_ORIENTATION_LIMIT, *LPSAFETY_TOOL_ORIENTATION_LIMIT;

typedef struct _CONFIG_TOOL_ORIENTATION_LIMIT_ZONE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[10];
    /* safety object */
    SAFETY_OBJECT                   _tZone[10];
    /* orientation limit for each zone */
    SAFETY_TOOL_ORIENTATION_LIMIT   _tLimit[10];

} CONFIG_TOOL_ORIENTATION_LIMIT_ZONE, *LPCONFIG_TOOL_ORIENTATION_LIMIT_ZONE;

typedef struct _CONFIG_NUDGE
{
    /* enalbe/disalbe */
    unsigned char               _bEnable;
    /* input force */
    float                       _fInputForce;
    /* delay time */
    float                       _fDelayTime;

} CONFIG_NUDGE, *LPCONFIG_NUDGE;

typedef struct _CONFIG_COCKPIT_EX
{
    /* disalbe: 0, enable: 1*/
    unsigned char               _bEnable;
    /* Direct Teach: 0, TCP-Z: 1, TCP-XY: 2, Orientation Only: 3, Positon Only: 4 */
    unsigned char               _iButton[2];
    /* disalbe: 0, enable: 1*/
    unsigned char               _bRecoveryTeach;

} CONFIG_COCKPIT_EX, *LPCONFIG_COCKPIT_EX;

typedef struct _CONFIG_IDLE_OFF
{
    /* function enable */
    unsigned char        _bFuncEnable;
    /* elapse time */
    float               _fElapseTime;

} CONFIG_IDLE_OFF, *LPCONFIG_IDLE_OFF;

typedef struct _WRITE_MODBUS_DATA
{
    /* symbol name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* tcp address */
    char                        _szIpAddr[16];
    /* tcp port */
    unsigned short              _iPort;
    /* Slave ID*/
    int                         _iSlaveID;
    /* i/o type */
    unsigned char               _iRegType;
    /* register address */
    unsigned short              _iRegIndex;
    /* register value */
    unsigned short              _iRegValue;

} WRITE_MODBUS_DATA, *LPWRITE_MODBUS_DATA;

typedef WRITE_MODBUS_DATA
    WRITE_MODBUS_TCP_DATA, *LPWRITE_MODBUS_TCP_DATA;

typedef struct _WRITE_MODBUS_RTU_DATA
{
    /* symbol name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* tty device */
    char                        _szttyPort[16];
    /* Slave ID*/
    int                         _iSlaveID;
    /* Baud Rate */
    int                         _iBaudRate;
    /* Byte size */
    int                         _iByteSize;
    /* Parity */
    char                        _szParity;
    /* Stop bit */
    int                         _iStopBit;
    /* i/o type */
    unsigned char               _iRegType;
    /* register address */
    unsigned short              _iRegIndex;
    /* register value */
    unsigned short              _iRegValue;

} WRITE_MODBUS_RTU_DATA, *LPWRITE_MODBUS_RTU_DATA;

typedef struct _MODBUS_DATA {

    /*0: TCP, 1: RTU */
    unsigned char _iType;

    union {
        /* modbus tcp */
        WRITE_MODBUS_TCP_DATA _tcp;
        /* modbus rtu */
        WRITE_MODBUS_RTU_DATA _rtu;
        /* data buffer */
        unsigned char _szBuffer[70];
    } _tData;

} MODBUS_DATA, * LPMODBUS_DATA;

typedef struct _MODBUS_DATA_LIST
{
    /* modbus count */
    unsigned short              _nCount;
    /* modbus data*/
    MODBUS_DATA                 _tRegister[MAX_MODBUS_TOTAL_REGISTERS];

} MODBUS_DATA_LIST, *LPMODBUS_DATA_LIST;

typedef struct _CONFIG_WORLD_COORDINATE
{
    /* 설정타입: world2base: 0, base2ref: 1, world2ref: 2 */
    /* 설정여부: 미설정: 0, 설정: 1*/
    unsigned char               _iType;
    /* target pose */
    float                       _fPosition[NUMBER_OF_JOINT];

} CONFIG_WORLD_COORDINATE, *LPCONFIG_WORLD_COORDINATE;

typedef struct _CONFIG_CONFIGURABLE_IO
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_DIGITAL];

} CONFIG_CONFIGURABLE_IO, *LPCONFIG_CONFIGURABLE_IO;

typedef struct _CONFIG_CONFIGURABLE_IO_EX
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_DIGITAL];
    /* trigger level */
    unsigned char               _bLevel[TYPE_LAST][NUM_DIGITAL];

} CONFIG_CONFIGURABLE_IO_EX, *LPCONFIG_CONFIGURABLE_IO_EX;

typedef struct _CONFIG_TOOL_SHAPE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[5];
    /* safety object */
    SAFETY_OBJECT                   _tShape[5];

} CONFIG_TOOL_SHAPE, *LPCONFIG_TOOL_SHAPE;

typedef struct _CONFIG_TOOL_SYMBOL
{
    /* tool name */
    char                _szSymbol[MAX_SYMBOL_SIZE];
    /* tool data */
    CONFIG_TOOL         _tTool;

} CONFIG_TOOL_SYMBOL, *LPCONFIG_TOOL_SYMBOL;

typedef struct _CONFIG_TCP_SYMBOL
{
    /* tcp name */
    char                _szSymbol[MAX_SYMBOL_SIZE];
    /* tcp data */
    CONFIG_TCP          _tTCP;

} CONFIG_TCP_SYMBOL, *LPCONFIG_TCP_SYMBOL;

typedef struct _CONFIG_TOOL_LIST
{
    int                         _iToolCount;
    CONFIG_TOOL_SYMBOL          _tTooList[MAX_CONFIG_TOOL_SIZE];
} CONFIG_TOOL_LIST, *LPCONFIG_TOOL_LIST;

typedef struct _CONFIG_TOOL_SHAPE_SYMBOL
{
    char                _szSymbol[MAX_SYMBOL_SIZE];
    CONFIG_TOOL_SHAPE   _tToolShape;
} CONFIG_TOOL_SHAPE_SYMBOL, *LPCONFIG_TOOL_SHAPE_SYMBOL;

typedef struct _CONFIG_TCP_LIST
{
    int                         _iToolCount;
    CONFIG_TCP_SYMBOL           _tTooList[MAX_CONFIG_TCP_SIZE];
} CONFIG_TCP_LIST, *LPCONFIG_TCP_LIST;

typedef struct _CONFIG_TOOL_SHAPE_LIST
{
    int                         _iToolCount;
    CONFIG_TOOL_SHAPE_SYMBOL    _tTooList[MAX_CONFIG_TOOL_SIZE];
} CONFIG_TOOL_SHAPE_LIST, *LPCONFIG_TOOL_SHAPE_LIST;

typedef struct _SAFETY_CONFIGURATION_EX
{
    unsigned int _iDataVersion;
    CONFIG_JOINT_RANGE _tJointRange;
    CONFIG_GENERAL_RANGE _tGeneralRange;
    float _fCollisionSensitivity;
    CONFIG_SAFETY_FUNCTION _tSafetyFunc;
    CONFIG_TOOL_SYMBOL _tTool;
    CONFIG_TCP_SYMBOL _tTcp;
    CONFIG_INSTALL_POSE _tInstallPose;
    CONFIG_SAFETY_IO _tSafetyIO;
    //CONFIG_SAFETY_IO_EX         _tSafetyIO;

    CONFIG_VIRTUAL_FENCE _tSafetySpaceVF;
    CONFIG_SAFE_ZONE _tSafetySpaceSZ;
    ENABLE_SAFE_ZONE _tSafetySpaceESZ;
    CONFIG_PROTECTED_ZONE _tSafetySpacePZ;
    CONFIG_COLLISION_MUTE_ZONE _tSafetySpaceCM;
    CONFIG_TOOL_ORIENTATION_LIMIT_ZONE _tSafetySpaceTO;
    CONFIG_TOOL_SHAPE _tSafetySpaceTS;

    CONFIG_NUDGE _tConfigNudge;
    CONFIG_COCKPIT_EX _tCockPit;
    CONFIG_IDLE_OFF _tIdleOff;
    CONFIG_TCP_LIST _tConfigTCP;
    CONFIG_TOOL_LIST _tConfigTool;
    CONFIG_TOOL_SHAPE_LIST _tConfigToolShape;

    char                _szActiveTcp[MAX_SYMBOL_SIZE];
    char                _szActiveTool[MAX_SYMBOL_SIZE];
    char                _szActiveToolShape[MAX_SYMBOL_SIZE];

    MODBUS_DATA_LIST _tModbusList;
    CONFIG_WORLD_COORDINATE     _tWorld2BaseRelation;
    float m_CwsSpeedRatio;
    float m_IoSpeedRatio;

#ifdef _UNIFY_SAFETY_ZONE
    int _iSafetyZoneCount;
    CONFIG_SAFETY_ZONE _tSafetyZone[20];
#endif

#ifdef _FUNC_USER_COORDINATE
    int _iUserCoordCount;
    CONFIG_USER_COORDINATE _tUserCoordinates[20];
#endif
    CONFIG_CONFIGURABLE_IO _tConfigurableIO;

} SAFETY_CONFIGURATION_EX, *LPSAFETY_CONFIGURATION_EX;

#pragma pack()
