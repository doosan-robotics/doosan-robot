/*
 *  Inferfaces for doosan robot controllor 
  * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include "dsr_control/dsr_hw_interface.h"
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <sstream>

CDRFLEx Drfl;
Serial_comm ser_comm;

bool g_bHasControlAuthority = FALSE;
bool g_bTpInitailizingComplted = FALSE;
bool g_bHommingCompleted = FALSE;

DR_STATE    g_stDrState;
DR_ERROR    g_stDrError;

int g_nAnalogOutputModeCh1;
int g_nAnalogOutputModeCh2;


#define STABLE_BAND_JNT     0.05
#define DSR_CTL_PUB_RATE    100  //[hz] 10ms <----- 퍼블리싱 주기, but OnMonitoringDataCB() 은 100ms 마다 불려짐을 유의!   

namespace dsr_control{

    const char* GetRobotStateString(int nState)
    {
        switch(nState)
        {
        case STATE_INITIALIZING:    return "(0) INITIALIZING";
        case STATE_STANDBY:         return "(1) STANDBY";
        case STATE_MOVING:          return "(2) MOVING";
        case STATE_SAFE_OFF:        return "(3) SAFE_OFF";
        case STATE_TEACHING:        return "(4) TEACHING";
        case STATE_SAFE_STOP:       return "(5) SAFE_STOP";
        case STATE_EMERGENCY_STOP:  return "(6) EMERGENCY_STOP";
        case STATE_HOMMING:         return "(7) HOMMING";
        case STATE_RECOVERY:        return "(8) RECOVERY";
        case STATE_SAFE_STOP2:      return "(9) SAFE_STOP2";
        case STATE_SAFE_OFF2:       return "(10) SAFE_OFF2";
        case STATE_RESERVED1:       return "(11) RESERVED1";
        case STATE_RESERVED2:       return "(12) RESERVED2";
        case STATE_RESERVED3:       return "(13) RESERVED3";
        case STATE_RESERVED4:       return "(14) RESERVED4";
        case STATE_NOT_READY:       return "(15) NOT_READY";

        default:                  return "UNKNOWN";
        }
        return "UNKNOWN";
    }

    int IsInposition(double dCurPosDeg[], double dCmdPosDeg[])
    {
        int cnt=0;
        double dError[NUM_JOINT] ={0.0, };

        for(int i=0;i<NUM_JOINT;i++)
        {
            dError[i] = dCurPosDeg[i] - dCmdPosDeg[i];
            ROS_INFO("<inpos> %f = %f -%f",dError[i], dCurPosDeg[i], dCmdPosDeg[i]);
            if(fabs(dError[i]) < STABLE_BAND_JNT)
                cnt++;
        }
        if(NUM_JOINT == cnt)
            return true;
        else 
            return false;
    }

    //----- register the call-back functions ----------------------------------------
    void DRHWInterface::OnTpInitializingCompletedCB()
    {
        // request control authority after TP initialized
        cout << "[callback OnTpInitializingCompletedCB] tp initializing completed" << endl;
        g_bTpInitailizingComplted = TRUE;
        //Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
        Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

        g_stDrState.bTpInitialized = TRUE;
    }

    void DRHWInterface::OnHommingCompletedCB()
    {
        g_bHommingCompleted = TRUE;
        // Only work within 50msec
        cout << "[callback OnHommingCompletedCB] homming completed" << endl;

        g_stDrState.bHommingCompleted = TRUE;
    }

    void DRHWInterface::OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause)
    {
        cout << "[callback OnProgramStoppedCB] Program Stop: " << (int)iStopCause << endl;
        g_stDrState.bDrlStopped = TRUE;
    }
    // M2.4 or lower
    void DRHWInterface::OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO)
    {
        for (int i = 0; i < NUM_DIGITAL; i++){
            if(pCtrlIO){  
                g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
                g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
            }
        }
    }
    // M2.5 or higher
    void DRHWInterface::OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO) 
    {
        //ROS_INFO("DRHWInterface::OnMonitoringCtrlIOExCB");

        for (int i = 0; i < NUM_DIGITAL; i++){
            if(pCtrlIO){  
                g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
                g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
            }
        }

        //----- In M2.5 version or higher The following variables were added -----
        for (int i = 0; i < 3; i++)
            g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

        for (int i = 0; i < 2; i++){
            g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
            g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
            g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
            g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
            g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
            g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
            g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
            g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
        }  
        //-------------------------------------------------------------------------
    }

    // M2.4 or lower
    void DRHWInterface::OnMonitoringDataCB(const LPMONITORING_DATA pData)
    {
        // This function is called every 100 msec
        // Only work within 50msec
        //ROS_INFO("DRHWInterface::OnMonitoringDataCB");

        g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
        g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

        for (int i = 0; i < NUM_JOINT; i++){
            if(pData){  
                // joint         
                g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
                g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
                g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
                g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
                g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
                g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
                // task
                g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
                g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
                g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
                g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
                // Torque
                g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
                g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
                g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
                g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

                g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
                g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
                g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
            }
        }
        g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
        g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

        for (int i = 5; i < NUM_BUTTON; i++){
            if(pData){
                g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
            }
        }

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                if(pData){
                    g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
                }
            }
        }

        for (int i = 0; i < NUM_FLANGE_IO; i++){
            if(pData){
                g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
                g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
            }
        }
    }

    // M2.5 or higher    
    void DRHWInterface::OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
    {
        // This function is called every 100 msec
        // Only work within 50msec
        //ROS_INFO("DRHWInterface::OnMonitoringDataExCB");

        g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
        g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

        for (int i = 0; i < NUM_JOINT; i++){
            if(pData){  
                // joint         
                g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
                g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
                g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
                g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
                g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
                g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
                // task
                g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
                g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
                g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
                g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
                // Torque
                g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
                g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
                g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
                g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

                g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
                g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
                g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
            }
        }
        g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
        g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

        for (int i = 5; i < NUM_BUTTON; i++){
            if(pData){
                g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
            }
        }

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                if(pData){
                    g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
                }
            }
        }

        for (int i = 0; i < NUM_FLANGE_IO; i++){
            if(pData){
                g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
                g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
            }
        }

        //----- In M2.5 version or higher The following variables were added -----
        for (int i = 0; i < NUM_JOINT; i++){
            g_stDrState.fActualW2B[i] = pData->_tCtrl._tWorld._fActualW2B[i];
            g_stDrState.fCurrentVelW[i] = pData->_tCtrl._tWorld._fActualVel[i];
            g_stDrState.fWorldETT[i] = pData->_tCtrl._tWorld._fActualETT[i];
            g_stDrState.fTargetPosW[i] = pData->_tCtrl._tWorld._fTargetPos[i];
            g_stDrState.fTargetVelW[i] = pData->_tCtrl._tWorld._fTargetVel[i];
            g_stDrState.fCurrentVelU[i] = pData->_tCtrl._tWorld._fActualVel[i];
            g_stDrState.fUserETT[i] = pData->_tCtrl._tUser._fActualETT[i];
            g_stDrState.fTargetPosU[i] = pData->_tCtrl._tUser._fTargetPos[i];
            g_stDrState.fTargetVelU[i] = pData->_tCtrl._tUser._fTargetVel[i];
        }    

        for(int i = 0; i < 2; i++){
            for(int j = 0; j < 6; j++){
                g_stDrState.fCurrentPosW[i][j] = pData->_tCtrl._tWorld._fActualPos[i][j];
                g_stDrState.fCurrentPosU[i][j] = pData->_tCtrl._tUser._fActualPos[i][j];
            }
        }

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                g_stDrState.fRotationMatrixWorld[j][i] = pData->_tCtrl._tWorld._fRotationMatrix[j][i];
                g_stDrState.fRotationMatrixUser[j][i] = pData->_tCtrl._tUser._fRotationMatrix[j][i];
            }
        }

        g_stDrState.iActualUCN = pData->_tCtrl._tUser._iActualUCN;
        g_stDrState.iParent    = pData->_tCtrl._tUser._iParent;
        //-------------------------------------------------------------------------
    }

    void DRHWInterface::OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus)
    {
        g_stDrState.nRegCount = pModbus->_iRegCount;
        for (int i = 0; i < pModbus->_iRegCount; i++){
            cout << "[callback OnMonitoringModbusCB] " << pModbus->_tRegister[i]._szSymbol <<": " << pModbus->_tRegister[i]._iValue<< endl;
            g_stDrState.strModbusSymbol[i] = pModbus->_tRegister[i]._szSymbol;
            g_stDrState.nModbusValue[i]    = pModbus->_tRegister[i]._iValue;
        }
    }

    void DRHWInterface::OnMonitoringStateCB(const ROBOT_STATE eState)
    {
        //This function is called when the state changes.
        //ROS_INFO("DRHWInterface::OnMonitoringStateCB");    
        // Only work within 50msec
        ROS_INFO("On Monitor State");
        switch((unsigned char)eState)
        {
#if 0 // TP initializing logic, Don't use in API level. (If you want to operate without TP, use this logic)       
        case eSTATE_NOT_READY:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
            break;
        case eSTATE_INITIALIZING:
            // add initalizing logic
            if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
            break;
#endif      
        case STATE_EMERGENCY_STOP:
            // popup
            break;
        case STATE_STANDBY:
        case STATE_MOVING:
        case STATE_TEACHING:
            break;
        case STATE_SAFE_STOP:
            if (g_bHasControlAuthority) {
                Drfl.set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT);
                Drfl.set_robot_control(CONTROL_RESET_SAFET_STOP);
            }
            break;
        case STATE_SAFE_OFF:
            if (g_bHasControlAuthority){
                Drfl.set_robot_control(CONTROL_SERVO_ON);
				Drfl.set_robot_mode(ROBOT_MODE_MANUAL);   //Idle Servo Off 후 servo on 하는 상황 발생 시 set_robot_mode 명령을 전송해 manual 로 전환. add 2020/04/28
            } 
            break;
        case STATE_SAFE_STOP2:
            if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_STOP);
            break;
        case STATE_SAFE_OFF2:
            if (g_bHasControlAuthority) {
                Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_OFF);
            }
            break;
        case STATE_RECOVERY:
            Drfl.set_robot_control(CONTROL_RESET_RECOVERY);
            break;
        default:
            break;
        }

        cout << "[callback OnMonitoringStateCB] current state: " << GetRobotStateString((int)eState) << endl;
        g_stDrState.nRobotState = (int)eState;
        strncpy(g_stDrState.strRobotState, GetRobotStateString((int)eState), MAX_SYMBOL_SIZE); 
    }

    void DRHWInterface::OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl)
    {
        // Only work within 50msec

        cout << "[callback OnMonitoringAccessControlCB] eAccCtrl: " << eAccCtrl << endl;
        switch(eAccCtrl)
        {
        case MONITORING_ACCESS_CONTROL_REQUEST:
            Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
            //Drfl.TransitControlAuth(MANaGE_ACCESS_CONTROL_RESPONSE_YES);
            break;
        case MONITORING_ACCESS_CONTROL_GRANT:
            cout  << "access control granted" << endl;
            g_bHasControlAuthority = TRUE;
            OnMonitoringStateCB(Drfl.GetRobotState());
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
            ROS_INFO("Access control deny !!!!!!!!!!!!!!!");
            break;
        case MONITORING_ACCESS_CONTROL_LOSS:
            g_bHasControlAuthority = FALSE;
            if (g_bTpInitailizingComplted) {
                Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST);
                //Drfl.TransitControlAuth(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            }
            break;
        default:
            break;
        }
        g_stDrState.nAccessControl = (int)eAccCtrl;
    }

    void DRHWInterface::OnLogAlarm(LPLOG_ALARM pLogAlarm)
    {
        //This function is called when an error occurs.
        ros::NodeHandlePtr node=boost::make_shared<ros::NodeHandle>();
        ros::Publisher PubRobotError=node->advertise<dsr_msgs::RobotError>("error",100);
        dsr_msgs::RobotError msg;

        switch(pLogAlarm->_iLevel)
        {
        case LOG_LEVEL_SYSINFO:
            ROS_INFO("[callback OnLogAlarm]");
            ROS_INFO(" level : %d",(unsigned int)pLogAlarm->_iLevel);
            ROS_INFO(" group : %d",(unsigned int)pLogAlarm->_iGroup);
            ROS_INFO(" index : %d", pLogAlarm->_iIndex);
            ROS_INFO(" param : %s", pLogAlarm->_szParam[0] );
            ROS_INFO(" param : %s", pLogAlarm->_szParam[1] );
            ROS_INFO(" param : %s", pLogAlarm->_szParam[2] );
            break;
        case LOG_LEVEL_SYSWARN:
            ROS_WARN("[callback OnLogAlarm]");
            ROS_WARN(" level : %d",(unsigned int)pLogAlarm->_iLevel);
            ROS_WARN(" group : %d",(unsigned int)pLogAlarm->_iGroup);
            ROS_WARN(" index : %d", pLogAlarm->_iIndex);
            ROS_WARN(" param : %s", pLogAlarm->_szParam[0] );
            ROS_WARN(" param : %s", pLogAlarm->_szParam[1] );
            ROS_WARN(" param : %s", pLogAlarm->_szParam[2] );
            break;
        case LOG_LEVEL_SYSERROR:
        default:
            ROS_ERROR("[callback OnLogAlarm]");
            ROS_ERROR(" level : %d",(unsigned int)pLogAlarm->_iLevel);
            ROS_ERROR(" group : %d",(unsigned int)pLogAlarm->_iGroup);
            ROS_ERROR(" index : %d", pLogAlarm->_iIndex);
            ROS_ERROR(" param : %s", pLogAlarm->_szParam[0] );
            ROS_ERROR(" param : %s", pLogAlarm->_szParam[1] );
            ROS_ERROR(" param : %s", pLogAlarm->_szParam[2] );
            break;
        }

        g_stDrError.nLevel=(unsigned int)pLogAlarm->_iLevel;
        g_stDrError.nGroup=(unsigned int)pLogAlarm->_iGroup;
        g_stDrError.nCode=pLogAlarm->_iIndex;
        strncpy(g_stDrError.strMsg1, pLogAlarm->_szParam[0], MAX_STRING_SIZE);
        strncpy(g_stDrError.strMsg2, pLogAlarm->_szParam[1], MAX_STRING_SIZE);
        strncpy(g_stDrError.strMsg3, pLogAlarm->_szParam[2], MAX_STRING_SIZE);

        msg.level=g_stDrError.nLevel;
        msg.group=g_stDrError.nGroup;
        msg.code=g_stDrError.nCode;
        msg.msg1=g_stDrError.strMsg1;
        msg.msg2=g_stDrError.strMsg2;
        msg.msg3=g_stDrError.strMsg3;

        PubRobotError.publish(msg);
    }

    void DRHWInterface::OnTpPopupCB(LPMESSAGE_POPUP tPopup)
    {
        ROS_INFO("OnTpPopup");
    }

    void DRHWInterface::OnTpLogCB(const char* strLog)
    {
        ROS_INFO("OnTpLog");
        cout << strLog << endl;
    }
    
    void DRHWInterface::onTpProgressCB(LPMESSAGE_PROGRESS tProgress)
    {
        ROS_INFO("OnTpProgress");
    }
    
    void DRHWInterface::OnTpGetUserInputCB(LPMESSAGE_INPUT tInput)
    {
        ROS_INFO("OnTpGetUserInput");
    }


    //----- register the call-back functions end -------------------------------------
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    void MsgScriber(const dsr_msgs::RobotStop::ConstPtr& msg)
    {
        //ROS_INFO("receive msg.stop_mode = %d", msg->stop_mode);
        //ROS_INFO("receive msg.stop_mode = %d", msg->stop_mode);
        ROS_INFO("receive msg.stop_mode = %d", msg->stop_mode);
        Drfl.stop((STOP_TYPE)msg->stop_mode);
    } 

    int DRHWInterface::MsgPublisher_RobotState()
    {
        dsr_msgs::RobotState msg;
        dsr_msgs::ModbusState modbus_state;
        memcpy(&m_stDrState, &g_stDrState, sizeof(DR_STATE));
         
        msg.robot_state         = m_stDrState.nRobotState;
        msg.robot_state_str     = m_stDrState.strRobotState;
        msg.actual_mode         = m_stDrState.nActualMode;
        msg.actual_space        = m_stDrState.nActualSpace;

        for (int i = 0; i < NUM_JOINT; i++)
        {
            msg.current_posj[i]    = m_stDrState.fCurrentPosj[i];
            msg.current_velj[i]    = m_stDrState.fCurrentVelj[i];
            msg.joint_abs[i]       = m_stDrState.fJointAbs[i];
            msg.joint_err[i]       = m_stDrState.fJointErr[i];
            msg.target_posj[i]     = m_stDrState.fTargetPosj[i];
            msg.target_velj[i]     = m_stDrState.fTargetVelj[i];

            msg.current_posx[i]      = m_stDrState.fCurrentPosx[i];
            msg.current_tool_posx[i] = m_stDrState.fCurrentToolPosx[i];
            msg.current_velx[i]    = m_stDrState.fCurrentVelx[i];
            msg.task_err[i]        = m_stDrState.fTaskErr[i];
            msg.target_velx[i]     = m_stDrState.fTargetVelx[i];
            msg.target_posx[i]     = m_stDrState.fTargetPosx[i];

            msg.dynamic_tor[i]     = m_stDrState.fDynamicTor[i];
            msg.actual_jts[i]      = m_stDrState.fActualJTS[i];
            msg.actual_ejt[i]      = m_stDrState.fActualEJT[i];
            msg.actual_ett[i]      = m_stDrState.fActualETT[i];


            msg.actual_bk[i]       = m_stDrState.nActualBK[i];
            msg.actual_mc[i]       = m_stDrState.fActualMC[i];
            msg.actual_mt[i]       = m_stDrState.fActualMT[i];
        }
        msg.solution_space      = m_stDrState.nSolutionSpace;
        msg.sync_time           = m_stDrState.dSyncTime;
        std_msgs::Float64MultiArray arr;

        for (int i = 0; i < 3; i++){
            arr.data.clear();
            for (int j = 0; j < 3; j++){
                arr.data.push_back(m_stDrState.fRotationMatrix[i][j]);
            }
            msg.rotation_matrix.push_back(arr);
        }
        
        for (int i = 0; i < NUM_BUTTON; i++){
            msg.actual_bt[i] = m_stDrState.nActualBT[i];
        }
        for (int i = 0; i < NUM_DIGITAL; i++){
            msg.ctrlbox_digital_input[i]    = m_stDrState.bCtrlBoxDigitalInput[i];
            msg.ctrlbox_digital_output[i]   = m_stDrState.bCtrlBoxDigitalOutput[i];    
        }
        for (int i = 0; i < NUM_FLANGE_IO; i++){
            msg.flange_digital_input[i]     = m_stDrState.bFlangeDigitalInput[i];
            msg.flange_digital_output[i]    = m_stDrState.bFlangeDigitalOutput[i];
        }
        //msg.io_modbus;    GJH
        for (int i = 0; i < m_stDrState.nRegCount; i++){
            modbus_state.modbus_symbol   = m_stDrState.strModbusSymbol[i];
            modbus_state.modbus_value    = m_stDrState.nModbusValue[i];
            msg.modbus_state.push_back(modbus_state);
        }
        //msg.error;        GJH
        msg.access_control      = m_stDrState.nAccessControl;
        msg.homming_completed   = m_stDrState.bHommingCompleted;
        msg.tp_initialized      = m_stDrState.bTpInitialized; 
        msg.mastering_need      = m_stDrState.bMasteringNeed;
        msg.drl_stopped         = m_stDrState.bDrlStopped;
        msg.disconnected        = m_stDrState.bDisconnected;

        //--- The following messages have been updated since version M2.50 or higher ---
        if(m_nVersionDRCF >= 120500)    //M2.5 or later        
        {
            for (int i = 0; i < NUM_JOINT; i++){
                msg.fActualW2B[i]   = m_stDrState.fActualW2B[i];
                msg.fCurrentVelW[i] = m_stDrState.fCurrentVelW[i];
                msg.fWorldETT[i]    = m_stDrState.fWorldETT[i];
                msg.fTargetPosW[i]  = m_stDrState.fTargetPosW[i];
                msg.fTargetVelW[i]  = m_stDrState.fTargetVelW[i];
                msg.fCurrentVelU[i] = m_stDrState.fCurrentVelU[i];
                msg.fUserETT[i]     = m_stDrState.fUserETT[i];
                msg.fTargetPosU[i]  = m_stDrState.fTargetPosU[i];
                msg.fTargetVelU[i]  = m_stDrState.fTargetVelU[i];
            }      
            for(int i = 0; i < 2; i++){
                arr.data.clear();
                for(int j = 0; j < 6; j++){
                    arr.data.push_back(m_stDrState.fCurrentPosW[i][j]);
                }
                msg.fCurrentPosW.push_back(arr);
            }
            for(int i = 0; i < 2; i++){
                arr.data.clear();
                for(int j = 0; j < 6; j++){
                    arr.data.push_back(m_stDrState.fCurrentPosU[i][j]);
                }
                msg.fCurrentPosU.push_back(arr);
            }
            for(int i = 0; i < 3; i++){
                arr.data.clear();
                for(int j = 0; j < 3; j++){
                    arr.data.push_back(m_stDrState.fRotationMatrixWorld[i][j]);
                }
                msg.fRotationMatrixWorld.push_back(arr);
            }
            for(int i = 0; i < 3; i++){
                arr.data.clear();
                for(int j = 0; j < 3; j++){
                    arr.data.push_back(m_stDrState.fRotationMatrixUser[i][j]);
                }
                msg.fRotationMatrixUser.push_back(arr);
            }

            msg.iActualUCN = m_stDrState.iActualUCN;
            msg.iParent    = m_stDrState.iParent;

            for (int i = 0; i < 3; i++)
                msg.bActualSW[i] = m_stDrState.bActualSW[i];

            for (int i = 0; i < 2; i++){
                msg.bActualSI[i] = m_stDrState.bActualSI[i];
                msg.fActualAI[i] = m_stDrState.fActualAI[i];
                msg.iActualAT[i] = m_stDrState.iActualAT[i];
                msg.fTargetAO[i] = m_stDrState.fTargetAO[i];
                msg.iTargetAT[i] = m_stDrState.iTargetAT[i];
                msg.bActualES[i] = m_stDrState.bActualES[i];
                msg.iActualED[i] = m_stDrState.iActualED[i];
                msg.bActualER[i] = m_stDrState.bActualER[i];
            }
        }        
        //------------------------------------------------------------------------------

        m_PubRobotState.publish(msg);
        return 0; 
    }

    /* 현재 미사용 : DRHWInterface::OnLogAlarm 에서 바로 퍼블리싱 함.
    int DRHWInterface::MsgPublisher_RobotError()
    {
        dsr_msgs::RobotError msg;
        memcpy(&m_stDrError, &g_stDrError, sizeof(DR_ERROR));

        msg.level = m_stDrError.nLevel;
        msg.group = m_stDrError.nGroup;
        msg.code  = m_stDrError.nCode;
        msg.msg1  = m_stDrError.strMsg1;  
        msg.msg2  = m_stDrError.strMsg2;
        msg.msg3  = m_stDrError.strMsg3;

        m_PubRobotError.publish(msg);
        return 0; 
    }
    */

    void DRHWInterface::thread_subscribe(ros::NodeHandle nh)
    {
        //ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
        ros::Subscriber sub_robot_stop = nh.subscribe("stop", 100, MsgScriber);
        //ros::spin();
        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    }

    void DRHWInterface::thread_publisher(DRHWInterface* pDRHWInterface, ros::NodeHandle nh, int nPubRate)
    {  
        //ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
        ros::Publisher PubRobotState = nh.advertise<dsr_msgs::RobotState>("state",100);
        dsr_msgs::RobotState msg;

        ros::Rate r(nPubRate);
        while (ros::ok())
        {
            //ROS_INFO("thread_publisher running!");      
            if(pDRHWInterface) pDRHWInterface->MsgPublisher_RobotState();
            r.sleep();
        }
    }  

    DRHWInterface::DRHWInterface(ros::NodeHandle& nh):private_nh_(nh)
    {
        /*
        <arg name="ns"    value="$(arg ns)"/>
        <arg name="model" value="$(arg model)"/>
        <arg name="host"  value="$(arg host)"/>
        <arg name="mode"  value="$(arg mode)"/>
        */

        private_nh_.getParam("name", m_strRobotName);
        private_nh_.getParam("model", m_strRobotModel);
        private_nh_.getParam("gripper", m_strRobotGripper);

        ROS_INFO("name_space is %s, %s\n",m_strRobotName.c_str(), m_strRobotModel.c_str());

        ROS_INFO("[dsr_hw_interface] constructed");
        ros::V_string arm_joint_names;
        if(m_strRobotGripper == "robotiq_2f"){
            arm_joint_names =
            boost::assign::list_of("joint1")("joint2")("joint3")("joint4")("joint5")("joint6")("robotiq_85_left_knuckle_joint").convert_to_container<ros::V_string>();
        }
        else if(m_strRobotGripper == "none")
        {
            arm_joint_names =
            boost::assign::list_of("joint1")("joint2")("joint3")("joint4")("joint5")("joint6").convert_to_container<ros::V_string>();
        }
        for(unsigned int i = 0; i < arm_joint_names.size(); i++){
            hardware_interface::JointStateHandle jnt_state_handle(
                arm_joint_names[i],
                &joints[i].pos,
                &joints[i].vel,
                &joints[i].eff);
            jnt_state_interface.registerHandle(jnt_state_handle);

            hardware_interface::JointHandle jnt_pos_handle(
                jnt_state_handle,
                &joints[i].cmd);
            jnt_pos_interface.registerHandle(jnt_pos_handle);
        }
        registerInterface(&jnt_state_interface);
        registerInterface(&jnt_pos_interface);

        /*ros::V_string joint_names = boost::assign::list_of("front_left_wheel")("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
        for (unsigned int i = 0; i < joint_names.size(); i++){
            hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joints[i].pos, &joints[i].vel, &joints[i].eff);
            jnt_state_interface.registerHandle(joint_state_handle);

            hardware_interface::JointHandle joint_handle(joint_state_handle, &joints[i].cmd);
            velocity_joint_interface_.registerHandle(joint_handle);
        }
        registerInterface(&velocity_joint_interface_);
        */

        // Publisher msg 
        m_PubRobotState = private_nh_.advertise<dsr_msgs::RobotState>("state",100);
        m_PubRobotError = private_nh_.advertise<dsr_msgs::RobotError>("error",100);
        ///m_PubJogMultiAxis = private_nh_.advertise<dsr_msgs::JogMultiAxis>("jog_multi",100);

        // gazebo에 joint position 전달
        m_PubtoGazebo = private_nh_.advertise<std_msgs::Float64MultiArray>("/dsr_joint_position_controller/command",10);
        // moveit의 trajectory/goal를 받아 제어기로 전달
        m_sub_joint_trajectory = private_nh_.subscribe("dsr_joint_trajectory_controller/follow_joint_trajectory/goal", 10, &DRHWInterface::trajectoryCallback, this);
        // topic echo 명령으로 제어기에 전달
        m_sub_joint_position = private_nh_.subscribe("dsr_joint_position_controller/command", 10, &DRHWInterface::positionCallback, this);
        
        ros::NodeHandle nh_temp;
        m_SubSerialRead = nh_temp.subscribe("serial_read", 100, &Serial_comm::read_callback, &ser_comm);
        m_PubSerialWrite = nh_temp.advertise<std_msgs::String>("serial_write", 100);

        // subscribe : Multi-JOG topic msg
        m_sub_jog_multi_axis = private_nh_.subscribe("jog_multi", 10, &DRHWInterface::jogCallback, this);
        m_sub_alter_motion_stream = private_nh_.subscribe("alter_motion_stream", 20, &DRHWInterface::alterCallback, this);
        m_sub_servoj_stream = private_nh_.subscribe("servoj_stream", 20, &DRHWInterface::servojCallback, this);
        m_sub_servol_stream = private_nh_.subscribe("servol_stream", 20, &DRHWInterface::servolCallback, this);
        m_sub_speedj_stream = private_nh_.subscribe("speedj_stream", 20, &DRHWInterface::speedjCallback, this);
        m_sub_speedl_stream = private_nh_.subscribe("speedl_stream", 20, &DRHWInterface::speedlCallback, this);

        m_sub_servoj_rt_stream = private_nh_.subscribe("servoj_rt_stream", 20, &DRHWInterface::servojRTCallback, this);
        m_sub_servol_rt_stream = private_nh_.subscribe("servol_rt_stream", 20, &DRHWInterface::servolRTCallback, this);
        m_sub_speedj_rt_stream = private_nh_.subscribe("speedj_rt_stream", 20, &DRHWInterface::speedjRTCallback, this);
        m_sub_speedl_rt_stream = private_nh_.subscribe("speedl_rt_stream", 20, &DRHWInterface::speedlRTCallback, this);
        m_sub_torque_rt_stream = private_nh_.subscribe("torque_rt_stream", 20, &DRHWInterface::torqueRTCallback, this);

        // system Operations
        m_nh_system[0] = private_nh_.advertiseService("system/set_robot_mode", &DRHWInterface::set_robot_mode_cb, this);
        m_nh_system[1] = private_nh_.advertiseService("system/get_robot_mode", &DRHWInterface::get_robot_mode_cb, this);
        m_nh_system[2] = private_nh_.advertiseService("system/set_robot_system", &DRHWInterface::set_robot_system_cb, this);
        m_nh_system[3] = private_nh_.advertiseService("system/get_robot_system", &DRHWInterface::get_robot_system_cb, this);
        m_nh_system[4] = private_nh_.advertiseService("system/set_robot_speed_mode", &DRHWInterface::set_robot_speed_mode_cb, this);
        m_nh_system[5] = private_nh_.advertiseService("system/get_robot_speed_mode", &DRHWInterface::get_robot_speed_mode_cb, this);
        m_nh_system[6] = private_nh_.advertiseService("system/get_current_pose", &DRHWInterface::get_current_pose_cb, this);
        m_nh_system[7] = private_nh_.advertiseService("system/get_current_solution_space", &DRHWInterface::get_current_solution_space_cb, this);
        m_nh_system[8] = private_nh_.advertiseService("system/set_safe_stop_reset_type", &DRHWInterface::set_safe_stop_reset_type_cb, this);
        m_nh_system[9] = private_nh_.advertiseService("system/get_last_alarm", &DRHWInterface::get_last_alarm_cb, this);
        m_nh_system[10]= private_nh_.advertiseService("system/get_robot_state", &DRHWInterface::get_robot_state_cb, this);
        m_nh_system[11]= private_nh_.advertiseService("system/get_joint_torque", &DRHWInterface::get_joint_torque_cb, this);
        m_nh_system[12]= private_nh_.advertiseService("system/set_robot_control", &DRHWInterface::set_robot_control_cb, this);
        m_nh_system[13]= private_nh_.advertiseService("system/manage_access_control", &DRHWInterface::manage_access_control_cb, this);
        m_nh_system[14]= private_nh_.advertiseService("system/release_protective_stop", &DRHWInterface::release_protective_stop_cb, this);

        //  motion Operations
        m_nh_motion_service[0] = private_nh_.advertiseService("motion/move_joint", &DRHWInterface::movej_cb, this);
        m_nh_motion_service[1] = private_nh_.advertiseService("motion/move_line", &DRHWInterface::movel_cb, this);
        m_nh_motion_service[2] = private_nh_.advertiseService("motion/move_jointx", &DRHWInterface::movejx_cb, this);
        m_nh_motion_service[3] = private_nh_.advertiseService("motion/move_circle", &DRHWInterface::movec_cb, this);
        m_nh_motion_service[4] = private_nh_.advertiseService("motion/move_spline_joint", &DRHWInterface::movesj_cb, this);
        m_nh_motion_service[5] = private_nh_.advertiseService("motion/move_spline_task", &DRHWInterface::movesx_cb, this);
        m_nh_motion_service[6] = private_nh_.advertiseService("motion/move_blending", &DRHWInterface::moveb_cb, this);
        m_nh_motion_service[7] = private_nh_.advertiseService("motion/move_spiral", &DRHWInterface::movespiral_cb, this);
        m_nh_motion_service[8] = private_nh_.advertiseService("motion/move_periodic", &DRHWInterface::moveperiodic_cb, this);
        m_nh_motion_service[9] = private_nh_.advertiseService("motion/move_wait", &DRHWInterface::movewait_cb, this);
        m_nh_motion_service[10]= private_nh_.advertiseService("motion/jog", &DRHWInterface::jog_cb, this);
        m_nh_motion_service[11]= private_nh_.advertiseService("motion/jog_multi", &DRHWInterface::jog_multi_cb, this);
        m_nh_motion_service[12]= private_nh_.advertiseService("motion/move_stop", &DRHWInterface::move_stop_cb, this);
        m_nh_motion_service[13]= private_nh_.advertiseService("motion/move_pause", &DRHWInterface::move_pause_cb, this);
        m_nh_motion_service[14]= private_nh_.advertiseService("motion/move_resume", &DRHWInterface::move_resume_cb, this);
        m_nh_motion_service[15]= private_nh_.advertiseService("motion/trans", &DRHWInterface::trans_cb, this);     
        m_nh_motion_service[16]= private_nh_.advertiseService("motion/fkin", &DRHWInterface::fkin_cb, this);
        m_nh_motion_service[17]= private_nh_.advertiseService("motion/ikin", &DRHWInterface::ikin_cb, this);
        m_nh_motion_service[18]= private_nh_.advertiseService("motion/set_ref_coord", &DRHWInterface::set_ref_coord_cb, this);
        m_nh_motion_service[19]= private_nh_.advertiseService("motion/move_home", &DRHWInterface::move_home_cb, this);
        m_nh_motion_service[20]= private_nh_.advertiseService("motion/check_motion", &DRHWInterface::check_motion_cb, this);
        m_nh_motion_service[21]= private_nh_.advertiseService("motion/change_operation_speed", &DRHWInterface::change_operation_speed_cb, this);
        m_nh_motion_service[22]= private_nh_.advertiseService("motion/enable_alter_motion", &DRHWInterface::enable_alter_motion_cb, this);
        m_nh_motion_service[23]= private_nh_.advertiseService("motion/alter_motion", &DRHWInterface::alter_motion_cb, this);
        m_nh_motion_service[24]= private_nh_.advertiseService("motion/disable_alter_motion", &DRHWInterface::disable_alter_motion_cb, this);
        m_nh_motion_service[25]= private_nh_.advertiseService("motion/set_singularity_handling", &DRHWInterface::set_singularity_handling_cb, this);
        m_nh_motion_service[26]= private_nh_.advertiseService("motion/ikin_ex", &DRHWInterface::ikin_ex_cb, this);

        // Auxiliary Control Operations
        m_nh_aux_control_service[0]  = private_nh_.advertiseService("aux_control/get_control_mode", &DRHWInterface::get_control_mode_cb, this);                   
        m_nh_aux_control_service[1]  = private_nh_.advertiseService("aux_control/get_control_space", &DRHWInterface::get_control_space_cb, this);                         

        m_nh_aux_control_service[2]  = private_nh_.advertiseService("aux_control/get_current_posj", &DRHWInterface::get_current_posj_cb, this);                              
        m_nh_aux_control_service[3]  = private_nh_.advertiseService("aux_control/get_current_velj", &DRHWInterface::get_current_velj_cb, this);                               
        m_nh_aux_control_service[4]  = private_nh_.advertiseService("aux_control/get_desired_posj", &DRHWInterface::get_desired_posj_cb, this);                                         
        m_nh_aux_control_service[5]  = private_nh_.advertiseService("aux_control/get_desired_velj", &DRHWInterface::get_desired_velj_cb, this);                                   

        m_nh_aux_control_service[6]  = private_nh_.advertiseService("aux_control/get_current_posx", &DRHWInterface::get_current_posx_cb, this);                              
        m_nh_aux_control_service[7]  = private_nh_.advertiseService("aux_control/get_current_velx", &DRHWInterface::get_current_velx_cb, this);                                                         
        m_nh_aux_control_service[8]  = private_nh_.advertiseService("aux_control/get_desired_posx", &DRHWInterface::get_desired_posx_cb, this);                                           
        m_nh_aux_control_service[9]  = private_nh_.advertiseService("aux_control/get_desired_velx", &DRHWInterface::get_desired_velx_cb, this);                                                    

        m_nh_aux_control_service[10]  = private_nh_.advertiseService("aux_control/get_current_tool_flange_posx", &DRHWInterface::get_current_tool_flange_posx_cb, this);                                            
        m_nh_aux_control_service[11] = private_nh_.advertiseService("aux_control/get_current_solution_space", &DRHWInterface::get_current_solution_space_cb, this);                                                 
        m_nh_aux_control_service[12] = private_nh_.advertiseService("aux_control/get_current_rotm", &DRHWInterface::get_current_rotm_cb, this);                                                              
        m_nh_aux_control_service[13] = private_nh_.advertiseService("aux_control/get_joint_torque", &DRHWInterface::get_joint_torque_cb, this);                                                              
        m_nh_aux_control_service[14] = private_nh_.advertiseService("aux_control/get_external_torque", &DRHWInterface::get_external_torque_cb, this);                                                          
        m_nh_aux_control_service[15] = private_nh_.advertiseService("aux_control/get_tool_force", &DRHWInterface::get_tool_force_cb, this);                                                          
        m_nh_aux_control_service[16] = private_nh_.advertiseService("aux_control/get_solution_space", &DRHWInterface::get_solution_space_cb, this);                                                      
        m_nh_aux_control_service[17] = private_nh_.advertiseService("aux_control/get_orientation_error", &DRHWInterface::get_orientation_error_cb, this);                                                     


        //force & stiffness
        m_nh_force_service[0] = private_nh_.advertiseService("force/parallel_axis1", &DRHWInterface::parallel_axis1_cb, this);
        m_nh_force_service[1] = private_nh_.advertiseService("force/parallel_axis2", &DRHWInterface::parallel_axis2_cb, this);
        m_nh_force_service[2] = private_nh_.advertiseService("force/align_axis1", &DRHWInterface::align_axis1_cb, this);
        m_nh_force_service[3] = private_nh_.advertiseService("force/align_axis2", &DRHWInterface::align_axis2_cb, this);
        m_nh_force_service[4] = private_nh_.advertiseService("force/is_done_bolt_tightening", &DRHWInterface::is_done_bolt_tightening_cb, this);
        m_nh_force_service[5] = private_nh_.advertiseService("force/release_compliance_ctrl", &DRHWInterface::release_compliance_ctrl_cb, this);
        m_nh_force_service[6] = private_nh_.advertiseService("force/task_compliance_ctrl", &DRHWInterface::task_compliance_ctrl_cb, this);
        m_nh_force_service[7] = private_nh_.advertiseService("force/set_stiffnessx", &DRHWInterface::set_stiffnessx_cb, this);
        m_nh_force_service[8] = private_nh_.advertiseService("force/calc_coord", &DRHWInterface::calc_coord_cb, this);
        m_nh_force_service[9] = private_nh_.advertiseService("force/set_user_cart_coord1", &DRHWInterface::set_user_cart_coord1_cb, this);
        m_nh_force_service[10]= private_nh_.advertiseService("force/set_user_cart_coord2", &DRHWInterface::set_user_cart_coord2_cb, this);
        m_nh_force_service[11]= private_nh_.advertiseService("force/set_user_cart_coord3", &DRHWInterface::set_user_cart_coord3_cb, this);
        m_nh_force_service[12]= private_nh_.advertiseService("force/overwrite_user_cart_coord", &DRHWInterface::overwrite_user_cart_coord_cb, this);
        m_nh_force_service[13]= private_nh_.advertiseService("force/get_user_cart_coord", &DRHWInterface::get_user_cart_coord_cb, this);
        m_nh_force_service[14]= private_nh_.advertiseService("force/set_desired_force", &DRHWInterface::set_desired_force_cb, this);
        m_nh_force_service[15]= private_nh_.advertiseService("force/release_force", &DRHWInterface::release_force_cb, this);
        m_nh_force_service[16]= private_nh_.advertiseService("force/check_position_condition", &DRHWInterface::check_position_condition_cb, this);
        m_nh_force_service[17]= private_nh_.advertiseService("force/check_force_condition", &DRHWInterface::check_force_condition_cb, this);
        m_nh_force_service[18]= private_nh_.advertiseService("force/check_orientation_condition1", &DRHWInterface::check_orientation_condition1_cb, this);
        m_nh_force_service[19]= private_nh_.advertiseService("force/check_orientation_condition2", &DRHWInterface::check_orientation_condition2_cb, this);
        m_nh_force_service[20]= private_nh_.advertiseService("force/coord_transform", &DRHWInterface::coord_transform_cb, this);
        m_nh_force_service[21]= private_nh_.advertiseService("force/get_workpiece_weight", &DRHWInterface::get_workpiece_weight_cb, this);
        m_nh_force_service[22]= private_nh_.advertiseService("force/reset_workpiece_weight", &DRHWInterface::reset_workpiece_weight_cb, this);

        //  GPIO Operations
        m_nh_io_service[0] = private_nh_.advertiseService("io/set_digital_output", &DRHWInterface::set_digital_output_cb, this);
        m_nh_io_service[1] = private_nh_.advertiseService("io/get_digital_input", &DRHWInterface::get_digital_input_cb, this);
        m_nh_io_service[2] = private_nh_.advertiseService("io/set_tool_digital_output", &DRHWInterface::set_tool_digital_output_cb, this);
        m_nh_io_service[3] = private_nh_.advertiseService("io/get_tool_digital_input", &DRHWInterface::get_tool_digital_input_cb, this);
        m_nh_io_service[4] = private_nh_.advertiseService("io/set_analog_output", &DRHWInterface::set_analog_output_cb, this);
        m_nh_io_service[5] = private_nh_.advertiseService("io/get_analog_input", &DRHWInterface::get_analog_input_cb, this);
        m_nh_io_service[6] = private_nh_.advertiseService("io/set_analog_output_type", &DRHWInterface::set_analog_output_type_cb, this);
        m_nh_io_service[7] = private_nh_.advertiseService("io/set_analog_input_type", &DRHWInterface::set_analog_input_type_cb, this);
        m_nh_io_service[8] = private_nh_.advertiseService("io/get_digital_output", &DRHWInterface::get_digital_output_cb, this);
        m_nh_io_service[9] = private_nh_.advertiseService("io/get_tool_digital_output", &DRHWInterface::get_tool_digital_output_cb, this);

        //  Modbus Operations
        m_nh_modbus_service[0] = private_nh_.advertiseService("modbus/set_modbus_output", &DRHWInterface::set_modbus_output_cb, this);
        m_nh_modbus_service[1] = private_nh_.advertiseService("modbus/get_modbus_input", &DRHWInterface::get_modbus_input_cb, this);
        m_nh_modbus_service[2] = private_nh_.advertiseService("modbus/config_create_modbus", &DRHWInterface::config_create_modbus_cb, this);
        m_nh_modbus_service[3] = private_nh_.advertiseService("modbus/config_delete_modbus", &DRHWInterface::config_delete_modbus_cb, this);

        // TCP Operations
        m_nh_tcp_service[0] = private_nh_.advertiseService("tcp/set_current_tcp", &DRHWInterface::set_current_tcp_cb, this);
        m_nh_tcp_service[1] = private_nh_.advertiseService("tcp/get_current_tcp", &DRHWInterface::get_current_tcp_cb, this);
        m_nh_tcp_service[2] = private_nh_.advertiseService("tcp/config_create_tcp", &DRHWInterface::config_create_tcp_cb, this);
        m_nh_tcp_service[3] = private_nh_.advertiseService("tcp/config_delete_tcp", &DRHWInterface::config_delete_tcp_cb, this);

        // Tool Operations
        m_nh_tool_service[0] = private_nh_.advertiseService("tool/set_current_tool", &DRHWInterface::set_current_tool_cb, this);
        m_nh_tool_service[1] = private_nh_.advertiseService("tool/get_current_tool", &DRHWInterface::get_current_tool_cb, this);
        m_nh_tool_service[2] = private_nh_.advertiseService("tool/config_create_tool", &DRHWInterface::config_create_tool_cb, this);
        m_nh_tool_service[3] = private_nh_.advertiseService("tool/config_delete_tool", &DRHWInterface::config_delete_tool_cb, this);
        m_nh_tool_service[4] = private_nh_.advertiseService("tool/set_tool_shape", &DRHWInterface::set_tool_shape_cb, this);

        // DRL Operations
        m_nh_drl_service[0] = private_nh_.advertiseService("drl/drl_pause", &DRHWInterface::drl_pause_cb, this);
        m_nh_drl_service[1] = private_nh_.advertiseService("drl/drl_resume", &DRHWInterface::drl_resume_cb, this);
        m_nh_drl_service[2] = private_nh_.advertiseService("drl/drl_start", &DRHWInterface::drl_start_cb, this);
        m_nh_drl_service[3] = private_nh_.advertiseService("drl/drl_stop", &DRHWInterface::drl_stop_cb, this);
        m_nh_drl_service[4] = private_nh_.advertiseService("drl/get_drl_state", &DRHWInterface::get_drl_state_cb, this);
        
        // Gripper Operations
        m_nh_gripper_service[0] = private_nh_.advertiseService("gripper/robotiq_2f_open", &DRHWInterface::robotiq_2f_open_cb, this);
        m_nh_gripper_service[1] = private_nh_.advertiseService("gripper/robotiq_2f_close", &DRHWInterface::robotiq_2f_close_cb, this);
        m_nh_gripper_service[2] = private_nh_.advertiseService("gripper/robotiq_2f_move", &DRHWInterface::robotiq_2f_move_cb, this);

        // Serial Operations  
        m_nh_serial_service[0] = private_nh_.advertiseService("gripper/serial_send_data", &DRHWInterface::serial_send_data_cb, this);

        // Realtime Operations
        m_nh_realtime_service[0] = private_nh_.advertiseService("realtime/connect_rt_control", &DRHWInterface::connect_rt_control_cb, this);
        m_nh_realtime_service[1] = private_nh_.advertiseService("realtime/disconnect_rt_control", &DRHWInterface::disconnect_rt_control_cb, this);
        m_nh_realtime_service[2] = private_nh_.advertiseService("realtime/get_rt_control_output_version_list", &DRHWInterface::get_rt_control_output_version_list_cb, this);
        m_nh_realtime_service[3] = private_nh_.advertiseService("realtime/get_rt_control_input_version_list", &DRHWInterface::get_rt_control_input_version_list_cb, this);
        m_nh_realtime_service[4] = private_nh_.advertiseService("realtime/get_rt_control_input_data_list", &DRHWInterface::get_rt_control_input_data_list_cb, this);
        m_nh_realtime_service[5] = private_nh_.advertiseService("realtime/get_rt_control_output_data_list", &DRHWInterface::get_rt_control_output_data_list_cb, this);
        m_nh_realtime_service[6] = private_nh_.advertiseService("realtime/set_rt_control_input", &DRHWInterface::set_rt_control_input_cb, this);
        m_nh_realtime_service[7] = private_nh_.advertiseService("realtime/set_rt_control_output", &DRHWInterface::set_rt_control_output_cb, this);
        m_nh_realtime_service[8] = private_nh_.advertiseService("realtime/start_rt_control", &DRHWInterface::start_rt_control_cb, this);
        m_nh_realtime_service[9] = private_nh_.advertiseService("realtime/stop_rt_control", &DRHWInterface::stop_rt_control_cb, this);
        m_nh_realtime_service[10] = private_nh_.advertiseService("realtime/set_velj_rt", &DRHWInterface::set_velj_rt_cb, this);
        m_nh_realtime_service[11] = private_nh_.advertiseService("realtime/set_accj_rt", &DRHWInterface::set_accj_rt_cb, this);
        m_nh_realtime_service[12] = private_nh_.advertiseService("realtime/set_velx_rt", &DRHWInterface::set_velx_rt_cb, this);
        m_nh_realtime_service[13] = private_nh_.advertiseService("realtime/set_accx_rt", &DRHWInterface::set_accx_rt_cb, this);
        m_nh_realtime_service[14] = private_nh_.advertiseService("realtime/read_data_rt", &DRHWInterface::read_data_rt_cb, this);
        m_nh_realtime_service[14] = private_nh_.advertiseService("realtime/write_data_rt", &DRHWInterface::write_data_rt_cb, this);

        memset(&g_stDrState, 0x00, sizeof(DR_STATE)); 
        memset(&g_stDrError, 0x00, sizeof(DR_ERROR)); 
        memset(&m_stDrState, 0x00, sizeof(DR_STATE));
        memset(&m_stDrError, 0x00, sizeof(DR_ERROR));

        // create threads     
        m_th_subscribe = boost::thread( boost::bind(&thread_subscribe, private_nh_) );
        m_th_publisher = boost::thread( boost::bind(&thread_publisher, this, private_nh_, DSR_CTL_PUB_RATE/*hz*/) );    //100hz(10ms)

        g_nAnalogOutputModeCh1 = -1;
        g_nAnalogOutputModeCh2 = -1;

    }
    DRHWInterface::~DRHWInterface()
    {
        //ROS_INFO("DRHWInterface::~DRHWInterface() 0");
        Drfl.close_connection();

        //ROS_INFO("DRHWInterface::~DRHWInterface() 1");
        m_th_publisher.join();   //kill publisher thread
        //ROS_INFO("DRHWInterface::~DRHWInterface() 2");

        m_th_subscribe.join();   //kill subscribe thread 
        ROS_INFO("DRHWInterface::~DRHWInterface()");
    }

    bool DRHWInterface::init()
    {
        ROS_INFO("[dsr_hw_interface] init() ==> setup callback fucntion");
        int nServerPort = 12345;
        ROS_INFO("INIT@@@@@@@@@@@@@@@@@@@@@@@@@");
        //--- doosan API's call-back fuctions : Only work within 50msec in call-back functions
        Drfl.set_on_tp_initializing_completed(OnTpInitializingCompletedCB);
        Drfl.set_on_homming_completed(OnHommingCompletedCB);
        Drfl.set_on_program_stopped(OnProgramStoppedCB);
        Drfl.set_on_monitoring_modbus(OnMonitoringModbusCB);
        Drfl.set_on_monitoring_data(OnMonitoringDataCB);           // Callback function in M2.4 and earlier
        Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);       // Callback function in M2.4 and earlier
        Drfl.set_on_monitoring_state(OnMonitoringStateCB);
        Drfl.set_on_monitoring_access_control(OnMonitoringAccessControlCB);
        Drfl.set_on_log_alarm(OnLogAlarm);
        Drfl.set_on_tp_popup(OnTpPopupCB);
        Drfl.set_on_tp_log(OnTpLogCB);
        Drfl.set_on_tp_get_user_input(OnTpGetUserInputCB);
        //Drfl.set_on_tp_progress(OnTpProgressCB);

        ROS_INFO("[dsr_hw_interface] init() ==> arm is standby");
        std::string host;
        std::string mode;
        private_nh_.getParam("host", host);
        private_nh_.getParam("port", nServerPort);

        private_nh_.param<bool>("command", bCommand_, false);
        private_nh_.getParam("mode", mode);

        //for test host = "127.0.0.1";

        ROS_INFO("host %s, port=%d bCommand: %d, mode: %s\n", host.c_str(), nServerPort, bCommand_, mode.c_str());


        if(Drfl.open_connection(host, nServerPort))
        {
            //--- connect Emulator ? ------------------------------    
            if(host == "127.0.0.1") m_bIsEmulatorMode = true; 
            else                    m_bIsEmulatorMode = false;

            //--- Get version -------------------------------------            
            SYSTEM_VERSION tSysVerion = {'\0', };
            assert(Drfl.get_system_version(&tSysVerion));

            //--- Get DRCF version & convert to integer  ----------            
            m_nVersionDRCF = 0; 
            int k=0;
            for(int i=strlen(tSysVerion._szController); i>0; i--)
                if(tSysVerion._szController[i]>='0' && tSysVerion._szController[i]<='9')
                    m_nVersionDRCF += (tSysVerion._szController[i]-'0')*pow(10.0,k++);
            if(m_nVersionDRCF < 100000) m_nVersionDRCF += 100000; 

            ROS_INFO("_______________________________________________");   
            if(m_bIsEmulatorMode) ROS_INFO("    Emulator Mode");
            else                  ROS_INFO("    Real Robot Mode");
            ROS_INFO("    DRCF version = %s",tSysVerion._szController);
            ROS_INFO("    DRFL version = %s",Drfl.get_library_version());
            ROS_INFO("    m_nVersionDRCF = %d", m_nVersionDRCF);  //ex> M2.40 = 120400, M2.50 = 120500  
            ROS_INFO("_______________________________________________");   

            if(m_nVersionDRCF >= 120500)    //M2.5 or later        
            {
                Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);      //Callback function in version 2.5 and higher
                Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);  //Callback function in version 2.5 and higher
                Drfl.setup_monitoring_version(1);                        //Enabling extended monitoring functions 
            }

            //--- Check Robot State : STATE_STANDBY ---               
            int delay;
            ros::param::param<int>("~standby", delay, 5000);
            while ((Drfl.get_robot_state() != STATE_STANDBY)){
                usleep(delay);
            }

            //--- Set Robot mode : MANUAL or AUTO
            //assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
            assert(Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));

            //--- Set Robot mode : virual or real 
            ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
            if(mode == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
            assert(Drfl.set_robot_system(eTargetSystem));

            // to compare with joints[].cmd
            for(int i = 0; i < NUM_JOINT; i++){
                ROS_INFO("[init]::read %d-pos: %7.3f", i, joints[i].cmd);
                cmd_[i] = joints[i].cmd;
            }
            return true;
         }
        return false;
    }

    void DRHWInterface::read(ros::Duration& elapsed_time)
    {
        std_msgs::Float64MultiArray msg;
        // joints.pos, vel, eff should be update
        //ROS_DEBUG("DRHWInterface::read()");
        LPROBOT_POSE pose = Drfl.GetCurrentPose();
        for(int i = 0; i < NUM_JOINT; i++){
            ROS_DEBUG("[DRHWInterface::read] %d-pos: %7.3f", i, pose->_fPosition[i]);
            joints[i].pos = deg2rad(pose->_fPosition[i]);	//update pos to Rviz
            msg.data.push_back(joints[i].pos);
        }
        if(m_strRobotGripper != "none"){
            msg.data.push_back(joints[6].pos);
        }
        m_PubtoGazebo.publish(msg);
    }
    void DRHWInterface::write(ros::Duration& elapsed_time)
    {
        //ROS_INFO("DRHWInterface::write()");
        static int count = 0;
        // joints.cmd is updated
        std::array<float, NUM_JOINT> tmp;
        for(int i = 0; i < NUM_JOINT; i++){
            ROS_DEBUG("[write]::write %d-pos: %7.3f %d-vel: %7.3f %d-cmd: %7.3f",
            i,
            joints[i].pos,
            i,
            joints[i].vel,
            i,
            joints[i].cmd);
            tmp[i] = joints[i].cmd;
        }
        if( !bCommand_ ) return;
        /*int state = Drfl.GetRobotState();
        if( state == STATE_STANDBY ){
            for(int i = 0; i < NUM_JOINT; i++){
                if( fabs(cmd_[i] - joints[i].cmd) > 0.0174532925 ){
                    Drfl.MoveJAsync(tmp.data(), 50, 50);
                    ROS_INFO_STREAM("[write] current state: " << GetRobotStateString(state));
                    std::copy(tmp.cbegin(), tmp.cend(), cmd_.begin());
                    break;
                }
            }
        }*/
    }

    //----- SIG Handler --------------------------------------------------------------
    void DRHWInterface::sigint_handler(int signo)
    {
        ROS_INFO("SIG HANDLER !!!!!!!!!");

        ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
        ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/"+m_strRobotName +m_strRobotModel+"/stop",100);
        
        dsr_msgs::RobotStop msg;
        
        msg.stop_mode  = STOP_TYPE_QUICK;
        pubRobotStop.publish(msg);

        ROS_INFO("[sigint_hangler] CloseConnection");
    }
    void DRHWInterface::positionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        ROS_INFO("callback: Position received");
        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->data.cbegin(), msg->data.cend(), target_pos.begin());
        Drfl.amovej(target_pos.data(), 50, 50);
    }

    void DRHWInterface::jogCallback(const dsr_msgs::JogMultiAxis::ConstPtr& msg){
        //ROS_INFO("callback: jogCallback received");

        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->jog_axis.cbegin(), msg->jog_axis.cend(), target_pos.begin());       
        msg->move_reference;
        msg->speed;

        //ROS_INFO("jog_axis = %f,%f,%f,%f,%f,%f", target_pos[0],target_pos[1],target_pos[2],target_pos[3],target_pos[4],target_pos[5]);
        //ROS_INFO("move_reference = %d", msg->move_reference);
        //ROS_INFO("speed = %f", msg->speed);

        Drfl.multi_jog(target_pos.data(), (MOVE_REFERENCE)msg->move_reference, msg->speed);

    }

    void DRHWInterface::alterCallback(const dsr_msgs::AlterMotionStream::ConstPtr& msg){
        
        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());

        Drfl.alter_motion(target_pos.data());
    }

    void DRHWInterface::servojCallback(const dsr_msgs::ServoJStream::ConstPtr& msg){
        
        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
        std::array<float, NUM_JOINT> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, NUM_JOINT> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.servoj(target_pos.data(), target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::servolCallback(const dsr_msgs::ServoLStream::ConstPtr& msg){
        
        std::array<float, NUM_TASK> target_pos;
        std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
        std::array<float, 2> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, 2> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.servol(target_pos.data(), target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::speedjCallback(const dsr_msgs::SpeedJStream::ConstPtr& msg){
        
        std::array<float, NUM_JOINT> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, NUM_JOINT> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.speedj(target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::speedlCallback(const dsr_msgs::SpeedLStream::ConstPtr& msg){
        
        std::array<float, NUM_JOINT> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, 2> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.speedl(target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::servojRTCallback(const dsr_msgs::ServoJRTStream::ConstPtr& msg){
        
        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
        std::array<float, NUM_JOINT> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, NUM_JOINT> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.servoj_rt(target_pos.data(), target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::servolRTCallback(const dsr_msgs::ServoLRTStream::ConstPtr& msg){
        
        std::array<float, NUM_TASK> target_pos;
        std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
        std::array<float, NUM_TASK> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, NUM_TASK> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.servol_rt(target_pos.data(), target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::speedjRTCallback(const dsr_msgs::SpeedJRTStream::ConstPtr& msg){
        
        std::array<float, NUM_JOINT> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, NUM_JOINT> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.speedj_rt(target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::speedlRTCallback(const dsr_msgs::SpeedLRTStream::ConstPtr& msg){
        
        std::array<float, NUM_TASK> target_vel;
        std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
        std::array<float, NUM_TASK> target_acc;
        std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
        int time = msg->time;

        Drfl.speedl_rt(target_vel.data(), target_acc.data(), time);
    }

    void DRHWInterface::torqueRTCallback(const dsr_msgs::TorqueRTStream::ConstPtr& msg){
        
        std::array<float, NUM_TASK> tor;
        std::copy(msg->tor.cbegin(), msg->tor.cend(), tor.begin());
        int time = msg->time;

        Drfl.torque_rt(tor.data(), time);
    }

    void DRHWInterface::trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
    {
        ROS_INFO("callback: Trajectory received");
        ROS_INFO("  msg->goal.trajectory.points.size() =%d",(int)msg->goal.trajectory.points.size());   //=10 가변젹 
        ROS_INFO("  msg->goal.trajectory.joint_names.size() =%d",(int)msg->goal.trajectory.joint_names.size()); //=6

        float preTargetTime = 0.0;
        float targetTime = 0.0;

        float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT] = {0.0, };
        int nCntTargetPos =0; 

        nCntTargetPos = msg->goal.trajectory.points.size();
        if(nCntTargetPos > MAX_SPLINE_POINT)
        {
            ROS_INFO("DRHWInterface::trajectoryCallback over max Trajectory (%d > %d)",nCntTargetPos ,MAX_SPLINE_POINT);
            return; 
        }

        for(int i = 0; i < msg->goal.trajectory.points.size(); i++) //=10
        {
            std::array<float, NUM_JOINT> degrees;
            ros::Duration d(msg->goal.trajectory.points[i].time_from_start);    

            //ROS_INFO("  msg->goal.trajectory.points[%d].time_from_start = %7.3%f",i,(float)msg->goal.trajectory.points[i].time_from_start );  

            targetTime = d.toSec();
            ///ROS_INFO("[trajectory] preTargetTime: %7.3f", preTargetTime);
            ///targetTime = targetTime - preTargetTime;
            ///preTargetTime = targetTime;
            ///ROS_INFO("[trajectory] time_from_start: %7.3f", targetTime);

            ROS_INFO("[trajectory] [%02d : %.3f] %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",i ,targetTime
                ,rad2deg(msg->goal.trajectory.points[i].positions[0]) ,rad2deg(msg->goal.trajectory.points[i].positions[1]), rad2deg(msg->goal.trajectory.points[i].positions[2])
                ,rad2deg(msg->goal.trajectory.points[i].positions[3]) ,rad2deg(msg->goal.trajectory.points[i].positions[4]), rad2deg(msg->goal.trajectory.points[i].positions[5]) );

            for(int j = 0; j < msg->goal.trajectory.joint_names.size(); j++)    //=6    
            {
                //ROS_INFO("[trajectory] %d-pos: %7.3f", j, msg->goal.trajectory.points[i].positions[j]);
                /* todo
                get a position & time_from_start
                convert radian to degree the position
                run MoveJ(position, time_From_start)
                */
                degrees[j] = rad2deg( msg->goal.trajectory.points[i].positions[j] );

                fTargetPos[i][j] = degrees[j];

            }
        }
        Drfl.movesj(fTargetPos, nCntTargetPos, 0.0, 0.0, targetTime, (MOVE_MODE)MOVE_MODE_ABSOLUTE);

        //Drfl.MoveJAsync(degrees.data(), 30, 30, 0, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_OVERRIDE);
        /*
        for(int i = 0; i < NUM_JOINT; i++){
            ROS_INFO("[]::cmd %d-pos: %7.3f", i, joints[i].cmd);
            cmd_[i] = joints[i].cmd;
        }
        */
    }

    //----- SYSTEM Service Call-back functions ------------------------------------------------------------

    bool DRHWInterface::set_robot_mode_cb(dsr_msgs::SetRobotMode::Request& req, dsr_msgs::SetRobotMode::Response& res){
        res.success = false;
        res.success = Drfl.set_robot_mode((ROBOT_MODE)req.robot_mode);   
        return true;
    }
    
    bool DRHWInterface::get_robot_mode_cb(dsr_msgs::GetRobotMode::Request& req, dsr_msgs::GetRobotMode::Response& res){
        res.success = false;
        res.robot_mode = Drfl.get_robot_mode();
        res.success = true;
        return true;
    }

    bool DRHWInterface::set_robot_system_cb(dsr_msgs::SetRobotSystem::Request& req, dsr_msgs::SetRobotSystem::Response& res){
        res.success = false;
        res.success = Drfl.set_robot_system((ROBOT_SYSTEM)req.robot_system);
        return true;
    }
    bool DRHWInterface::get_robot_system_cb(dsr_msgs::GetRobotSystem::Request& req, dsr_msgs::GetRobotSystem::Response& res){
        res.success = false;
        res.robot_system = Drfl.get_robot_system();
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_robot_state_cb(dsr_msgs::GetRobotState::Request& req, dsr_msgs::GetRobotState::Response& res){
        res.success = false;
        res.robot_state = Drfl.get_robot_state();
        return true;
    }
    bool DRHWInterface::set_robot_speed_mode_cb(dsr_msgs::SetRobotSpeedMode::Request& req, dsr_msgs::SetRobotSpeedMode::Response& res){
        res.success = false;
        res.success = Drfl.set_robot_speed_mode((SPEED_MODE)req.speed_mode);
        return true;
    }
    bool DRHWInterface::get_robot_speed_mode_cb(dsr_msgs::GetRobotSpeedMode::Request& req, dsr_msgs::GetRobotSpeedMode::Response& res){
        res.success = false;
        res.speed_mode = Drfl.get_robot_speed_mode();
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_current_pose_cb(dsr_msgs::GetCurrentPose::Request& req, dsr_msgs::GetCurrentPose::Response& res){
        res.success = false;
        LPROBOT_POSE robot_pos = Drfl.get_current_pose((ROBOT_SPACE)req.space_type);
        for(int i = 0; i < NUM_TASK; i++){
            res.pos[i] = robot_pos->_fPosition[i];
        }
        res.success = true;
        return true;
    }
    bool DRHWInterface::set_safe_stop_reset_type_cb(dsr_msgs::SetSafeStopResetType::Request& req, dsr_msgs::SetSafeStopResetType::Response& res){
        res.success = false;
        Drfl.SetSafeStopResetType((SAFE_STOP_RESET_TYPE)req.reset_type);
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_last_alarm_cb(dsr_msgs::GetLastAlarm::Request& req, dsr_msgs::GetLastAlarm::Response& res){
        res.success = false;
        res.log_alarm.level = Drfl.get_last_alarm()->_iLevel;
        res.log_alarm.group = Drfl.get_last_alarm()->_iGroup;
        res.log_alarm.index = Drfl.get_last_alarm()->_iIndex;
        for(int i = 0; i < 3; i++){
            std::string str_temp(Drfl.get_last_alarm()->_szParam[i]);
            res.log_alarm.param[i] = str_temp;
        }
        res.success = true;
        return true;
    }

    bool DRHWInterface::set_robot_control_cb(dsr_msgs::SetRobotControl::Request& req, dsr_msgs::SetRobotControl::Response& res){
        res.success = false;
        Drfl.set_robot_control((ROBOT_CONTROL)req.robot_control);
        res.success = true;
        return true;
    }

    bool DRHWInterface::manage_access_control_cb(dsr_msgs::ManageAccessControl::Request& req, dsr_msgs::ManageAccessControl::Response& res){
        res.success = false;
        Drfl.manage_access_control((MANAGE_ACCESS_CONTROL)req.access_control);
        res.success = true;
        return true;
    }
    
    bool DRHWInterface::release_protective_stop_cb(dsr_msgs::ReleaseProtectiveStop::Request& req, dsr_msgs::ReleaseProtectiveStop::Response& res){
        res.success = false;
        Drfl.release_protective_stop((RELEASE_MODE)req.release_mode);
        res.success = true;
        return true;
    }

    //----- MOTION Service Call-back functions ------------------------------------------------------------

    bool DRHWInterface::movej_cb(dsr_msgs::MoveJoint::Request& req, dsr_msgs::MoveJoint::Response& res)
    {
        res.success = false;
        std::array<float, NUM_JOINT> target_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::movej_cb() called and calling Drfl.MoveJ");
            res.success = Drfl.movej(target_pos.data(), req.vel, req.acc, req.time, (MOVE_MODE)req.mode, req.radius, (BLENDING_SPEED_TYPE)req.blendType);   
        }
        else{
            //ROS_INFO("DRHWInterface::movej_cb() called and calling Drfl.MoveJAsync");
            res.success = Drfl.amovej(target_pos.data(), req.vel, req.acc, req.time, (MOVE_MODE)req.mode, (BLENDING_SPEED_TYPE)req.blendType);
        }
        return true;
    }
    bool DRHWInterface::movel_cb(dsr_msgs::MoveLine::Request& req, dsr_msgs::MoveLine::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> target_pos;
        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        std::copy(req.vel.cbegin(), req.vel.cend(), target_vel.begin());
        std::copy(req.acc.cbegin(), req.acc.cend(), target_acc.begin());

        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::movel_cb() called and calling Drfl.MoveL");
            res.success = Drfl.movel(target_pos.data(), target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, req.radius, (BLENDING_SPEED_TYPE)req.blendType);
        }
        else{
            //ROS_INFO("DRHWInterface::movel_cb() called and calling Drfl.MoveLAsync");
            res.success = Drfl.amovel(target_pos.data(), target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (BLENDING_SPEED_TYPE)req.blendType);
        }
        return true;
    }
    bool DRHWInterface::movejx_cb(dsr_msgs::MoveJointx::Request& req, dsr_msgs::MoveJointx::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> target_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::movejx_cb() called and calling Drfl.MoveJX");
            res.success = Drfl.movejx(target_pos.data(), req.sol, req.vel, req.acc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, req.radius, (BLENDING_SPEED_TYPE)req.blendType);    
        }
        else{
            //ROS_INFO("DRHWInterface::movejx_cb() called and calling Drfl.MoveJXAsync");
            res.success = Drfl.amovejx(target_pos.data(), req.sol, req.vel, req.acc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (BLENDING_SPEED_TYPE)req.blendType);    
        }
        return true;
    }
    bool DRHWInterface::movec_cb(dsr_msgs::MoveCircle::Request& req, dsr_msgs::MoveCircle::Response& res)
    {
        res.success = false;
        float fTargetPos[2][NUM_TASK];
        float fTargetVel[2];
        float fTargetAcc[2];
        for(int i = 0; i < 2; i++){
            for(int j = 0; j < NUM_TASK; j++){
                std_msgs::Float64MultiArray pos = req.pos.at(i);
                fTargetPos[i][j] = pos.data[j];
            }
            fTargetVel[i] = req.vel[i];
            fTargetAcc[i] = req.acc[i];
        }
        ///ROS_INFO("  <xxx pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
        ///ROS_INFO("  <xxx pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);
        if(req.syncType == 0){   
            //ROS_INFO("DRHWInterface::movec_cb() called and calling Drfl.MoveC");
            res.success = Drfl.movec(fTargetPos, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, req.radius, (BLENDING_SPEED_TYPE)req.blendType);      
        }
        else{
            //ROS_INFO("DRHWInterface::movec_cb() called and calling Drfl.MoveCAsync");
            res.success = Drfl.amovec(fTargetPos, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (BLENDING_SPEED_TYPE)req.blendType);  
        }
        return true;
    }
    bool DRHWInterface::movesj_cb(dsr_msgs::MoveSplineJoint::Request& req, dsr_msgs::MoveSplineJoint::Response& res)
    {
        res.success = false;
        float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT];
        float fTargetVel[NUM_JOINT];
        float fTargetAcc[NUM_JOINT];

        for(int i=0; i<req.posCnt; i++){
            for(int j=0; j<NUM_JOINT; j++){
                std_msgs::Float64MultiArray pos = req.pos.at(i);
                fTargetPos[i][j] = pos.data[j];
            }
        }

        for(int i = 0; i < NUM_JOINT; i++){
            fTargetVel[i] = req.vel[i];
            fTargetAcc[i] = req.acc[i];
        }

        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::movesj_cb() called and calling Drfl.MoveSJ");
            res.success = Drfl.movesj(fTargetPos, req.posCnt, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode);
        }
        else{
            //ROS_INFO("DRHWInterface::movesj_cb() called and calling Drfl.MoveSJAsync");
            res.success = Drfl.amovesj(fTargetPos, req.posCnt, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode);
        }
        return true;
    }
    bool DRHWInterface::movesx_cb(dsr_msgs::MoveSplineTask::Request& req, dsr_msgs::MoveSplineTask::Response& res)
    {
        res.success = false;
        float fTargetPos[MAX_SPLINE_POINT][NUM_TASK];
        float fTargetVel[2];
        float fTargetAcc[2];

        for(int i=0; i<req.posCnt; i++){
            for(int j=0; j<NUM_TASK; j++){
                std_msgs::Float64MultiArray pos = req.pos.at(i);
                fTargetPos[i][j] = pos.data[j];
            }
          //  fTargetVel[i] = req.vel[i];
          //  fTargetAcc[i] = req.acc[i];
        }
        for(int i=0; i<2; i++){
            fTargetVel[i] = req.vel[i];
            fTargetAcc[i] = req.acc[i];
        }
        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::movesx_cb() called and calling Drfl.MoveSX");
            res.success = Drfl.movesx(fTargetPos, req.posCnt, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (SPLINE_VELOCITY_OPTION)req.opt);
        }
        else{
            //ROS_INFO("DRHWInterface::movesx_cb() called and calling Drfl.MoveSXAsync");
            res.success = Drfl.amovesx(fTargetPos, req.posCnt, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (SPLINE_VELOCITY_OPTION)req.opt);
        }
        return true;
    }
    bool DRHWInterface::moveb_cb(dsr_msgs::MoveBlending::Request& req, dsr_msgs::MoveBlending::Response& res)
    {
        res.success = false;
        MOVE_POSB posb[req.posCnt];
        for(int i=0; i<req.posCnt; i++){
            std_msgs::Float64MultiArray segment = req.segment.at(i);
            for(int j=0; j<NUM_TASK; j++){
                posb[i]._fTargetPos[0][j] = segment.data[j];            //0~5
                posb[i]._fTargetPos[1][j] = segment.data[j + NUM_TASK]; //6~11
            }
            posb[i]._iBlendType = segment.data[NUM_TASK + NUM_TASK];    //12
            posb[i]._fBlendRad = segment.data[NUM_TASK + NUM_TASK +1];  //13
        }

        /*
        for(int i=0; i<req.posCnt; i++){
            printf("----- segment %d -----\n",i);
            printf("    pos1: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                    ,posb[i]._fTargetPos[0][0], posb[i]._fTargetPos[0][1], posb[i]._fTargetPos[0][2], posb[i]._fTargetPos[0][3], posb[i]._fTargetPos[0][4], posb[i]._fTargetPos[0][5]);

            printf("    pos2: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                    ,posb[i]._fTargetPos[1][0], posb[i]._fTargetPos[1][1], posb[i]._fTargetPos[1][2], posb[i]._fTargetPos[1][3], posb[i]._fTargetPos[1][4], posb[i]._fTargetPos[1][5]);

            printf("    posb[%d]._iBlendType = %d\n",i,posb[i]._iBlendType); 
            printf("    posb[%d]._fBlendRad  = %f\n",i,posb[i]._fBlendRad); 
        }
        */

        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req.vel.cbegin(), req.vel.cend(), target_vel.begin());
        std::copy(req.acc.cbegin(), req.acc.cend(), target_acc.begin());

        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::moveb_cb() called and calling Drfl.MoveB");
            res.success = Drfl.moveb(posb, req.posCnt, target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref);
        }
        else{
            //ROS_INFO("DRHWInterface::moveb_cb() called and calling Drfl.MoveBAsync");
            res.success = Drfl.amoveb(posb, req.posCnt, target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref);
        }
        return true;
    }
    bool DRHWInterface::movespiral_cb(dsr_msgs::MoveSpiral::Request& req, dsr_msgs::MoveSpiral::Response& res)
    {
        res.success = false;
        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req.vel.cbegin(), req.vel.cend(), target_vel.begin());
        std::copy(req.acc.cbegin(), req.acc.cend(), target_acc.begin());

        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::movespiral_cb() called and calling Drfl.MoveSpiral");
            res.success = Drfl.move_spiral((TASK_AXIS)req.taskAxis, req.revolution, req.maxRadius, req.maxLength, target_vel.data(), target_acc.data(), req.time, (MOVE_REFERENCE)req.ref);
        }
        else{
            //ROS_INFO("DRHWInterface::movespiral_cb() called and calling Drfl.MoveSpiralAsync");
            res.success = Drfl.amove_spiral((TASK_AXIS)req.taskAxis, req.revolution, req.maxRadius, req.maxLength, target_vel.data(), target_acc.data(), req.time, (MOVE_REFERENCE)req.ref);
        }
        return true;
    }
    bool DRHWInterface::moveperiodic_cb(dsr_msgs::MovePeriodic::Request& req, dsr_msgs::MovePeriodic::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> target_amp;
        std::array<float, NUM_TASK> target_periodic;
        std::copy(req.amp.cbegin(), req.amp.cend(), target_amp.begin());
        std::copy(req.periodic.cbegin(), req.periodic.cend(), target_periodic.begin());
        if(req.syncType == 0){
            //ROS_INFO("DRHWInterface::moveperiodic_cb() called and calling Drfl.MovePeriodic");
            res.success = Drfl.move_periodic(target_amp.data(), target_periodic.data(), req.acc, req.repeat, (MOVE_REFERENCE)req.ref);
        }
        else{
            //ROS_INFO("DRHWInterface::moveperiodic_cb() called and calling Drfl.MovePeriodicAsync");
            res.success = Drfl.amove_periodic(target_amp.data(), target_periodic.data(), req.acc, req.repeat, (MOVE_REFERENCE)req.ref);
        }
        return true;
    }

    bool DRHWInterface::movewait_cb(dsr_msgs::MoveWait::Request& req, dsr_msgs::MoveWait::Response& res)
    {
        res.success = false;
        //ROS_INFO("DRHWInterface::movewait_cb() called and calling Drfl.MoveWait");
        res.success = Drfl.mwait();
        return true;
    }

    bool DRHWInterface::jog_cb(dsr_msgs::Jog::Request& req, dsr_msgs::Jog::Response& res)
    {
        res.success = false;
        ROS_INFO("DRHWInterface::jog_cb() called and calling Drfl.Jog");
        ROS_INFO("req.jog_axis = %d, req.move_reference=%d req.speed=%f",req.jog_axis, req.move_reference, req.speed);    

        res.success = Drfl.jog((JOG_AXIS)req.jog_axis, (MOVE_REFERENCE)req.move_reference, req.speed);
        return true;
    }

    bool DRHWInterface::jog_multi_cb(dsr_msgs::JogMulti::Request& req, dsr_msgs::JogMulti::Response& res)
    {
        res.success = false;
        ROS_INFO("DRHWInterface::jog_multi_cb() called and calling Drfl.MultiJog");
        ROS_INFO("req.jog_axis = %f,%f,%f,%f,%f,%f",req.jog_axis[0],req.jog_axis[1],req.jog_axis[2],req.jog_axis[3],req.jog_axis[4],req.jog_axis[5]);    

        std::array<float, NUM_JOINT> target_jog;
        std::copy(req.jog_axis.cbegin(), req.jog_axis.cend(), target_jog.begin());

        res.success = Drfl.multi_jog(target_jog.data(), (MOVE_REFERENCE)req.move_reference, req.speed);
        return true;
    }

    bool DRHWInterface::move_stop_cb(dsr_msgs::MoveStop::Request& req, dsr_msgs::MoveStop::Response& res)
    {
        res.success = false;
        res.success = Drfl.stop((STOP_TYPE)req.stop_mode);
        return true;
    }

    bool DRHWInterface::move_resume_cb(dsr_msgs::MoveResume::Request& req, dsr_msgs::MoveResume::Response& res)
    {
        res.success = false;
        res.success = Drfl.move_resume();
        return true;
    }
    
    bool DRHWInterface::move_pause_cb(dsr_msgs::MovePause::Request& req, dsr_msgs::MovePause::Response& res)
    {
        res.success = false;
        res.success = Drfl.move_pause();
        return true;
    }
    bool DRHWInterface::trans_cb(dsr_msgs::Trans::Request& req, dsr_msgs::Trans::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> target_pos;
        std::array<float, NUM_TASK> delta_pos;
 
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        std::copy(req.delta.cbegin(), req.delta.cend(), delta_pos.begin());
  
        LPROBOT_POSE robot_pos = Drfl.trans(target_pos.data(), delta_pos.data(), (COORDINATE_SYSTEM)req.ref, (COORDINATE_SYSTEM)req.ref_out);
        for(int i=0; i<NUM_TASK; i++){
            res.trans_pos[i] = robot_pos->_fPosition[i];
        }
        res.success = true;
        return true;
    }

    bool DRHWInterface::fkin_cb(dsr_msgs::Fkin::Request& req, dsr_msgs::Fkin::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> joint_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), joint_pos.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< fkin_cb >");
        ROS_INFO("    joint_pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]);
        ROS_INFO("    ref       = %d",req.ref);      
    #endif
        LPROBOT_POSE task_pos = Drfl.fkin(joint_pos.data(), (COORDINATE_SYSTEM)req.ref);
        for(int i=0; i<NUM_TASK; i++){
            res.conv_posx[i] = task_pos->_fPosition[i];
        }
        res.success = true;
        return true;
    }
    bool DRHWInterface::ikin_cb(dsr_msgs::Ikin::Request& req, dsr_msgs::Ikin::Response& res)
    {       
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), task_pos.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< ikin_cb >");
        ROS_INFO("    task_pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
        ROS_INFO("    ref       = %d",req.ref);      
    #endif

        LPROBOT_POSE joint_pos = Drfl.ikin(task_pos.data(), req.sol_space, (COORDINATE_SYSTEM)req.ref);
        for(int i=0; i<NUM_TASK; i++){
            res.conv_posj[i] = joint_pos->_fPosition[i];
        }
        res.success = true;
        return true;
    }

    bool DRHWInterface::ikin_ex_cb(dsr_msgs::IkinEx::Request& req, dsr_msgs::IkinEx::Response& res)
    {       
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), task_pos.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< ikin_cb >");
        ROS_INFO("    task_pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
        ROS_INFO("    ref       = %d",req.ref);
        ROS_INFO("    ref_pos_opt = %d",req.ref_pos_opt);      
    #endif

        LPINVERSE_KINEMATIC_RESPONSE joint_pos = Drfl.ikin(task_pos.data(), req.sol_space, (COORDINATE_SYSTEM)req.ref, req.ref_pos_opt);
        for(int i=0; i<NUM_TASK; i++){
            res.conv_posj[i] = joint_pos->_fTargetPos[i];
        }
        res.status = joint_pos->_iStatus;
        res.success = true;
        return true;
    }
	
    bool DRHWInterface::set_ref_coord_cb(dsr_msgs::SetRefCoord::Request& req, dsr_msgs::SetRefCoord::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< set_ref_coord_cb >");
        ROS_INFO("    coord = %d",req.coord);      
    #endif

        res.success = Drfl.set_ref_coord((COORDINATE_SYSTEM)req.coord);
        return true;
    }
    bool DRHWInterface::move_home_cb(dsr_msgs::MoveHome::Request& req, dsr_msgs::MoveHome::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< move_home_cb >");
        ROS_INFO("    target = %d",req.target);      
    #endif

        if(0 == req.target) 
            res.success = Drfl.move_home(MOVE_HOME_MECHANIC);
        else 
            res.success = Drfl.move_home(MOVE_HOME_USER);
        return true;
    }
    bool DRHWInterface::check_motion_cb(dsr_msgs::CheckMotion::Request& req, dsr_msgs::CheckMotion::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< check_motion_cb >");
    #endif

        res.status = Drfl.check_motion();
        res.success = true;
        return true;
    }
    bool DRHWInterface::change_operation_speed_cb(dsr_msgs::ChangeOperationSpeed::Request& req, dsr_msgs::ChangeOperationSpeed::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< change_operation_speed_cb >");
        ROS_INFO("    speed = %f",(float)req.speed);
    #endif

        res.success = Drfl.change_operation_speed((float)req.speed);
        return true;
    }
    bool DRHWInterface::enable_alter_motion_cb(dsr_msgs::EnableAlterMotion::Request& req, dsr_msgs::EnableAlterMotion::Response& res)
    {
        res.success = false;
        std::array<float, 2> limit;
        std::array<float, 2> limit_per;
        std::copy(req.limit_dPOS.cbegin(), req.limit_dPOS.cend(), limit.begin());
        std::copy(req.limit_dPOS_per.cbegin(), req.limit_dPOS_per.cend(), limit_per.begin());
 
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< enable_alter_motion_cb >");
        ROS_INFO("    n         = %d",req.n);
        ROS_INFO("    mode      = %d",req.mode);
        ROS_INFO("    ref       = %d",req.ref);
        ROS_INFO("    limit     = %7.3f,%7.3f",limit[0],limit[1]);
        ROS_INFO("    limit_per = %7.3f,%7.3f",limit_per[0],limit_per[1]);
    #endif

        res.success = Drfl.enable_alter_motion((int)req.n, (PATH_MODE)req.mode, (COORDINATE_SYSTEM)req.ref, limit.data(), limit_per.data());
        return true;
    }
    bool DRHWInterface::alter_motion_cb(dsr_msgs::AlterMotion::Request& req, dsr_msgs::AlterMotion::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> pos_alter;
        std::copy(req.pos.cbegin(), req.pos.cend(), pos_alter.begin());
 
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< alter_motion_cb >");
        ROS_INFO("    pos_alter = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",pos_alter[0],pos_alter[1],pos_alter[2],pos_alter[3],pos_alter[4],pos_alter[5]);
    #endif

        res.success = Drfl.alter_motion(pos_alter.data());
        return true;
    }
    bool DRHWInterface::disable_alter_motion_cb(dsr_msgs::DisableAlterMotion::Request& req, dsr_msgs::DisableAlterMotion::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< disable_alter_motion_cb >");
    #endif

        res.success = Drfl.disable_alter_motion();
        return true;
    }
    bool DRHWInterface::set_singularity_handling_cb(dsr_msgs::SetSingularityHandling::Request& req, dsr_msgs::SetSingularityHandling::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< set_singularity_handling_cb >");
        ROS_INFO("    mode = %d",req.mode);
    #endif

        res.success = Drfl.set_singularity_handling((SINGULARITY_AVOIDANCE)req.mode);
        return true;
    }


    //----- AUX Control Service Call-back functions ------------------------------------------------------------

    bool DRHWInterface::get_control_mode_cb(dsr_msgs::GetControlMode::Request& req, dsr_msgs::GetControlMode::Response& res)               
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_control_mode_cb >");
    #endif
        //NO API , get mon_data      
        res.control_mode = g_stDrState.nActualMode;
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_control_space_cb(dsr_msgs::GetControlSpace::Request& req, dsr_msgs::GetControlSpace::Response& res)              
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_control_space_cb >");
    #endif
        //NO API , get mon_data
        res.space = g_stDrState.nActualSpace;
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_current_posj_cb(dsr_msgs::GetCurrentPosj::Request& req, dsr_msgs::GetCurrentPosj::Response& res)               
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_current_posj_cb >");
    #endif
        LPROBOT_POSE robot_pos = Drfl.get_current_pose((ROBOT_SPACE)ROBOT_SPACE_JOINT);
        for(int i = 0; i < NUM_TASK; i++){
            res.pos[i] = robot_pos->_fPosition[i];
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_current_velj_cb(dsr_msgs::GetCurrentVelj::Request& req, dsr_msgs::GetCurrentVelj::Response& res)               
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_current_velj_cb >");
    #endif

        //NO API , get mon_data
        for(int i=0; i<NUM_TASK; i++){
            res.joint_speed[i] = g_stDrState.fCurrentVelj[i];
        }
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_desired_posj_cb(dsr_msgs::GetDesiredPosj::Request& req, dsr_msgs::GetDesiredPosj::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_desired_posj_cb >");
    #endif
        //NO API , get mon_data
        for(int i=0; i<NUM_TASK; i++){
            res.pos[i] = g_stDrState.fTargetPosj[i];
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_desired_velj_cb(dsr_msgs::GetDesiredVelj::Request& req, dsr_msgs::GetDesiredVelj::Response& res)              
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_desired_velj_cb >");
    #endif
        //NO API , get mon_data
        for(int i=0; i<NUM_TASK; i++){
            res.joint_vel[i] = g_stDrState.fTargetVelj[i];
        }
        res.success = true;        
        return true;
    }

    bool DRHWInterface::get_current_posx_cb(dsr_msgs::GetCurrentPosx::Request& req, dsr_msgs::GetCurrentPosx::Response& res)               
    {
        res.success = false;
        std_msgs::Float64MultiArray arr;

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_current_posx_cb >");
    #endif

        LPROBOT_TASK_POSE cur_posx = Drfl.get_current_posx((COORDINATE_SYSTEM)req.ref);
        arr.data.clear();
        for (int i = 0; i < NUM_TASK; i++){
            arr.data.push_back(cur_posx->_fTargetPos[i]);
        }
        arr.data.push_back(cur_posx->_iTargetSol);
        res.task_pos_info.push_back(arr);
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_current_velx_cb(dsr_msgs::GetCurrentVelx::Request& req, dsr_msgs::GetCurrentVelx::Response& res)               
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_current_velx_cb >");
    #endif
   
        //NO API , get mon_data
        for(int i=0; i<NUM_TASK; i++){
            res.vel[i] = g_stDrState.fCurrentVelx[i];
        }
        res.success = true;            
        return true;
    }
    bool DRHWInterface::get_desired_posx_cb(dsr_msgs::GetDesiredPosx::Request& req, dsr_msgs::GetDesiredPosx::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_desired_posx_cb >");
    #endif
        LPROBOT_POSE task_pos = Drfl.get_desired_posx((COORDINATE_SYSTEM)req.ref);
        for(int i=0; i<NUM_TASK; i++){
            res.pos[i] = task_pos->_fPosition[i];
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_desired_velx_cb(dsr_msgs::GetDesiredVelx::Request& req, dsr_msgs::GetDesiredVelx::Response& res)               
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_desired_velx_cb >");
    #endif
        //NO API , get mon_data
        for(int i=0; i<NUM_TASK; i++){
            res.vel[i] = g_stDrState.fTargetVelx[i];
        }
        res.success = true;                    
        return true;
    }
    bool DRHWInterface::get_current_tool_flange_posx_cb(dsr_msgs::GetCurrentToolFlangePosx::Request& req, dsr_msgs::GetCurrentToolFlangePosx::Response& res)                                                          
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_current_tool_flange_posx_cb >");
    #endif
        //NO API , get mon_data
        for(int i=0; i<NUM_TASK; i++){
            res.pos[i] = g_stDrState.fCurrentToolPosx[i];
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_current_solution_space_cb(dsr_msgs::GetCurrentSolutionSpace::Request& req, dsr_msgs::GetCurrentSolutionSpace::Response& res)     
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_current_solution_space_cb >");
    #endif
        res.sol_space = Drfl.get_current_solution_space();
        res.success = true;
        return true;
    }    
    bool DRHWInterface::get_current_rotm_cb(dsr_msgs::GetCurrentRotm::Request& req, dsr_msgs::GetCurrentRotm::Response& res)               
    {
        res.success = false;
        std_msgs::Float64MultiArray arr;

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_current_rotm_cb >");
    #endif
        //NO API , get mon_data
        for (int i = 0; i < 3; i++){
            arr.data.clear();
            for (int j = 0; j < 3; j++){
                arr.data.push_back(g_stDrState.fRotationMatrix[i][j]);
            }
            res.rot_matrix.push_back(arr);
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_joint_torque_cb(dsr_msgs::GetJointTorque::Request& req, dsr_msgs::GetJointTorque::Response& res)               
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_joint_torque_cb >");
    #endif
        //NO API , get mon_data
        for(int i = 0; i < NUM_TASK; i++){
            res.jts[i] = g_stDrState.fActualJTS[i];
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_external_torque_cb(dsr_msgs::GetExternalTorque::Request& req, dsr_msgs::GetExternalTorque::Response& res)           
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_external_torque_cb >");
    #endif
        //NO API , get mon_data
        for(int i = 0; i < NUM_TASK; i++){
            res.ext_torque[i] = g_stDrState.fActualEJT[i];
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_tool_force_cb(dsr_msgs::GetToolForce::Request& req, dsr_msgs::GetToolForce::Response& res)                 
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_tool_force_cb >");
    #endif
        //NO API , get mon_data
        for(int i = 0; i < NUM_TASK; i++){
            res.tool_force[i] = g_stDrState.fActualETT[i];
        }
        res.success = true;        
        return true;
    }
    bool DRHWInterface::get_solution_space_cb(dsr_msgs::GetSolutionSpace::Request& req, dsr_msgs::GetSolutionSpace::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), task_pos.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_solution_space_cb >");
        ROS_INFO("    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    #endif
        res.sol_space = Drfl.get_solution_space(task_pos.data());
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_orientation_error_cb(dsr_msgs::GetOrientationError::Request& req, dsr_msgs::GetOrientationError::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos1;
        std::array<float, NUM_TASK> task_pos2;

        std::copy(req.xd.cbegin(), req.xd.cend(), task_pos1.begin());
        std::copy(req.xc.cbegin(), req.xc.cend(), task_pos2.begin());

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_orientation_error_cb >");
        ROS_INFO("    xd = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
        ROS_INFO("    xc = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);      
        ROS_INFO("    axis = %d",req.axis);
    #endif
        res.ori_error = Drfl.get_orientation_error(task_pos1.data(), task_pos2.data(), (TASK_AXIS)req.axis);
        res.success = true;
        return true;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool DRHWInterface::get_workpiece_weight_cb(dsr_msgs::GetWorkpieceWeight::Request& req, dsr_msgs::GetWorkpieceWeight::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_workpiece_weight_cb >");
    #endif
        res.weight = Drfl.get_workpiece_weight();
        res.success = true;
        return true;
    } 
    bool DRHWInterface::reset_workpiece_weight_cb(dsr_msgs::ResetWorkpieceWeight::Request& req, dsr_msgs::ResetWorkpieceWeight::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< reset_workpiece_weight_cb >");
    #endif
        res.success = Drfl.reset_workpiece_weight();

        return true;
    } 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //----- FORCE Control Service Call-back functions ------------------------------------------------------------

    bool DRHWInterface::parallel_axis1_cb(dsr_msgs::ParallelAxis1::Request& req, dsr_msgs::ParallelAxis1::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos1;
        std::array<float, NUM_TASK> task_pos2;
        std::array<float, NUM_TASK> task_pos3;
 
        std::copy(req.x1.cbegin(), req.x1.cend(), task_pos1.begin());
        std::copy(req.x2.cbegin(), req.x2.cend(), task_pos2.begin());
        std::copy(req.x3.cbegin(), req.x3.cend(), task_pos3.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< parallel_axis1_cb >");
        ROS_INFO("    x1 = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
        ROS_INFO("    x2 = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
        ROS_INFO("    x3 = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos3[0],task_pos3[1],task_pos3[2],task_pos3[3],task_pos3[4],task_pos3[5]);
    #endif
        res.success = Drfl.parallel_axis(task_pos1.data(), task_pos2.data(), task_pos3.data(), (TASK_AXIS)req.axis, (COORDINATE_SYSTEM)req.ref);

        return true;
    }
    bool DRHWInterface::parallel_axis2_cb(dsr_msgs::ParallelAxis2::Request& req, dsr_msgs::ParallelAxis2::Response& res)
    {
        res.success = false;
        std::array<float, 3> vector;
 
        std::copy(req.vect.cbegin(), req.vect.cend(), vector.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< parallel_axis2_cb >");
        ROS_INFO("    vect = %7.3f,%7.3f,%7.3f",vector[0],vector[1],vector[2]);
        ROS_INFO("    axis = %d",req.axis);
        ROS_INFO("    ref  = %d",req.ref);
    #endif
        res.success = Drfl.parallel_axis(vector.data(), (TASK_AXIS)req.axis, (COORDINATE_SYSTEM)req.ref);

        return true;
    }
    bool DRHWInterface::align_axis1_cb(dsr_msgs::AlignAxis1::Request& req, dsr_msgs::AlignAxis1::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos1;
        std::array<float, NUM_TASK> task_pos2;
        std::array<float, NUM_TASK> task_pos3;
        //std::array<float, NUM_TASK> task_pos4;
        float fSourceVec[3] = {0, };

        std::copy(req.x1.cbegin(), req.x1.cend(), task_pos1.begin());
        std::copy(req.x2.cbegin(), req.x2.cend(), task_pos2.begin());
        std::copy(req.x3.cbegin(), req.x3.cend(), task_pos3.begin());
        //std::copy(req.pos.cbegin(),req.pos.cend(),task_pos4.begin());
          //req.pos[6] -> fTargetVec[3] : only use [x,y,z]    
        for(int i=0; i<3; i++)        
            fSourceVec[i] = req.source_vect[i];

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< align_axis1_cb >");
        ROS_INFO("    x1  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
        ROS_INFO("    x2  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
        ROS_INFO("    x3  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos3[0],task_pos3[1],task_pos3[2],task_pos3[3],task_pos3[4],task_pos3[5]);
        ROS_INFO("    pos = %7.3f,%7.3f,%7.3f",fSourceVec[0],fSourceVec[1],fSourceVec[2]);
        ROS_INFO("    axis = %d",req.axis);
        ROS_INFO("    ref  = %d",req.ref);
    #endif
        res.success = Drfl.align_axis(task_pos1.data(), task_pos2.data(), task_pos3.data(), fSourceVec, (TASK_AXIS)req.axis, (COORDINATE_SYSTEM)req.ref);
        return true;
    }
    bool DRHWInterface::align_axis2_cb(dsr_msgs::AlignAxis2::Request& req, dsr_msgs::AlignAxis2::Response& res)
    {
        res.success = false;
        float fTargetVec[3] = {0, };
        float fSourceVec[3] = {0, };

        for(int i=0; i<3; i++)
        {        
            fTargetVec[i] = req.target_vect[i];
            fSourceVec[i] = req.source_vect[i];     ////req.pos[6] -> fSourceVec[3] : only use [x,y,z]
        }
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< align_axis2_cb >");
        ROS_INFO("    vect = %7.3f,%7.3f,%7.3f",fTargetVec[0],fTargetVec[1],fTargetVec[2]);
        ROS_INFO("    pos  = %7.3f,%7.3f,%7.3f",fSourceVec[0],fSourceVec[1],fSourceVec[2]);
        ROS_INFO("    axis = %d",req.axis);
        ROS_INFO("    ref  = %d",req.ref);
    #endif
        res.success = Drfl.align_axis(fTargetVec, fSourceVec, (TASK_AXIS)req.axis, (COORDINATE_SYSTEM)req.ref);

        return true;
    }
    bool DRHWInterface::is_done_bolt_tightening_cb(dsr_msgs::IsDoneBoltTightening::Request& req, dsr_msgs::IsDoneBoltTightening::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< is_done_bolt_tightening_cb >");
        ROS_INFO("    m       = %f",req.m);
        ROS_INFO("    timeout = %f",req.timeout);
        ROS_INFO("    axis    = %d",req.axis);
    #endif
        res.success = Drfl.is_done_bolt_tightening((FORCE_AXIS)req.axis, req.m, req.timeout);

        return true;
    }
    bool DRHWInterface::release_compliance_ctrl_cb(dsr_msgs::ReleaseComplianceCtrl::Request& req, dsr_msgs::ReleaseComplianceCtrl::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< release_compliance_ctrl_cb >");
    #endif
        res.success = Drfl.release_compliance_ctrl();

        return true;
    }
    bool DRHWInterface::task_compliance_ctrl_cb(dsr_msgs::TaskComplianceCtrl::Request& req, dsr_msgs::TaskComplianceCtrl::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> stiffnesses;
 
        std::copy(req.stx.cbegin(), req.stx.cend(), stiffnesses.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< task_compliance_ctrl_cb >");
        ROS_INFO("    stx     = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",stiffnesses[0],stiffnesses[1],stiffnesses[2],stiffnesses[3],stiffnesses[4],stiffnesses[5]);
        ROS_INFO("    ref     = %d",req.ref);
        ROS_INFO("    timeout = %f",req.time);
    #endif
        res.success = Drfl.task_compliance_ctrl(stiffnesses.data(), (COORDINATE_SYSTEM)req.ref, req.time);

        return true;
    }
    bool DRHWInterface::set_stiffnessx_cb(dsr_msgs::SetStiffnessx::Request& req, dsr_msgs::SetStiffnessx::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> stiffnesses;
 
        std::copy(req.stx.cbegin(), req.stx.cend(), stiffnesses.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< set_stiffnessx_cb >");
        ROS_INFO("    stx     = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",stiffnesses[0],stiffnesses[1],stiffnesses[2],stiffnesses[3],stiffnesses[4],stiffnesses[5]);
        ROS_INFO("    ref     = %d",req.ref);
        ROS_INFO("    timeout = %f",req.time);
    #endif
        res.success = Drfl.set_stiffnessx(stiffnesses.data(), (COORDINATE_SYSTEM)req.ref, req.time);

        return true;
    }
    bool DRHWInterface::calc_coord_cb(dsr_msgs::CalcCoord::Request& req, dsr_msgs::CalcCoord::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos1;
        std::array<float, NUM_TASK> task_pos2;
        std::array<float, NUM_TASK> task_pos3;
        std::array<float, NUM_TASK> task_pos4;
 
        std::copy(req.x1.cbegin(), req.x1.cend(), task_pos1.begin());
        std::copy(req.x2.cbegin(), req.x2.cend(), task_pos2.begin());
        std::copy(req.x3.cbegin(), req.x3.cend(), task_pos3.begin());
        std::copy(req.x4.cbegin(), req.x4.cend(), task_pos4.begin());

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< calc_coord_cb >");
        ROS_INFO("    input_pos_cnt = %d",req.input_pos_cnt); 
        ROS_INFO("    x1  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
        ROS_INFO("    x2  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
        ROS_INFO("    x3  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos3[0],task_pos3[1],task_pos3[2],task_pos3[3],task_pos3[4],task_pos3[5]);
        ROS_INFO("    x4  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos4[0],task_pos4[1],task_pos4[2],task_pos4[3],task_pos4[4],task_pos4[5]);
        ROS_INFO("    ref = %d",req.ref);
        ROS_INFO("    mod = %d",req.mod);
    #endif
        LPROBOT_POSE task_pos = Drfl.calc_coord(req.input_pos_cnt, req.mod, (COORDINATE_SYSTEM)req.ref, task_pos1.data(), task_pos2.data(), task_pos3.data(), task_pos4.data());
        for(int i=0; i<NUM_TASK; i++){
            res.conv_posx[i] = task_pos->_fPosition[i];
        }
        res.success = true;
        return true;
    }
    
    bool DRHWInterface::set_user_cart_coord1_cb(dsr_msgs::SetUserCartCoord1::Request& req, dsr_msgs::SetUserCartCoord1::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
 
        std::copy(req.pos.cbegin(), req.pos.cend(), task_pos.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< set_user_cart_coord1_cb >");
        ROS_INFO("    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
        ROS_INFO("    ref = %d",req.ref);
    #endif
        res.id = Drfl.set_user_cart_coord(0, task_pos.data(), (COORDINATE_SYSTEM)req.ref);  
        res.success = true;
        return true;
    }
    bool DRHWInterface::set_user_cart_coord2_cb(dsr_msgs::SetUserCartCoord2::Request& req, dsr_msgs::SetUserCartCoord2::Response& res)
    {
        res.success = false;
        //std::array<float, NUM_TASK> task_pos1;
        //std::array<float, NUM_TASK> task_pos2;
        //std::array<float, NUM_TASK> task_pos3;
        //std::array<float, NUM_TASK> target_org;
        float fTargetPos[3][NUM_TASK] = {0, };
        float fTargetOrg[3] = {0, };
        
        //req.x1[6] + req.x2[6] + req.x3[6] -> fTargetPos[3][NUM_TASK] 
        for(int i=0; i<NUM_TASK; i++)        
        {
            fTargetPos[0][i] = req.x1[i];
            fTargetPos[1][i] = req.x2[i];
            fTargetPos[2][i] = req.x3[i];
        }
        //req.pos[6] -> fTargetOrg[3] : only use [x,y,z]    
        for(int i=0; i<3; i++)        
            fTargetOrg[i] = req.pos[i];

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< set_user_cart_coord2_cb >");
        ROS_INFO("    x1  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
        ROS_INFO("    x2  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);
        ROS_INFO("    x3  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetPos[2][0],fTargetPos[2][1],fTargetPos[2][2],fTargetPos[2][3],fTargetPos[2][4],fTargetPos[2][5]);
        ROS_INFO("    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetOrg[0],fTargetOrg[1],fTargetOrg[2],fTargetOrg[3],fTargetOrg[4],fTargetOrg[5]);
        ROS_INFO("    ref = %d",req.ref);
    #endif
        res.id = Drfl.set_user_cart_coord(fTargetPos, fTargetOrg, (COORDINATE_SYSTEM)req.ref);
        res.success = true;
        return true;
    }
    bool DRHWInterface::set_user_cart_coord3_cb(dsr_msgs::SetUserCartCoord3::Request& req, dsr_msgs::SetUserCartCoord3::Response& res)
    {
        res.success = false;
        float fTargetVec[2][3] = {0, };
        float fTargetOrg[3] = {0, };

        //req.u1[3] + req.c1[3] -> fTargetVec[2][3]
        for(int i=0; i<3; i++)        
        {
            fTargetVec[0][i] = req.u1[i];
            fTargetVec[1][i] = req.v1[i];
        }
        //req.pos[6] -> fTargetOrg[3] : only use [x,y,z]    
        for(int i=0; i<3; i++)        
            fTargetOrg[i] = req.pos[i];

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< set_user_cart_coord3_cb >");
        ROS_INFO("    u1  = %7.3f,%7.3f,%7.3f",fTargetVec[0][0],fTargetVec[0][1],fTargetVec[0][2]);
        ROS_INFO("    v1  = %7.3f,%7.3f,%7.3f",fTargetVec[1][0],fTargetVec[1][1],fTargetVec[1][2]);
        ROS_INFO("    org = %7.3f,%7.3f,%7.3f",fTargetOrg[0],fTargetOrg[1],fTargetOrg[2]);
        ROS_INFO("    ref = %d",req.ref);
    #endif
        res.id = Drfl.set_user_cart_coord(fTargetVec, fTargetOrg, (COORDINATE_SYSTEM)req.ref);
        res.success = true;
        return true;
    }
    
    bool DRHWInterface::overwrite_user_cart_coord_cb(dsr_msgs::OverwriteUserCartCoord::Request& req, dsr_msgs::OverwriteUserCartCoord::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
 
        std::copy(req.pos.cbegin(),req.pos.cend(),task_pos.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< overwrite_user_cart_coord_cb >");
        ROS_INFO("    id  = %d",req.id);
        ROS_INFO("    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
        ROS_INFO("    ref = %d",req.ref);
    #endif
        res.id = Drfl.overwrite_user_cart_coord(0, req.id, task_pos.data(), (COORDINATE_SYSTEM)req.ref);  //0=AUTO 
        res.success = true;
        return true;
    }
    bool DRHWInterface::get_user_cart_coord_cb(dsr_msgs::GetUserCartCoord::Request& req, dsr_msgs::GetUserCartCoord::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< get_user_cart_coord_cb >");
        ROS_INFO("    id  = %d",req.id);
    #endif
        LPUSER_COORDINATE result = Drfl.get_user_cart_coord(req.id);
        for(int i=0; i<NUM_TASK; i++){
            res.conv_posx[i] = result->_fTargetPos[i];
        }
        res.ref = result->_iTargetRef;
        res.success = true;
        return true;
    }
    bool DRHWInterface::set_desired_force_cb(dsr_msgs::SetDesiredForce::Request& req, dsr_msgs::SetDesiredForce::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> feedback;
        std::array<unsigned char, NUM_TASK> direction;
 
        std::copy(req.fd.cbegin(), req.fd.cend(), feedback.begin());
        std::copy(req.dir.cbegin(),req.dir.cend(), direction.begin());

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< set_desired_force_cb >");
        ROS_INFO("    feedback  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",feedback[0],feedback[1],feedback[2],feedback[3],feedback[4],feedback[5]);
        ROS_INFO("    direction = %d,%d,%d,%d,%d,%d",direction[0],direction[1],direction[2],direction[3],direction[4],direction[5]);
        ROS_INFO("    ref   = %d", req.ref);
        ROS_INFO("    time  = %f", req.time); 
        ROS_INFO("    mod   = %d", req.mod);
    #endif
        res.success = Drfl.set_desired_force(feedback.data(), direction.data(), (COORDINATE_SYSTEM)req.ref, req.time, (FORCE_MODE)req.mod);

        return true;
    }
    bool DRHWInterface::release_force_cb(dsr_msgs::ReleaseForce::Request& req, dsr_msgs::ReleaseForce::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< release_force_cb >");
    #endif
        res.success = Drfl.release_force(req.time);

        return true;
    }
    bool DRHWInterface::check_position_condition_cb(dsr_msgs::CheckPositionCondition::Request& req, dsr_msgs::CheckPositionCondition::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), task_pos.begin());
  
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< check_position_condition_cb >");
        ROS_INFO("    axis = %d", req.axis);
        ROS_INFO("    min  = %f", req.min);
        ROS_INFO("    max  = %f", req.max);
        ROS_INFO("    ref  = %d", req.ref);
        ROS_INFO("    mode = %d", req.mode);
        ROS_INFO("    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    #endif
        if(0==req.mode)  //DR_MV_MOD_ABS
            res.success = Drfl.check_position_condition_abs((FORCE_AXIS)req.axis, req.min, req.max, (COORDINATE_SYSTEM)req.ref);
        else            //DR_MV_MOD_REL
            res.success = Drfl.check_position_condition_rel((FORCE_AXIS)req.axis, req.min, req.max, task_pos.data(), (COORDINATE_SYSTEM)req.ref);
        return true;
    }
    bool DRHWInterface::check_force_condition_cb(dsr_msgs::CheckForceCondition::Request& req, dsr_msgs::CheckForceCondition::Response& res)
    {
        res.success = false;
    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< check_force_condition_cb >");
        ROS_INFO("    axis = %d", req.axis);
        ROS_INFO("    min  = %f", req.min);
        ROS_INFO("    max  = %f", req.max);
        ROS_INFO("    ref  = %d", req.ref);
    #endif
        res.success = Drfl.check_force_condition((FORCE_AXIS)req.axis, req.min, req.max, (COORDINATE_SYSTEM)req.ref);
        return true;
    }
    bool DRHWInterface::check_orientation_condition1_cb(dsr_msgs::CheckOrientationCondition1::Request& req, dsr_msgs::CheckOrientationCondition1::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos1;
        std::array<float, NUM_TASK> task_pos2;
        std::copy(req.min.cbegin(), req.min.cend(), task_pos1.begin());
        std::copy(req.max.cbegin(), req.max.cend(), task_pos2.begin());

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< check_orientation_condition1_cb >");
        ROS_INFO("    axis = %d", req.axis);
        ROS_INFO("    min = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
        ROS_INFO("    max = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
        ROS_INFO("    ref  = %d", req.ref);
        ROS_INFO("    mode = %d", req.mode);
    #endif
        res.success = Drfl.check_orientation_condition((FORCE_AXIS)req.axis , task_pos1.data(), task_pos2.data(), (COORDINATE_SYSTEM)req.ref);
        return true;
    }
    bool DRHWInterface::check_orientation_condition2_cb(dsr_msgs::CheckOrientationCondition2::Request& req, dsr_msgs::CheckOrientationCondition2::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
 
        std::copy(req.pos.cbegin(), req.pos.cend(), task_pos.begin());

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< check_orientation_condition2_cb >");
        ROS_INFO("    axis = %d", req.axis);
        ROS_INFO("    min  = %f", req.min);
        ROS_INFO("    max  = %f", req.max);
        ROS_INFO("    ref  = %d", req.ref);
        ROS_INFO("    mode = %d", req.mode);
        ROS_INFO("    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    #endif
        res.success = Drfl.check_orientation_condition((FORCE_AXIS)req.axis , req.min, req.max, task_pos.data(), (COORDINATE_SYSTEM)req.ref);
        return true;
    }
    bool DRHWInterface::coord_transform_cb(dsr_msgs::CoordTransform::Request& req, dsr_msgs::CoordTransform::Response& res)
    {
        res.success = false;
        std::array<float, NUM_TASK> task_pos;
 
        std::copy(req.pos_in.cbegin(), req.pos_in.cend(), task_pos.begin());

    #if (_DEBUG_DSR_CTL)
        ROS_INFO("< coord_transform_cb >");
        ROS_INFO("    pos_in  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
        ROS_INFO("    ref_in  = %d", req.ref_in);
        ROS_INFO("    ref_out = %d", req.ref_out);
    #endif
        LPROBOT_POSE result_pos = Drfl.coord_transform(task_pos.data(), (COORDINATE_SYSTEM)req.ref_in, (COORDINATE_SYSTEM)req.ref_out);
        for(int i=0; i<NUM_TASK; i++){
            res.conv_posx[i] = result_pos->_fPosition[i];
        }
        res.success = true;
        return true;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool DRHWInterface::set_digital_output_cb(dsr_msgs::SetCtrlBoxDigitalOutput::Request& req, dsr_msgs::SetCtrlBoxDigitalOutput::Response& res)
    {
        //ROS_INFO("DRHWInterface::set_digital_output_cb() called and calling Drfl.SetCtrlBoxDigitalOutput");
        res.success = false;

        if((req.index < DR_DIO_MIN_INDEX) || (req.index > DR_DIO_MAX_INDEX)){
            ROS_ERROR("set_digital_output(index=%d, value=%d): index(%d) is out of range. (normal range: %d ~ %d)",req.index ,req.value ,req.index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
        }       
        else if((req.value < 0) || (req.value > 1)){
            ROS_ERROR("set_digital_output(index=%d, value=%d): value(%d) is out of range. [normal range: 0 or 1]",req.index ,req.value ,req.value);
        }       
        else{
            req.index -=1;
            res.success = Drfl.set_digital_output((GPIO_CTRLBOX_DIGITAL_INDEX)req.index, req.value);
        }

        return true;
    }
    bool DRHWInterface::get_digital_output_cb(dsr_msgs::GetCtrlBoxDigitalOutput::Request& req, dsr_msgs::GetCtrlBoxDigitalOutput::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_digital_output_cb() called and calling Drfl.GetCtrlBoxDigitalOutput");
        res.success = false;

        if((req.index < DR_DIO_MIN_INDEX) || (req.index > DR_DIO_MAX_INDEX)){
            ROS_ERROR("get_digital_output(index=%d): index(%d) is out of range. (normal range: %d ~ %d)",req.index ,req.index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
        }       
        else{
            req.index -=1;
            res.value = Drfl.get_digital_output((GPIO_CTRLBOX_DIGITAL_INDEX)req.index);
            res.success = true;
        }

        return true;
    }
    bool DRHWInterface::get_digital_input_cb(dsr_msgs::GetCtrlBoxDigitalInput::Request& req, dsr_msgs::GetCtrlBoxDigitalInput::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_digital_input_cb() called and calling Drfl.GetCtrlBoxDigitalInput");
        res.success = false;

        if((req.index < DR_DIO_MIN_INDEX) || (req.index > DR_DIO_MAX_INDEX)){
            ROS_ERROR("get_digital_input(index=%d): index(%d) is out of range. [normal range: %d ~ %d]",req.index ,req.index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
        }       
        else{
            req.index -=1;
            res.value = Drfl.get_digital_input((GPIO_CTRLBOX_DIGITAL_INDEX)req.index);
            res.success = true;
        }

        return true;
    }
    bool DRHWInterface::set_tool_digital_output_cb(dsr_msgs::SetToolDigitalOutput::Request& req, dsr_msgs::SetToolDigitalOutput::Response& res)
    {
        //ROS_INFO("DRHWInterface::set_tool_digital_output_cb() called and calling Drfl.SetToolDigitalOutput");
        res.success = false;

        if((req.index < DR_TDIO_MIN_INDEX) || (req.index > DR_TDIO_MAX_INDEX)){
            ROS_ERROR("set_tool_digital_output(index=%d, value=%d): index(%d) is out of range. [normal range: %d ~ %d]",req.index ,req.value ,req.index, DR_TDIO_MIN_INDEX, DR_TDIO_MAX_INDEX);
        }       
        else if((req.value < 0) || (req.value > 1)){
            ROS_ERROR("set_tool_digital_output(index=%d, value=%d): value(%d) is out of range. [normal range: 0 or 1]",req.index ,req.value ,req.value);
        }
        else{       
            req.index -=1;
            res.success = Drfl.set_tool_digital_output((GPIO_TOOL_DIGITAL_INDEX)req.index, req.value);
        }

        return true;
    }
    bool DRHWInterface::get_tool_digital_output_cb(dsr_msgs::GetToolDigitalOutput::Request& req, dsr_msgs::GetToolDigitalOutput::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_tool_digital_output_cb() called and calling Drfl.GetToolDigitalOutput");
        res.success = false;

        if((req.index < DR_TDIO_MIN_INDEX) || (req.index > DR_TDIO_MAX_INDEX)){
            ROS_ERROR("get_tool_digital_output(index=%d): index(%d) is out of range. [normal range: %d ~ %d]",req.index ,req.index, DR_TDIO_MIN_INDEX, DR_TDIO_MAX_INDEX);
        }       
        else{       
            req.index -=1;
            res.value = Drfl.get_tool_digital_output((GPIO_TOOL_DIGITAL_INDEX)req.index);
            res.success = true;
        }

        return true;
    }
    bool DRHWInterface::get_tool_digital_input_cb(dsr_msgs::GetToolDigitalInput::Request& req, dsr_msgs::GetToolDigitalInput::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_tool_digital_input_cb() called and calling Drfl.GetToolDigitalInput");
        res.success = false;
        if((req.index < DR_TDIO_MIN_INDEX) || (req.index > DR_TDIO_MAX_INDEX)){
            ROS_ERROR("get_tool_digital_input(index=%d): index(%d) is out of range. [normal range: %d ~ %d]",req.index ,req.index, DR_TDIO_MIN_INDEX, DR_TDIO_MAX_INDEX);
        }       
        else{
            req.index -=1;
            res.value = Drfl.get_tool_digital_input((GPIO_TOOL_DIGITAL_INDEX)req.index);
            res.success = true;
        }

        return true;
    }
    bool DRHWInterface::set_analog_output_cb(dsr_msgs::SetCtrlBoxAnalogOutput::Request& req, dsr_msgs::SetCtrlBoxAnalogOutput::Response& res)
    {        
        //ROS_INFO("DRHWInterface::set_analog_output_cb() called and calling Drfl.SetCtrlBoxAnalogOutput");
        res.success = false;
        bool bIsError = 0;   

        if((req.channel < 1) || (req.channel > 2)){
            ROS_ERROR("set_analog_output(channel=%d, value=%f): channel(%d) is out of range. [normal range: 1 or 2]",req.channel ,req.value, req.channel);
            bIsError = 1;
        }       
        else
        {
            if(req.channel == 1){                
                if(g_nAnalogOutputModeCh1==DR_ANALOG_CURRENT){
                    if((req.value < 4.0) || (req.value > 20.0)){
                        ROS_ERROR("set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 4.0 ~ 20.0]",req.channel ,req.value ,req.value);
                        bIsError = 1;
                    }
                }
                else if(g_nAnalogOutputModeCh1==DR_ANALOG_VOLTAGE){
                    if((req.value < 0.0) || (req.value > 10.0)){
                        ROS_ERROR("set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 0.0 ~ 10.0]",req.channel ,req.value ,req.value);
                        bIsError = 1;
                    }
                }         
                else{
                    ROS_ERROR("set_analog_output(channel=%d, value=%f): Analog output mode(ch%d) is not set",req.channel ,req.value, req.channel);
                    bIsError = 1;
                }    
            }
            if(req.channel == 2){                
                if(g_nAnalogOutputModeCh2==DR_ANALOG_CURRENT){
                    if((req.value < 4.0) || (req.value > 20.0)){
                        ROS_ERROR("set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 4.0 ~ 20.0]",req.channel ,req.value ,req.value);
                        bIsError = 1;
                    }
                }
                else if(g_nAnalogOutputModeCh2==DR_ANALOG_VOLTAGE){
                    if((req.value < 0.0) || (req.value > 10.0)){
                        ROS_ERROR("set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 0.0 ~ 10.0]",req.channel ,req.value ,req.value);
                        bIsError = 1;
                    }
                }         
                else{
                    ROS_ERROR("set_analog_output(channel=%d, value=%f): Analog output mode(ch%d) is not set",req.channel ,req.value, req.channel);
                    bIsError = 1;
                }    
            }
        }
        if(!bIsError)
        {
            req.channel -=1;
            res.success = Drfl.set_analog_output((GPIO_CTRLBOX_ANALOG_INDEX)req.channel, req.value);
        }

        return true;
    }
    bool DRHWInterface::get_analog_input_cb(dsr_msgs::GetCtrlBoxAnalogInput::Request& req, dsr_msgs::GetCtrlBoxAnalogInput::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_analog_input_cb() called and calling Drfl.GetCtrlBoxAnalogInput");
        res.success = false;
        bool bIsError = 0;   

        if((req.channel < 1) || (req.channel > 2)){
            ROS_ERROR("get_analog_input(channel=%d): channel(%d) is out of range. [normal range: 1 or 2]",req.channel ,req.channel);
            bIsError = 1;
        }       
        else{
            if(req.channel == 1){
                if(g_nAnalogOutputModeCh1 == -1){
                    ROS_ERROR("get_analog_input(channel=%d): Analog output mode(ch%d) is not set",req.channel ,req.channel);
                    bIsError = 1;
                }                                    
            }
            if(req.channel == 2){
                if(g_nAnalogOutputModeCh2 == -1){
                    ROS_ERROR("get_analog_input(channel=%d): Analog output mode(ch%d) is not set",req.channel ,req.channel);
                    bIsError = 1;
                }                                    
            }
        }

        if(!bIsError){
            req.channel -=1;
            res.value = Drfl.get_analog_input((GPIO_CTRLBOX_ANALOG_INDEX)req.channel);
            res.success = true;
        }

        return true;
    }
    bool DRHWInterface::set_analog_output_type_cb(dsr_msgs::SetCtrlBoxAnalogOutputType::Request& req, dsr_msgs::SetCtrlBoxAnalogOutputType::Response& res)
    {
        //ROS_INFO("DRHWInterface::set_analog_output_type_cb() called and calling Drfl.SetCtrlBoxAnalogOutputType");
        res.success = false;

        if((req.channel < 1) || (req.channel > 2)){
            ROS_ERROR("set_analog_output_type(channel=%d, mode=%d): channel(%d) is out of range. [normal range: 1 or 2]",req.channel ,req.mode, req.channel);
        }       
        else if((req.mode < 0) || (req.mode > 1)){
            ROS_ERROR("set_analog_output_type(channel=%d, mode=%d): mode(%d) is out of range. [normal range: 0 or 1]",req.channel ,req.mode, req.mode);
        }       
        else{
            if(req.channel == 1) g_nAnalogOutputModeCh1 = req.mode;    
            if(req.channel == 2) g_nAnalogOutputModeCh2 = req.mode;    
                    
            req.channel -=1;
            res.success = Drfl.set_mode_analog_output((GPIO_CTRLBOX_ANALOG_INDEX)req.channel, (GPIO_ANALOG_TYPE)req.mode);
        }

        return true;
    }
    bool DRHWInterface::set_analog_input_type_cb(dsr_msgs::SetCtrlBoxAnalogInputType::Request& req, dsr_msgs::SetCtrlBoxAnalogInputType::Response& res)
    {
        //ROS_INFO("DRHWInterface::set_analog_input_type_cb() called and calling Drfl.SetCtrlBoxAnalogInputType");
        res.success = false;

        if((req.channel < 1) || (req.channel > 2)){
            ROS_ERROR("set_analog_input_type(channel=%d, mode=%d): channel(%d) is out of range. [normal range: 1 or 2]",req.channel ,req.mode, req.channel);
        }       
        else if((req.mode < 0) || (req.mode > 1)){
            ROS_ERROR("set_analog_input_type(channel=%d, mode=%d): mode(%d) is out of range. [normal range: 0 or 1]",req.channel ,req.mode, req.mode);
        }       
        else{
            if(req.channel == 1) g_nAnalogOutputModeCh1 = req.mode;    
            if(req.channel == 2) g_nAnalogOutputModeCh2 = req.mode;    

            req.channel -=1;
            res.success = Drfl.set_mode_analog_input((GPIO_CTRLBOX_ANALOG_INDEX)req.channel, (GPIO_ANALOG_TYPE)req.mode);
        }

        return true;
    }
    bool DRHWInterface::set_modbus_output_cb(dsr_msgs::SetModbusOutput::Request& req, dsr_msgs::SetModbusOutput::Response& res)
    {
        //ROS_INFO("DRHWInterface::set_modbus_output_cb() called and calling Drfl.SetModbusOutput");
        res.success = false;
        res.success = Drfl.set_modbus_output(req.name, (unsigned short)req.value);
        return true;
    }
    bool DRHWInterface::get_modbus_input_cb(dsr_msgs::GetModbusInput::Request& req, dsr_msgs::GetModbusInput::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_modbus_input_cb() called and calling Drfl.GetModbusInput");
        res.value = Drfl.get_modbus_input(req.name);
        return true;
    }
    bool DRHWInterface::config_create_modbus_cb(dsr_msgs::ConfigCreateModbus::Request& req, dsr_msgs::ConfigCreateModbus::Response& res)
    {
        //ROS_INFO("DRHWInterface::config_create_modbus_cb() called and calling Drfl.ConfigCreateModbus");
        res.success = false;
        res.success = Drfl.add_modbus_signal(req.name, req.ip, (unsigned short)req.port, (MODBUS_REGISTER_TYPE)req.reg_type, (unsigned short)req.index, (unsigned short)req.value, (int)req.slave_id);
        return true;
    }
    bool DRHWInterface::config_delete_modbus_cb(dsr_msgs::ConfigDeleteModbus::Request& req, dsr_msgs::ConfigDeleteModbus::Response& res)
    {
        //ROS_INFO("DRHWInterface::config_delete_modbus_cb() called and calling Drfl.ConfigDeleteModbus");
        res.success = false;
        res.success = Drfl.del_modbus_signal(req.name);
        return true;
    }
    bool DRHWInterface::drl_pause_cb(dsr_msgs::DrlPause::Request& req, dsr_msgs::DrlPause::Response& res)
    {
        //ROS_INFO("DRHWInterface::drl_pause_cb() called and calling Drfl.DrlPause");
        res.success = false;

        if(m_bIsEmulatorMode)
            ROS_ERROR("The drl service cannot be used in emulator mode (available in real mode).");
        else 
            res.success = Drfl.drl_pause();

        return true;
    }
    bool DRHWInterface::drl_start_cb(dsr_msgs::DrlStart::Request& req, dsr_msgs::DrlStart::Response& res)
    {
        //ROS_INFO("DRHWInterface::drl_start_cb() called and calling Drfl.DrlStart");
        res.success = false;

        if(m_bIsEmulatorMode)
            ROS_ERROR("The drl service cannot be used in emulator mode (available in real mode).");
        else 
            res.success = Drfl.drl_start((ROBOT_SYSTEM)req.robotSystem, req.code);

        return true;
    }
    bool DRHWInterface::drl_stop_cb(dsr_msgs::DrlStop::Request& req, dsr_msgs::DrlStop::Response& res)
    {
        //ROS_INFO("DRHWInterface::drl_stop_cb() called and calling Drfl.DrlStop");
        res.success = false;

        if(m_bIsEmulatorMode)
            ROS_ERROR("The drl service cannot be used in emulator mode (available in real mode).");
        else 
            res.success = Drfl.drl_stop((STOP_TYPE)req.stop_mode);

        return true;
    }
    bool DRHWInterface::drl_resume_cb(dsr_msgs::DrlResume::Request& req, dsr_msgs::DrlResume::Response& res)
    {
        //ROS_INFO("DRHWInterface::drl_resume_cb() called and calling Drfl.DrlResume");
        res.success = false;

        if(m_bIsEmulatorMode)
            ROS_ERROR("The drl service cannot be used in emulator mode (available in real mode).");
        else 
            res.success = Drfl.drl_resume();

        return true;
    }
    bool DRHWInterface::get_drl_state_cb(dsr_msgs::GetDrlState::Request& req, dsr_msgs::GetDrlState::Response& res)
    {
        res.success = false;

        if(m_bIsEmulatorMode)
            ROS_ERROR("The drl service cannot be used in emulator mode (available in real mode).");
        else{ 
            res.drl_state = Drfl.get_program_state();
            res.success = true;
        }    

        return true;
    }
    bool DRHWInterface::set_current_tcp_cb(dsr_msgs::SetCurrentTcp::Request& req, dsr_msgs::SetCurrentTcp::Response& res)
    {
        //ROS_INFO("DRHWInterface::set_current_tcp_cb() called and calling Drfl.SetCurrentTCP");
        res.success = false;
        res.success = Drfl.set_tcp(req.name);
        return true;
    }
    bool DRHWInterface::get_current_tcp_cb(dsr_msgs::GetCurrentTcp::Request& req, dsr_msgs::GetCurrentTcp::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_current_tcp_cb() called and calling Drfl.GetCurrentTCP");
        res.success = false;
        res.info = Drfl.get_tcp();
        res.success = true;
        return true;
    }
    bool DRHWInterface::config_create_tcp_cb(dsr_msgs::ConfigCreateTcp::Request& req, dsr_msgs::ConfigCreateTcp::Response& res)
    {
        //ROS_INFO("DRHWInterface::config_create_tcp_cb() called and calling Drfl.ConfigCreateTCP");
        res.success = false;
        std::array<float, 6> target_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        res.success = Drfl.add_tcp(req.name, target_pos.data());
        return true;
    }
    bool DRHWInterface::config_delete_tcp_cb(dsr_msgs::ConfigDeleteTcp::Request& req, dsr_msgs::ConfigDeleteTcp::Response& res)
    {
        //ROS_INFO("DRHWInterface::config_delete_tcp_cb() called and calling Drfl.ConfigDeleteTCP");
        res.success = false;
        res.success = Drfl.del_tcp(req.name);
        return true;
    }

    bool DRHWInterface::set_current_tool_cb(dsr_msgs::SetCurrentTool::Request& req, dsr_msgs::SetCurrentTool::Response& res)
    {
        //ROS_INFO("DRHWInterface::set_current_tool_cb() called and calling Drfl.SetCurrentTool");
        res.success = false;
        res.success = Drfl.set_tool(req.name);
        return true;
    }
    bool DRHWInterface::get_current_tool_cb(dsr_msgs::GetCurrentTool::Request& req, dsr_msgs::GetCurrentTool::Response& res)
    {
        //ROS_INFO("DRHWInterface::get_current_tool_cb() called and calling Drfl.GetCurrentTool %s", Drfl.GetCurrentTool().c_str());
        res.success = false;
        res.info = Drfl.get_tool();
        res.success = true;
        return true;
    }
    bool DRHWInterface::config_create_tool_cb(dsr_msgs::ConfigCreateTool::Request& req, dsr_msgs::ConfigCreateTool::Response& res)
    {
        //ROS_INFO("DRHWInterface::config_create_tool_cb() called and calling Drfl.ConfigCreateTool");
        res.success = false;
        std::array<float, 3> target_cog;
        std::array<float, 6> target_inertia;
        std::copy(req.cog.cbegin(), req.cog.cend(), target_cog.begin());
        std::copy(req.inertia.cbegin(), req.inertia.cend(), target_inertia.begin());
        res.success = Drfl.add_tool(req.name, req.weight, target_cog.data(), target_inertia.data());
        return true;
    }
    bool DRHWInterface::config_delete_tool_cb(dsr_msgs::ConfigDeleteTool::Request& req, dsr_msgs::ConfigDeleteTool::Response& res)
    {
        //ROS_INFO("DRHWInterface::config_delete_tool_cb() called and calling Drfl.ConfigDeleteTool");
        res.success = false;
        res.success = Drfl.del_tool(req.name);
        return true;
    }
    bool DRHWInterface::set_tool_shape_cb(dsr_msgs::SetToolShape::Request& req, dsr_msgs::SetToolShape::Response& res)
    {
        res.success = false;
        res.success = Drfl.set_tool(req.name);
        return true;
    }

    //Gripper Service

    bool DRHWInterface::robotiq_2f_move_cb(dsr_msgs::Robotiq2FMove::Request& req, dsr_msgs::Robotiq2FMove::Response& res)
    {
        res.success = false;
        //ROS_INFO("DRHWInterface::gripper_move_cb() called and calling Nothing");
        /*
        if(mode == "robotiq_2f"){
            //Serial Communication
            ser.Activation();
            ros::Duration(0.1).sleep();
            ser.Close();
            ros::Duration(0.1).sleep();
            ser.Open();
        }
        */
        float goal_pos = req.width;
        
        while(abs(goal_pos - joints[6].pos) > 0.01){
            if(goal_pos > joints[6].pos){    
                joints[6].pos = joints[6].pos + 0.01;
            }
            else if(joints[6].pos > goal_pos){
                joints[6].pos = joints[6].pos - 0.01;
            }
            ros::Duration(0.01).sleep();
        }
        res.success = true;
        return true;
    }
    bool DRHWInterface::robotiq_2f_open_cb(dsr_msgs::Robotiq2FOpen::Request& req, dsr_msgs::Robotiq2FOpen::Response& res)
    {
        res.success = false;
        float goal_pos = 0.8;
        while(abs(goal_pos - joints[6].pos) > 0.01){
            if(goal_pos > joints[6].pos){    
                joints[6].pos = joints[6].pos + 0.01;
            }
            else if(joints[6].pos > goal_pos){
                joints[6].pos = joints[6].pos - 0.01;
            }
            ros::Duration(0.01).sleep();
        }
        res.success = true;
        return true;      
    }
    bool DRHWInterface::robotiq_2f_close_cb(dsr_msgs::Robotiq2FClose::Request& req, dsr_msgs::Robotiq2FClose::Response& res)
    {
        res.success = false;
        float goal_pos = 0.0;

        while(abs(goal_pos - joints[6].pos) > 0.01){
            if(goal_pos > joints[6].pos){    
                joints[6].pos = joints[6].pos + 0.01;
            }
            else if(joints[6].pos > goal_pos){
                joints[6].pos = joints[6].pos - 0.01;
            }
            ros::Duration(0.01).sleep();
        }
        res.success = true;
        return true;
    }

    bool DRHWInterface::serial_send_data_cb(dsr_msgs::SerialSendData::Request& req, dsr_msgs::SerialSendData::Response &res)
    {
        res.success = false;
        std_msgs::String send_data;
        send_data.data = req.data;
        m_PubSerialWrite.publish(send_data);
        res.success = true;
        return true;
    }

    //////////////////////////// Real Time Operation //////////////////////////////////////////

    bool DRHWInterface::connect_rt_control_cb(dsr_msgs::ConnectRTControl::Request& req, dsr_msgs::ConnectRTControl::Response& res)
    {
        res.success = false;
        res.success = Drfl.connect_rt_control(req.ip_address, req.port);
        return true;
    }
    bool DRHWInterface::disconnect_rt_control_cb(dsr_msgs::DisconnectRTControl::Request& req, dsr_msgs::DisconnectRTControl::Response& res)
    {
        res.success = false;
        res.success = Drfl.disconnect_rt_control();
        return true;
    }

    bool DRHWInterface::get_rt_control_output_version_list_cb(dsr_msgs::GetRTControlOutputVersionList::Request& req, dsr_msgs::GetRTControlOutputVersionList::Response& res)
    {
        res.success = false;
        res.version = Drfl.get_rt_control_output_version_list();
        res.success = true;
        return true;
    }

    bool DRHWInterface::get_rt_control_input_version_list_cb(dsr_msgs::GetRTControlInputVersionList::Request& req, dsr_msgs::GetRTControlInputVersionList::Response& res)
    {
        res.success = false;
        res.version = Drfl.get_rt_control_input_version_list();
        res.success = true;
        return true;
    }

    bool DRHWInterface::get_rt_control_input_data_list_cb(dsr_msgs::GetRTControlInputDataList::Request& req, dsr_msgs::GetRTControlInputDataList::Response& res)
    {
        res.success = false;
        res.data = Drfl.get_rt_control_input_data_list(req.version);
        res.success = true;
        return true;
    }

    bool DRHWInterface::get_rt_control_output_data_list_cb(dsr_msgs::GetRTControlOutputDataList::Request& req, dsr_msgs::GetRTControlOutputDataList::Response& res)
    {
        res.success = false;
        res.data = Drfl.get_rt_control_output_data_list(req.version);
        res.success = true;
        return true;
    }

    bool DRHWInterface::set_rt_control_input_cb(dsr_msgs::SetRTControlInput::Request& req, dsr_msgs::SetRTControlInput::Response& res)
    {
        res.success = false;
        res.success = Drfl.set_rt_control_input(req.version, req.period, req.loss);
        return true;
    }

    bool DRHWInterface::set_rt_control_output_cb(dsr_msgs::SetRTControlOutput::Request& req, dsr_msgs::SetRTControlOutput::Response& res)
    {
        res.success = false;
        res.success = Drfl.set_rt_control_output(req.version, req.period, req.loss);
        return true;
    }
    
    bool DRHWInterface::start_rt_control_cb(dsr_msgs::StartRTControl::Request& req, dsr_msgs::StartRTControl::Response& res)
    {
        res.success = false;
        res.success = Drfl.start_rt_control();
        return true;
    }

    bool DRHWInterface::stop_rt_control_cb(dsr_msgs::StopRTControl::Request& req, dsr_msgs::StopRTControl::Response& res)
    {
        res.success = false;
        res.success = Drfl.stop_rt_control();
        return true;
    }
    
    bool DRHWInterface::set_velj_rt_cb(dsr_msgs::SetVelJRT::Request& req, dsr_msgs::SetVelJRT::Response& res)
    {
        res.success = false;
        std::array<float, 6> vel;
        std::copy(req.vel.cbegin(), req.vel.cend(), vel.begin());
        res.success = Drfl.set_velj_rt(vel.data());
        return true;
    }

    bool DRHWInterface::set_accj_rt_cb(dsr_msgs::SetAccJRT::Request& req, dsr_msgs::SetAccJRT::Response& res)
    {
        res.success = false;
        std::array<float, 6> acc;
        std::copy(req.acc.cbegin(), req.acc.cend(), acc.begin());
        res.success = Drfl.set_accj_rt(acc.data());
        return true;
    }

    bool DRHWInterface::set_velx_rt_cb(dsr_msgs::SetVelXRT::Request& req, dsr_msgs::SetVelXRT::Response& res)
    {
        res.success = false;
        res.success = Drfl.set_velx_rt(req.trans, req.rotation);
        return true;
    }

    bool DRHWInterface::set_accx_rt_cb(dsr_msgs::SetAccXRT::Request& req, dsr_msgs::SetAccXRT::Response& res)
    {
        res.success = false;
        res.success = Drfl.set_accx_rt(req.trans, req.rotation);
        return true;
    }

    bool DRHWInterface::read_data_rt_cb(dsr_msgs::ReadDataRT::Request& req, dsr_msgs::ReadDataRT::Response& res)
    {
        LPRT_OUTPUT_DATA_LIST temp = Drfl.read_data_rt();
        res.data.time_stamp = temp->time_stamp;
        for(int i=0; i<6; i++){
            res.data.actual_joint_position[i] = temp->actual_joint_position[i];
            res.data.actual_joint_position_abs[i] = temp->actual_joint_position_abs[i];
            res.data.actual_joint_velocity[i] = temp->actual_joint_velocity[i];
            res.data.actual_joint_velocity_abs[i] = temp->actual_joint_velocity_abs[i];
            res.data.actual_tcp_position[i] = temp->actual_tcp_position[i];
            res.data.actual_tcp_velocity[i] = temp->actual_tcp_velocity[i];
            res.data.actual_flange_position[i] = temp->actual_flange_position[i];
            res.data.actual_flange_velocity[i] = temp->actual_flange_velocity[i];
            res.data.actual_motor_torque[i] = temp->actual_motor_torque[i];
            res.data.actual_joint_torque[i] = temp->actual_joint_torque[i];
            res.data.raw_joint_torque[i] = temp->raw_joint_torque[i];
            res.data.raw_force_torque[i] = temp->raw_force_torque[i];
            res.data.external_joint_torque[i] = temp->external_joint_torque[i];
            res.data.external_tcp_force[i] = temp->external_tcp_force[i];
            res.data.target_joint_position[i] = temp->target_joint_position[i];
            res.data.target_joint_velocity[i] = temp->target_joint_velocity[i];
            res.data.target_joint_acceleration[i] = temp->target_joint_acceleration[i];
            res.data.target_motor_torque[i] = temp->target_motor_torque[i];
            res.data.target_tcp_position[i] = temp->target_tcp_position[i];
            res.data.target_tcp_velocity[i] = temp->target_tcp_velocity[i];
            res.data.gravity_torque[i] = temp->gravity_torque[i];
            res.data.joint_temperature[i] = temp->joint_temperature[i];
            res.data.goal_joint_position[i] = temp->goal_joint_position[i];
            res.data.goal_tcp_position[i] = temp->goal_tcp_position[i];
            res.data.goal_joint_position[i] = temp->goal_joint_position[i];
            res.data.goal_tcp_position[i] = temp->goal_tcp_position[i];
        }

        std_msgs::Float64MultiArray arr;
        for(int i=0; i<6; i++){
            arr.data.clear();
            for(int j=0; j<6; j++){
                arr.data.push_back(temp->coriolis_matrix[i][j]);
            }
            res.data.coriolis_matrix.push_back(arr);
        }

        std_msgs::Float64MultiArray arr1;
        for(int i=0; i<6; i++){
            arr1.data.clear();
            for(int j=0; j<6; j++){
                arr1.data.push_back(temp->mass_matrix[i][j]);
            }
            res.data.mass_matrix.push_back(arr1);
        }

        std_msgs::Float64MultiArray arr2;
        for(int i=0; i<6; i++){
            arr2.data.clear();
            for(int j=0; j<6; j++){
                arr2.data.push_back(temp->jacobian_matrix[i][j]);
            }
            res.data.jacobian_matrix.push_back(arr2);
        }


        res.data.solution_space = temp->solution_space;
        res.data.singularity = temp->singularity;
        res.data.operation_speed_rate = temp->operation_speed_rate;
        res.data.controller_digital_input = temp->controller_digital_input;
        res.data.controller_digital_output = temp->controller_digital_output;

        for(int i=0; i<2; i++){
            res.data.controller_analog_input_type[i] = temp->controller_analog_input_type[i];
            res.data.controller_analog_input[i] = temp->controller_analog_input[i];
            res.data.controller_analog_output_type[i] = temp->controller_analog_output_type[i];
            res.data.controller_analog_output[i] = temp->controller_analog_output[i];
            res.data.external_encoder_strobe_count[i] = temp->external_encoder_strobe_count[i];
            res.data.external_encoder_count[i] = temp->external_encoder_count[i];
        }

        res.data.flange_digital_input = temp->flange_digital_input;
        res.data.flange_digital_output = temp->flange_digital_output;

        for(int i=0; i<4; i++){
            res.data.flange_analog_input[i] = temp->flange_analog_input[i];
        }
        res.data.robot_mode = temp->robot_mode;
        res.data.robot_state = temp->robot_state;
        res.data.control_mode = temp->control_mode;
        return true;
    }

    bool DRHWInterface::write_data_rt_cb(dsr_msgs::WriteDataRT::Request& req, dsr_msgs::WriteDataRT::Response& res)
    {
        res.success = false;
        std::array<float, 6> external_force_torque;
        std::array<float, 6> external_analog_output;
        std::array<float, 6> external_analog_input;

        std::copy(req.external_force_torque.cbegin(), req.external_force_torque.cend(), external_force_torque.begin());
        std::copy(req.external_analog_output.cbegin(), req.external_analog_output.cend(), external_analog_output.begin());
        std::copy(req.external_analog_input.cbegin(), req.external_analog_input.cend(), external_analog_input.begin());

        res.success = Drfl.write_data_rt(external_force_torque.data(), req.external_digital_input, req.external_digital_output, external_analog_input.data(), external_analog_output.data());
        return true;
    } 

}