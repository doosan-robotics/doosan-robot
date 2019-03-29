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

CDRFL Drfl;
Serial_comm ser_comm;

bool g_bHasControlAuthority = FALSE;
bool g_bTpInitailizingComplted = FALSE;
bool g_bHommingCompleted = FALSE;

DR_STATE    g_stDrState;
DR_ERROR    g_stDrError;

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
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

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

    void DRHWInterface::OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO)
    {
        int nIoCtlBox = 0x0;
        for (int i = 0; i < NUM_DIGITAL; i++)
        {
            cout << "[callback OnMonitoringCtrlIOCB] DI#"<< i << ": " << pCtrlIO->_tInput._iActualDI[i] << endl;
            nIoCtlBox += (pCtrlIO->_tInput._iActualDI[i] << i);
        }

        g_stDrState.nIoControlBox = nIoCtlBox;
    }

    void DRHWInterface::OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus)
    {
        for (int i = 0; i < pModbus->_iRegCount; i++)
            cout << "[callback OnMonitoringModbusCB] " << pModbus->_tRegister[i]._szSymbol <<": " << pModbus->_tRegister[i]._iValue<< endl;
    }

    void DRHWInterface::OnMonitoringDataCB(const LPMONITORING_DATA pData)
    {
        // This function is called every 100 msec
        // Only work within 50msec
        //ROS_INFO("DRHWInterface::OnMonitoringDataCB");


        /* 추후 연결 필요
        pData->_tCtrl._tState._iActualMode;             // position control: 0, torque control: 1
        pData->_tCtrl._tState._iActualSpace;            //joint space: 0, task space: 1

        pData->_tCtrl._tJoint._fActualPos[NUM_JOINT];   // Position Actual Value in INC 
        pData->_tCtrl._tJoint._fActualAbs[NUM_JOINT];   // Position Actual Value in ABS
        pData->_tCtrl._tJoint._fActualVel[NUM_JOINT];   // Velocity Actual Value
        pData->_tCtrl._tJoint._fActualErr[NUM_JOINT];   // Joint Error
        pData->_tCtrl._tJoint._fTargetPos[NUM_JOINT];   // Target Position
        pData->_tCtrl._tJoint._fTargetVel[NUM_JOINT];   // Target Velocity

        pData->_tCtrl._tTool._fActualPos[2][NUM_TASK];  // Position Actual Value(0: tool, 1: flange)
        pData->_tCtrl._tTool._fActualVel[NUM_TASK];     // Velocity Actual Value
        pData->_tCtrl._tTool._fActualErr[NUM_TASK];     // Task Error
        pData->_tCtrl._tTool._fTargetPos[NUM_TASK];     // Target Position
        pData->_tCtrl._tTool._fTargetVel[NUM_TASK];     // Target Velocity
        pData->_tCtrl._tTool._iSolutionSpace;           // Solution Space
        pData->_tCtrl._tTool._fRotationMatrix[3][3];    // Rotation Matrix

        pData->_tCtrl._tTorque._fDynamicTor[NUM_JOINT]; // Dynamics Torque
        pData->_tCtrl._tTorque._fActualJTS[NUM_JOINT];  // Joint Torque Sensor Value
        pData->_tCtrl._tTorque._fActualEJT[NUM_JOINT];  // External Joint Torque
        pData->_tCtrl._tTorque._fActualETT[NUM_JOINT];  // External Task Force/Torque

        pData->_tMisc._dSyncTime;                       // inner clock counter
        pData->_tMisc._iActualDI[NUM_FLANGE_IO];        // Digtal Input data    
        pData->_tMisc._iActualDO[NUM_FLANGE_IO];        // Digtal output data
        pData->_tMisc._iActualBK[NUM_JOINT];            // brake state
        pData->_tMisc._iActualBT[NUM_BUTTON];           // robot button state
        pData->_tMisc._fActualMC[NUM_JOINT];            // motor input current
        pData->_tMisc._fActualMT[NUM_JOINT];            // motro current temperature
        */

        for (int i = 0; i < NUM_JOINT; i++){
            if(pData){  
               g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];    
               ///g_stDrState.fCurrentPosx[i] = pData->_tCtrl._tWorld._fActualPos[i];    
            }
        }
    }

    void DRHWInterface::OnMonitoringStateCB(const ROBOT_STATE eState)
    {
        //This function is called when the state changes.
        //ROS_INFO("DRHWInterface::OnMonitoringStateCB");    
        // Only work within 50msec

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
                Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
                Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
            }
            break;
        case STATE_SAFE_OFF:
            if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_SERVO_ON);
            break;
        case STATE_SAFE_STOP2:
            if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
            break;
        case STATE_SAFE_OFF2:
            if (g_bHasControlAuthority) {
                Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
            }
            break;
        case STATE_RECOVERY:
            Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
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
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
            //Drfl.TransitControlAuth(MANaGE_ACCESS_CONTROL_RESPONSE_YES);
            break;
        case MONITORING_ACCESS_CONTROL_GRANT:
            cout  << "access control granted" << endl;
            g_bHasControlAuthority = TRUE;
            OnMonitoringStateCB(Drfl.GetRobotState());
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
        case MONITORING_ACCESS_CONTROL_LOSS:
            g_bHasControlAuthority = FALSE;
            if (g_bTpInitailizingComplted) {
                Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
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
        ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
        ros::Publisher PubRobotError = node->advertise<dsr_msgs::RobotError>("/doosan_robot/error",100);
        dsr_msgs::RobotError msg;

        ROS_ERROR("[callback OnLogAlarm]");
        ROS_ERROR("  level : %d",(unsigned int)pLogAlarm->_iLevel );
        ROS_ERROR("  group : %d",(unsigned int)pLogAlarm->_iGroup );
        ROS_ERROR("  index : %d", pLogAlarm->_iIndex );
        ROS_ERROR("  param : %s", pLogAlarm->_szParam[0] );
        ROS_ERROR("  param : %s", pLogAlarm->_szParam[1] );
        ROS_ERROR("  param : %s", pLogAlarm->_szParam[2] );

        g_stDrError.nLevel = (unsigned int)pLogAlarm->_iLevel;
        g_stDrError.nGroup = (unsigned int)pLogAlarm->_iGroup;
        g_stDrError.nCode  = pLogAlarm->_iIndex;
        strncpy(g_stDrError.strMsg1, pLogAlarm->_szParam[0], MAX_STRING_SIZE); 
        strncpy(g_stDrError.strMsg2, pLogAlarm->_szParam[1], MAX_STRING_SIZE); 
        strncpy(g_stDrError.strMsg3, pLogAlarm->_szParam[2], MAX_STRING_SIZE); 

        msg.level = g_stDrError.nLevel;
        msg.group = g_stDrError.nGroup;
        msg.code  = g_stDrError.nCode;
        msg.msg1  = g_stDrError.strMsg1;  
        msg.msg2  = g_stDrError.strMsg2;
        msg.msg3  = g_stDrError.strMsg3;

        PubRobotError.publish(msg);
    }
    //----- register the call-back functions end -------------------------------------
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    void MsgScriber(const dsr_msgs::RobotStop::ConstPtr& msg)
    {
        ROS_INFO("receive msg.stop_mode = %d", msg->stop_mode);
        ROS_INFO("receive msg.stop_mode = %d", msg->stop_mode);
        ROS_INFO("receive msg.stop_mode = %d", msg->stop_mode);
    
        Drfl.MoveStop((STOP_TYPE)msg->stop_mode);
    } 

    int DRHWInterface::MsgPublisher_RobotState()
    {
        dsr_msgs::RobotState msg;
        memcpy(&m_stDrState, &g_stDrState, sizeof(DR_STATE));
         
        msg.robot_state         = m_stDrState.nRobotState;
        msg.robot_state_str     = m_stDrState.strRobotState;
        ///printf("[%s,%s] msg.robot_state_str =%s m_stDrState.strRobotState=%s\n",m_strRobotName.c_str(),m_strRobotModel.c_str(),msg.robot_state_str.c_str(),m_stDrState.strRobotState);

        for (int i = 0; i < NUM_JOINT; i++)
        {
            msg.current_posj[i]    = m_stDrState.fCurrentPosj[i];
            ///msg.current_posx[i]    = m_stDrState.fCurrentPosx[i];
        }
        msg.io_control_box      = m_stDrState.nIoControlBox;
        //msg.io_modbus;    GJH
        //msg.error;        GJH
        msg.access_control      = m_stDrState.nAccessControl;
        msg.homming_completed   = m_stDrState.bHommingCompleted;
        msg.tp_initialized      = m_stDrState.bTpInitialized; 
        msg.speed               = m_stDrState.nSpeed;
        msg.mastering_need      = m_stDrState.bMasteringNeed;
        msg.drl_stopped         = m_stDrState.bDrlStopped;
        msg.disconnected        = m_stDrState.bDisconnected;

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
        // gazebo에 joint position 전달
        m_PubtoGazebo = private_nh_.advertise<std_msgs::Float64MultiArray>("/dsr_joint_position_controller/command",10);
        // moveit의 trajectory/goal를 받아 제어기로 전달
        m_sub_joint_trajectory = private_nh_.subscribe("dsr_joint_trajectory_controller/follow_joint_trajectory/goal", 10, &DRHWInterface::trajectoryCallback, this);
        // topic echo 명령으로 제어기에 전달
        m_sub_joint_position = private_nh_.subscribe("dsr_joint_position_controller/command", 10, &DRHWInterface::positionCallback, this);
        
        ros::NodeHandle nh_temp;
        m_SubSerialRead = nh_temp.subscribe("serial_read", 100, &Serial_comm::read_callback, &ser_comm);
        m_PubSerialWrite = nh_temp.advertise<std_msgs::String>("serial_write", 100);
        

        //  motion Operations
        m_nh_move_service[0] = private_nh_.advertiseService("motion/move_joint", &DRHWInterface::movej_cb, this);
        m_nh_move_service[1] = private_nh_.advertiseService("motion/move_line", &DRHWInterface::movel_cb, this);
        m_nh_move_service[2] = private_nh_.advertiseService("motion/move_jointx", &DRHWInterface::movejx_cb, this);
        m_nh_move_service[3] = private_nh_.advertiseService("motion/move_circle", &DRHWInterface::movec_cb, this);
        m_nh_move_service[4] = private_nh_.advertiseService("motion/move_spline_joint", &DRHWInterface::movesj_cb, this);
        m_nh_move_service[5] = private_nh_.advertiseService("motion/move_spline_task", &DRHWInterface::movesx_cb, this);
        m_nh_move_service[6] = private_nh_.advertiseService("motion/move_blending", &DRHWInterface::moveb_cb, this);
        m_nh_move_service[7] = private_nh_.advertiseService("motion/move_spiral", &DRHWInterface::movespiral_cb, this);
        m_nh_move_service[8] = private_nh_.advertiseService("motion/move_periodic", &DRHWInterface::moveperiodic_cb, this);
        m_nh_move_service[9] = private_nh_.advertiseService("motion/move_wait", &DRHWInterface::movewait_cb, this);

        //  GPIO Operations
        m_nh_io_service[0] = private_nh_.advertiseService("io/set_digital_output", &DRHWInterface::set_digital_output_cb, this);
        m_nh_io_service[1] = private_nh_.advertiseService("io/get_digital_input", &DRHWInterface::get_digital_input_cb, this);
        m_nh_io_service[2] = private_nh_.advertiseService("io/set_tool_digital_output", &DRHWInterface::set_tool_digital_output_cb, this);
        m_nh_io_service[3] = private_nh_.advertiseService("io/get_tool_digital_input", &DRHWInterface::get_tool_digital_input_cb, this);
        m_nh_io_service[4] = private_nh_.advertiseService("io/set_analog_output", &DRHWInterface::set_analog_output_cb, this);
        m_nh_io_service[5] = private_nh_.advertiseService("io/get_analog_input", &DRHWInterface::get_analog_input_cb, this);
        m_nh_io_service[6] = private_nh_.advertiseService("io/set_analog_output_type", &DRHWInterface::set_analog_output_type_cb, this);
        m_nh_io_service[7] = private_nh_.advertiseService("io/set_analog_input_type", &DRHWInterface::set_analog_input_type_cb, this);

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

        // DRL Operations
        m_nh_drl_service[0] = private_nh_.advertiseService("drl/drl_pause", &DRHWInterface::drl_pause_cb, this);
        m_nh_drl_service[1] = private_nh_.advertiseService("drl/drl_resume", &DRHWInterface::drl_resume_cb, this);
        m_nh_drl_service[2] = private_nh_.advertiseService("drl/drl_start", &DRHWInterface::drl_start_cb, this);
        m_nh_drl_service[3] = private_nh_.advertiseService("drl/drl_stop", &DRHWInterface::drl_stop_cb, this);

        // Gripper Operations
        m_nh_gripper_service[0] = private_nh_.advertiseService("gripper/robotiq_2f_open", &DRHWInterface::robotiq_2f_open_cb, this);
        m_nh_gripper_service[1] = private_nh_.advertiseService("gripper/robotiq_2f_close", &DRHWInterface::robotiq_2f_close_cb, this);
        m_nh_gripper_service[2] = private_nh_.advertiseService("gripper/robotiq_2f_move", &DRHWInterface::robotiq_2f_move_cb, this);

        // Serial Operations
        m_nh_serial_service[0] = private_nh_.advertiseService("gripper/serial_open", &DRHWInterface::serial_open_cb, this);
        m_nh_serial_service[1] = private_nh_.advertiseService("gripper/serial_close", &DRHWInterface::serial_close_cb, this);   
        m_nh_serial_service[2] = private_nh_.advertiseService("gripper/serial_send_data", &DRHWInterface::serial_send_data_cb, this);

        memset(&g_stDrState, 0x00, sizeof(DR_STATE)); 
        memset(&g_stDrError, 0x00, sizeof(DR_ERROR)); 
        memset(&m_stDrState, 0x00, sizeof(DR_STATE));
        memset(&m_stDrError, 0x00, sizeof(DR_ERROR));

        // create threads     
        m_th_subscribe = boost::thread( boost::bind(&thread_subscribe, private_nh_) );
        m_th_publisher = boost::thread( boost::bind(&thread_publisher, this, private_nh_, DSR_CTL_PUB_RATE/*hz*/) );    //100hz(10ms)

    }
    DRHWInterface::~DRHWInterface()
    {
        ROS_INFO("DRHWInterface::~DRHWInterface() 0");
    
        Drfl.CloseConnection();
        ROS_INFO("DRHWInterface::~DRHWInterface() 1");

        m_th_publisher.join();   //kill publisher thread
        ROS_INFO("DRHWInterface::~DRHWInterface() 2");

        m_th_subscribe.join();   //kill subscribe thread 
        ROS_INFO("DRHWInterface::~DRHWInterface() 3");
    }

    bool DRHWInterface::init()
    {
        ROS_INFO("[dsr_hw_interface] init() ==> setup callback fucntion");
        int nServerPort = 12345;

        //--- doosan API's call-back fuctions : Only work within 50msec in call-back functions
        Drfl.SetOnTpInitializingCompleted(OnTpInitializingCompletedCB);
        Drfl.SetOnHommingCompleted(OnHommingCompletedCB);
        Drfl.SetOnProgramStopped(OnProgramStoppedCB);
        Drfl.SetOnMonitoringCtrlIO(OnMonitoringCtrlIOCB);
        Drfl.SetOnMonitoringModbus(OnMonitoringModbusCB);
        Drfl.SetOnMonitoringData(OnMonitoringDataCB);
        Drfl.SetOnMonitoringState(OnMonitoringStateCB);
        Drfl.SetOnMonitoringAccessControl(OnMonitoringAccessControlCB);
        Drfl.SetOnLogAlarm(OnLogAlarm);
        ROS_INFO("[dsr_hw_interface] init() ==> arm is standby");
        std::string host;
        std::string mode;
        private_nh_.getParam("host", host);
        private_nh_.param<int>("port", nServerPort, 12345);

        private_nh_.param<bool>("command", bCommand_, false);
        private_nh_.getParam("mode", mode);

        //for test host = "127.0.0.1";

        ROS_INFO("host %s, port=%d bCommand: %d, mode: %s\n", host.c_str(), nServerPort, bCommand_, mode.c_str());

        if(Drfl.OpenConnection(host, nServerPort))
        {
            //--- Get version ---            
            SYSTEM_VERSION tSysVerion = {'\0', };
            assert(Drfl.GetSystemVersion(&tSysVerion));
            ROS_INFO("DRCF version = %s",tSysVerion._szController);
            ROS_INFO("DRFL version = %s",Drfl.GetLibraryVersion());

            //--- Check Robot State : STATE_STANDBY ---               
            int delay;
            ros::param::param<int>("~standby", delay, 5000);
            while ((Drfl.GetRobotState() != STATE_STANDBY)){
                usleep(delay);
            }

            //--- Set Robot mode : NANUAL
            assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));

            //--- Set Robot mode : virual or real 
            ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
            if(mode == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
            assert(Drfl.SetRobotSystem(eTargetSystem));

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
    void DRHWInterface::sigint_handler( int signo)
    {
        ROS_INFO("[sigint_hangler] CloseConnection");
        ROS_INFO("[sigint_hangler] CloseConnection");
        ROS_INFO("[sigint_hangler] CloseConnection");
    }
    void DRHWInterface::positionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        ROS_INFO("callback: Position received");
        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->data.cbegin(), msg->data.cend(), target_pos.begin());
        Drfl.MoveJAsync(target_pos.data(), 50, 50);
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
        Drfl.MoveSJ(fTargetPos, nCntTargetPos, 0.0, 0.0, targetTime, (MOVE_MODE)MOVE_MODE_ABSOLUTE);

        //Drfl.MoveJAsync(degrees.data(), 30, 30, 0, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_OVERRIDE);


        /*
        for(int i = 0; i < NUM_JOINT; i++){
            ROS_INFO("[]::cmd %d-pos: %7.3f", i, joints[i].cmd);
            cmd_[i] = joints[i].cmd;
        }
        */
    }

    //----- Service Call-back functions ------------------------------------------------------------
    bool DRHWInterface::movej_cb(dsr_msgs::MoveJoint::Request& req, dsr_msgs::MoveJoint::Response& res)
    {
        std::array<float, NUM_JOINT> target_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        if(req.syncType == 0){
            ROS_INFO("DRHWInterface::movej_cb() called and calling Drfl.MoveJ");
            res.success = Drfl.MoveJ(target_pos.data(), req.vel, req.acc, req.time, (MOVE_MODE)req.mode, req.radius, (BLENDING_SPEED_TYPE)req.blendType);   
        }
        else{
            ROS_INFO("DRHWInterface::movej_cb() called and calling Drfl.MoveJAsync");
            res.success = Drfl.MoveJAsync(target_pos.data(), req.vel, req.acc, req.time, (MOVE_MODE)req.mode, (BLENDING_SPEED_TYPE)req.blendType);
        }
    }
    bool DRHWInterface::movel_cb(dsr_msgs::MoveLine::Request& req, dsr_msgs::MoveLine::Response& res)
    {
        std::array<float, NUM_TASK> target_pos;
        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        std::copy(req.vel.cbegin(), req.vel.cend(), target_vel.begin());
        std::copy(req.acc.cbegin(), req.acc.cend(), target_acc.begin());

        if(req.syncType == 0){
            ROS_INFO("DRHWInterface::movel_cb() called and calling Drfl.MoveL");
            res.success = Drfl.MoveL(target_pos.data(), target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, req.radius, (BLENDING_SPEED_TYPE)req.blendType);
        }
        else{
            ROS_INFO("DRHWInterface::movel_cb() called and calling Drfl.MoveLAsync");
            res.success = Drfl.MoveLAsync(target_pos.data(), target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (BLENDING_SPEED_TYPE)req.blendType);
        }
    }
    bool DRHWInterface::movejx_cb(dsr_msgs::MoveJointx::Request& req, dsr_msgs::MoveJointx::Response& res)
    {
        std::array<float, NUM_TASK> target_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        if(req.syncType == 0){
            ROS_INFO("DRHWInterface::movejx_cb() called and calling Drfl.MoveJX");
            res.success = Drfl.MoveJX(target_pos.data(), req.sol, req.vel, req.acc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, req.radius, (BLENDING_SPEED_TYPE)req.blendType);    
        }
        else{
            ROS_INFO("DRHWInterface::movejx_cb() called and calling Drfl.MoveJXAsync");
            res.success = Drfl.MoveJXAsync(target_pos.data(), req.sol, req.vel, req.acc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (BLENDING_SPEED_TYPE)req.blendType);    
        }
    }
    bool DRHWInterface::movec_cb(dsr_msgs::MoveCircle::Request& req, dsr_msgs::MoveCircle::Response& res)
    {
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
            ROS_INFO("DRHWInterface::movec_cb() called and calling Drfl.MoveC");
            res.success = Drfl.MoveC(fTargetPos, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, req.radius, (BLENDING_SPEED_TYPE)req.blendType);      
        }
        else{
            ROS_INFO("DRHWInterface::movec_cb() called and calling Drfl.MoveCAsync");
            res.success = Drfl.MoveCAsync(fTargetPos, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (BLENDING_SPEED_TYPE)req.blendType);  
        }
    }
    bool DRHWInterface::movesj_cb(dsr_msgs::MoveSplineJoint::Request& req, dsr_msgs::MoveSplineJoint::Response& res)
    {
        float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT];

        for(int i=0; i<req.posCnt; i++){
            for(int j=0; j<NUM_JOINT; j++){
                std_msgs::Float64MultiArray pos = req.pos.at(i);
                fTargetPos[i][j] = pos.data[j];
            }
        }
        if(req.syncType == 0){
            ROS_INFO("DRHWInterface::movesj_cb() called and calling Drfl.MoveSJ");
            res.success = Drfl.MoveSJ(fTargetPos, req.posCnt, req.vel, req.acc, req.time, (MOVE_MODE)req.mode);
        }
        else{
            ROS_INFO("DRHWInterface::movesj_cb() called and calling Drfl.MoveSJAsync");
            res.success = Drfl.MoveSJAsync(fTargetPos, req.posCnt, req.vel, req.acc, req.time, (MOVE_MODE)req.mode);
        }
    }
    bool DRHWInterface::movesx_cb(dsr_msgs::MoveSplineTask::Request& req, dsr_msgs::MoveSplineTask::Response& res)
    {
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
            ROS_INFO("DRHWInterface::movesx_cb() called and calling Drfl.MoveSX");
            res.success = Drfl.MoveSX(fTargetPos, req.posCnt, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (SPLINE_VELOCITY_OPTION)req.opt);
        }
        else{
            ROS_INFO("DRHWInterface::movesx_cb() called and calling Drfl.MoveSXAsync");
            res.success = Drfl.MoveSXAsync(fTargetPos, req.posCnt, fTargetVel, fTargetAcc, req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref, (SPLINE_VELOCITY_OPTION)req.opt);
        }
    }
    bool DRHWInterface::moveb_cb(dsr_msgs::MoveBlending::Request& req, dsr_msgs::MoveBlending::Response& res)
    {

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

        for(int i=0; i<req.posCnt; i++){
            printf("----- segment %d -----\n",i);
            printf("    pos1: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                    ,posb[i]._fTargetPos[0][0], posb[i]._fTargetPos[0][1], posb[i]._fTargetPos[0][2], posb[i]._fTargetPos[0][3], posb[i]._fTargetPos[0][4], posb[i]._fTargetPos[0][5]);

            printf("    pos2: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                    ,posb[i]._fTargetPos[1][0], posb[i]._fTargetPos[1][1], posb[i]._fTargetPos[1][2], posb[i]._fTargetPos[1][3], posb[i]._fTargetPos[1][4], posb[i]._fTargetPos[1][5]);

            printf("    posb[%d]._iBlendType = %d\n",i,posb[i]._iBlendType); 
            printf("    posb[%d]._fBlendRad  = %f\n",i,posb[i]._fBlendRad); 
        }


        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req.vel.cbegin(), req.vel.cend(), target_vel.begin());
        std::copy(req.acc.cbegin(), req.acc.cend(), target_acc.begin());

        if(req.syncType == 0){
            ROS_INFO("DRHWInterface::moveb_cb() called and calling Drfl.MoveB");
            res.success = Drfl.MoveB(posb, req.posCnt, target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref);
        }
        else{
            ROS_INFO("DRHWInterface::moveb_cb() called and calling Drfl.MoveBAsync");
            res.success = Drfl.MoveBAsync(posb, req.posCnt, target_vel.data(), target_acc.data(), req.time, (MOVE_MODE)req.mode, (MOVE_REFERENCE)req.ref);
        }
    }
    bool DRHWInterface::movespiral_cb(dsr_msgs::MoveSpiral::Request& req, dsr_msgs::MoveSpiral::Response& res)
    {
        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req.vel.cbegin(), req.vel.cend(), target_vel.begin());
        std::copy(req.acc.cbegin(), req.acc.cend(), target_acc.begin());

        if(req.syncType == 0){
            ROS_INFO("DRHWInterface::movespiral_cb() called and calling Drfl.MoveSpiral");
            res.success = Drfl.MoveSpiral((TASK_AXIS)req.taskAxis, req.revolution, req.maxRadius, req.maxLength, target_vel.data(), target_acc.data(), req.time, (MOVE_REFERENCE)req.ref);
        }
        else{
            ROS_INFO("DRHWInterface::movespiral_cb() called and calling Drfl.MoveSpiralAsync");
            res.success = Drfl.MoveSpiralAsync((TASK_AXIS)req.taskAxis, req.revolution, req.maxRadius, req.maxLength, target_vel.data(), target_acc.data(), req.time, (MOVE_REFERENCE)req.ref);
        }
    }
    bool DRHWInterface::moveperiodic_cb(dsr_msgs::MovePeriodic::Request& req, dsr_msgs::MovePeriodic::Response& res)
    {
        std::array<float, NUM_TASK> target_amp;
        std::array<float, NUM_TASK> target_periodic;
        std::copy(req.amp.cbegin(), req.amp.cend(), target_amp.begin());
        std::copy(req.periodic.cbegin(), req.periodic.cend(), target_periodic.begin());
        if(req.syncType == 0){
            ROS_INFO("DRHWInterface::moveperiodic_cb() called and calling Drfl.MovePeriodic");
            res.success = Drfl.MovePeriodic(target_amp.data(), target_periodic.data(), req.acc, req.repeat, (MOVE_REFERENCE)req.ref);
        }
        else{
            ROS_INFO("DRHWInterface::moveperiodic_cb() called and calling Drfl.MovePeriodicAsync");
            res.success = Drfl.MovePeriodicAsync(target_amp.data(), target_periodic.data(), req.acc, req.repeat, (MOVE_REFERENCE)req.ref);
        }
    }

    bool DRHWInterface::movewait_cb(dsr_msgs::MoveWait::Request& req, dsr_msgs::MoveWait::Response& res)
    {
        ROS_INFO("DRHWInterface::movewait_cb() called and calling Drfl.MoveWait");
        res.success = Drfl.MoveWait();
    }

    bool DRHWInterface::set_digital_output_cb(dsr_msgs::SetCtrlBoxDigitalOutput::Request& req, dsr_msgs::SetCtrlBoxDigitalOutput::Response& res)
    {
        ROS_INFO("DRHWInterface::set_digital_output_cb() called and calling Drfl.SetCtrlBoxDigitalOutput");
        res.success = Drfl.SetCtrlBoxDigitalOutput((GPIO_CTRLBOX_DIGITAL_INDEX)req.index, req.value);
    }
    bool DRHWInterface::get_digital_input_cb(dsr_msgs::GetCtrlBoxDigitalInput::Request& req, dsr_msgs::GetCtrlBoxDigitalInput::Response& res)
    {
        ROS_INFO("DRHWInterface::get_digital_input_cb() called and calling Drfl.GetCtrlBoxDigitalInput");
        res.value = Drfl.GetCtrlBoxDigitalInput((GPIO_CTRLBOX_DIGITAL_INDEX)req.index);
    }
    bool DRHWInterface::set_tool_digital_output_cb(dsr_msgs::SetToolDigitalOutput::Request& req, dsr_msgs::SetToolDigitalOutput::Response& res)
    {
        ROS_INFO("DRHWInterface::set_tool_digital_output_cb() called and calling Drfl.SetToolDigitalOutput");
        res.success = Drfl.SetToolDigitalOutput((GPIO_TOOL_DIGITAL_INDEX)req.index, req.value);
    }
    bool DRHWInterface::get_tool_digital_input_cb(dsr_msgs::GetToolDigitalInput::Request& req, dsr_msgs::GetToolDigitalInput::Response& res)
    {
        ROS_INFO("DRHWInterface::get_tool_digital_input_cb() called and calling Drfl.GetToolDigitalInput");
        res.value = Drfl.GetToolDigitalInput((GPIO_TOOL_DIGITAL_INDEX)req.index);
    }
    bool DRHWInterface::set_analog_output_cb(dsr_msgs::SetCtrlBoxAnalogOutput::Request& req, dsr_msgs::SetCtrlBoxAnalogOutput::Response& res)
    {
        ROS_INFO("DRHWInterface::set_analog_output_cb() called and calling Drfl.SetCtrlBoxAnalogOutput");
        res.success = Drfl.SetCtrlBoxAnalogOutput((GPIO_CTRLBOX_ANALOG_INDEX)req.channel, req.value);
    }
    bool DRHWInterface::get_analog_input_cb(dsr_msgs::GetCtrlBoxAnalogInput::Request& req, dsr_msgs::GetCtrlBoxAnalogInput::Response& res)
    {
        ROS_INFO("DRHWInterface::get_analog_input_cb() called and calling Drfl.GetCtrlBoxAnalogInput");
        res.value = Drfl.GetCtrlBoxAnalogInput((GPIO_CTRLBOX_ANALOG_INDEX)req.channel);
    }
    bool DRHWInterface::set_analog_output_type_cb(dsr_msgs::SetCtrlBoxAnalogOutputType::Request& req, dsr_msgs::SetCtrlBoxAnalogOutputType::Response& res)
    {
        ROS_INFO("DRHWInterface::set_analog_output_type_cb() called and calling Drfl.SetCtrlBoxAnalogOutputType");
        res.success = Drfl.SetCtrlBoxAnalogOutputType((GPIO_CTRLBOX_ANALOG_INDEX)req.channel, (GPIO_ANALOG_TYPE)req.mode);
    }
    bool DRHWInterface::set_analog_input_type_cb(dsr_msgs::SetCtrlBoxAnalogInputType::Request& req, dsr_msgs::SetCtrlBoxAnalogInputType::Response& res)
    {
        ROS_INFO("DRHWInterface::set_analog_input_type_cb() called and calling Drfl.SetCtrlBoxAnalogInputType");
        res.success = Drfl.SetCtrlBoxAnalogInputType((GPIO_CTRLBOX_ANALOG_INDEX)req.channel, (GPIO_ANALOG_TYPE)req.mode);
    }
    bool DRHWInterface::set_modbus_output_cb(dsr_msgs::SetModbusOutput::Request& req, dsr_msgs::SetModbusOutput::Response& res)
    {
        ROS_INFO("DRHWInterface::set_modbus_output_cb() called and calling Drfl.SetModbusOutput");
        res.success = Drfl.SetModbusValue(req.name, (unsigned short)req.value);
    }
    bool DRHWInterface::get_modbus_input_cb(dsr_msgs::GetModbusInput::Request& req, dsr_msgs::GetModbusInput::Response& res)
    {
        ROS_INFO("DRHWInterface::get_modbus_input_cb() called and calling Drfl.GetModbusInput");
        res.value = Drfl.GetModbusValue(req.name);
    }
    bool DRHWInterface::config_create_modbus_cb(dsr_msgs::ConfigCreateModbus::Request& req, dsr_msgs::ConfigCreateModbus::Response& res)
    {
        ROS_INFO("DRHWInterface::config_create_modbus_cb() called and calling Drfl.ConfigCreateModbus");
        res.success = Drfl.ConfigCreateModbus(req.name, req.ip, (unsigned short)req.port, (MODBUS_REGISTER_TYPE)req.reg_type, (unsigned short)req.index, (unsigned short)req.value);
    }
    bool DRHWInterface::config_delete_modbus_cb(dsr_msgs::ConfigDeleteModbus::Request& req, dsr_msgs::ConfigDeleteModbus::Response& res)
    {
        ROS_INFO("DRHWInterface::config_delete_modbus_cb() called and calling Drfl.ConfigDeleteModbus");
        res.success = Drfl.ConfigDeleteModbus(req.name);
    }
    bool DRHWInterface::drl_pause_cb(dsr_msgs::DrlPause::Request& req, dsr_msgs::DrlPause::Response& res)
    {
        ROS_INFO("DRHWInterface::drl_pause_cb() called and calling Drfl.DrlPause");
        res.success = Drfl.PlayDrlPause();
    }
    bool DRHWInterface::drl_start_cb(dsr_msgs::DrlStart::Request& req, dsr_msgs::DrlStart::Response& res)
    {
        ROS_INFO("DRHWInterface::drl_start_cb() called and calling Drfl.DrlStart");
        res.success = Drfl.PlayDrlStart((ROBOT_SYSTEM)req.robotSystem, req.code);
    }
    bool DRHWInterface::drl_stop_cb(dsr_msgs::DrlStop::Request& req, dsr_msgs::DrlStop::Response& res)
    {
        ROS_INFO("DRHWInterface::drl_stop_cb() called and calling Drfl.DrlStop");
        res.success = Drfl.PlayDrlStop((STOP_TYPE)req.stop_mode);
    }
    bool DRHWInterface::drl_resume_cb(dsr_msgs::DrlResume::Request& req, dsr_msgs::DrlResume::Response& res)
    {
        ROS_INFO("DRHWInterface::drl_resume_cb() called and calling Drfl.DrlResume");
        res.success = Drfl.PlayDrlResume();
    }
    bool DRHWInterface::set_current_tcp_cb(dsr_msgs::SetCurrentTcp::Request& req, dsr_msgs::SetCurrentTcp::Response& res)
    {
        ROS_INFO("DRHWInterface::set_current_tcp_cb() called and calling Drfl.SetCurrentTCP");
        res.success = Drfl.SetCurrentTCP(req.name);
    }
    bool DRHWInterface::get_current_tcp_cb(dsr_msgs::GetCurrentTcp::Request& req, dsr_msgs::GetCurrentTcp::Response& res)
    {
        ROS_INFO("DRHWInterface::get_current_tcp_cb() called and calling Drfl.GetCurrentTCP");
        res.info = Drfl.GetCurrentTCP();
    }
    bool DRHWInterface::config_create_tcp_cb(dsr_msgs::ConfigCreateTcp::Request& req, dsr_msgs::ConfigCreateTcp::Response& res)
    {
        ROS_INFO("DRHWInterface::config_create_tcp_cb() called and calling Drfl.ConfigCreateTCP");
        std::array<float, 6> target_pos;
        std::copy(req.pos.cbegin(), req.pos.cend(), target_pos.begin());
        res.success = Drfl.ConfigCreateTCP(req.name, target_pos.data());
    }
    bool DRHWInterface::config_delete_tcp_cb(dsr_msgs::ConfigDeleteTcp::Request& req, dsr_msgs::ConfigDeleteTcp::Response& res)
    {
        ROS_INFO("DRHWInterface::config_delete_tcp_cb() called and calling Drfl.ConfigDeleteTCP");
        res.success = Drfl.ConfigDeleteTCP(req.name);
    }
    bool DRHWInterface::set_current_tool_cb(dsr_msgs::SetCurrentTool::Request& req, dsr_msgs::SetCurrentTool::Response& res)
    {
        ROS_INFO("DRHWInterface::set_current_tool_cb() called and calling Drfl.SetCurrentTool");
        res.success = Drfl.SetCurrentTool(req.name);
    }
    bool DRHWInterface::get_current_tool_cb(dsr_msgs::GetCurrentTool::Request& req, dsr_msgs::GetCurrentTool::Response& res)
    {
        ROS_INFO("DRHWInterface::get_current_tool_cb() called and calling Drfl.GetCurrentTool %s", Drfl.GetCurrentTool().c_str());
        res.info = Drfl.GetCurrentTool();
    }
    bool DRHWInterface::config_create_tool_cb(dsr_msgs::ConfigCreateTool::Request& req, dsr_msgs::ConfigCreateTool::Response& res)
    {
        ROS_INFO("DRHWInterface::config_create_tool_cb() called and calling Drfl.ConfigCreateTool");
        std::array<float, 3> target_cog;
        std::array<float, 6> target_inertia;
        std::copy(req.cog.cbegin(), req.cog.cend(), target_cog.begin());
        std::copy(req.inertia.cbegin(), req.inertia.cend(), target_inertia.begin());
        res.success = Drfl.ConfigCreateTool(req.name, req.weight, target_cog.data(), target_inertia.data());
    }
    bool DRHWInterface::config_delete_tool_cb(dsr_msgs::ConfigDeleteTool::Request& req, dsr_msgs::ConfigDeleteTool::Response& res)
    {
        ROS_INFO("DRHWInterface::config_delete_tool_cb() called and calling Drfl.ConfigDeleteTool");
        res.success = Drfl.ConfigDeleteTool(req.name);
    }
    
    //Gripper Service

    bool DRHWInterface::robotiq_2f_move_cb(dsr_msgs::Robotiq2FMove::Request& req, dsr_msgs::Robotiq2FMove::Response& res)
    {
        ROS_INFO("DRHWInterface::gripper_move_cb() called and calling Nothing");
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
    }
    bool DRHWInterface::robotiq_2f_open_cb(dsr_msgs::Robotiq2FOpen::Request& req, dsr_msgs::Robotiq2FOpen::Response& res){
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
        
    }
    bool DRHWInterface::robotiq_2f_close_cb(dsr_msgs::Robotiq2FClose::Request& req, dsr_msgs::Robotiq2FClose::Response& res){
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
    }

    bool DRHWInterface::serial_open_cb(dsr_msgs::SerialOpen::Request& req, dsr_msgs::SerialOpen::Response& res){
    }

    bool DRHWInterface::serial_close_cb(dsr_msgs::SerialClose::Request& req, dsr_msgs::SerialClose::Response& res){

    }

    bool DRHWInterface::serial_send_data_cb(dsr_msgs::SerialSendData::Request& req, dsr_msgs::SerialSendData::Response &res){
        std_msgs::String send_data;
        send_data.data = req.data;
        m_PubSerialWrite.publish(send_data);
        res.success = true;
    }

}