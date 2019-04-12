/*********************************************************************
 *
 *  Inferfaces for doosan robot controllor 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
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

#ifndef DR_HW_INTERFACE_H
#define DR_HW_INTERFACE_H

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <array>
#include <algorithm>  // std::copy
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

// msg
#include <dsr_msgs/RobotError.h>
#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/RobotStop.h>

// service
// motion
#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/MoveJointx.h>
#include <dsr_msgs/MoveCircle.h>
#include <dsr_msgs/MoveSplineJoint.h>
#include <dsr_msgs/MoveSplineTask.h>
#include <dsr_msgs/MoveBlending.h>
#include <dsr_msgs/MoveSpiral.h>
#include <dsr_msgs/MovePeriodic.h>
#include <dsr_msgs/MoveWait.h>
//io
#include <dsr_msgs/SetCtrlBoxDigitalOutput.h>
#include <dsr_msgs/GetCtrlBoxDigitalInput.h>
#include <dsr_msgs/SetToolDigitalOutput.h>
#include <dsr_msgs/GetToolDigitalInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutput.h>
#include <dsr_msgs/GetCtrlBoxAnalogInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutputType.h>
#include <dsr_msgs/SetCtrlBoxAnalogInputType.h>

//modbus
#include <dsr_msgs/SetModbusOutput.h>
#include <dsr_msgs/GetModbusInput.h>
#include <dsr_msgs/ConfigCreateModbus.h>
#include <dsr_msgs/ConfigDeleteModbus.h>

//drl
#include <dsr_msgs/DrlPause.h>
#include <dsr_msgs/DrlStart.h>
#include <dsr_msgs/DrlStop.h>
#include <dsr_msgs/DrlResume.h>


//tcp
#include <dsr_msgs/ConfigCreateTcp.h>
#include <dsr_msgs/ConfigDeleteTcp.h>
#include <dsr_msgs/GetCurrentTcp.h>
#include <dsr_msgs/SetCurrentTcp.h>

//tool
#include <dsr_msgs/ConfigCreateTool.h>
#include <dsr_msgs/ConfigDeleteTool.h>
#include <dsr_msgs/GetCurrentTool.h>
#include <dsr_msgs/SetCurrentTool.h>

//gripper
#include <dsr_msgs/Robotiq2FOpen.h>
#include <dsr_msgs/Robotiq2FClose.h>
#include <dsr_msgs/Robotiq2FMove.h>

//serial
#include <dsr_msgs/SerialSendData.h>

// moveit
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

///#include "DRFL.h"
#include "../../../common/include/DRFL.h"
#include "../../../common/include/dsr_serial.h"

#ifndef PI
#define PI 3.14159265359
#endif
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

typedef struct {
    int	    nLevel;         // INFO =1, WARN =2, ERROR =3 
    int	    nGroup;         // SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
    int	    nCode;          // error code 
    char    strMsg1[MAX_STRING_SIZE];   // error msg 1
    char    strMsg2[MAX_STRING_SIZE];   // error msg 2
    char    strMsg3[MAX_STRING_SIZE];   // error msg 3
} DR_ERROR, *LPDR_ERROR;

typedef struct {
    int     nRobotState;
    char    strRobotState[MAX_SYMBOL_SIZE];
    float   fCurrentPosj[NUM_JOINT];
    float   fCurrentPosx[NUM_TASK];
    int     nIoControlBox;
    //io_modbus           # GJH
    //error               # GJH
    int     nAccessControl;
    bool    bHommingCompleted;
    bool    bTpInitialized;
    int     nSpeed;
    bool    bMasteringNeed;
    bool    bDrlStopped;
    bool    bDisconnected;
} DR_STATE, *LPDR_STATE;

using namespace DRAFramework;

namespace dsr_control{

    class DRHWInterface : public hardware_interface::RobotHW
    {
    public:
        DRHWInterface(ros::NodeHandle& nh);
        virtual ~DRHWInterface();

        bool init();
        virtual void read(ros::Duration& elapsed_time);
        virtual void write(ros::Duration& elapsed_time);
        int MsgPublisher_RobotState();
        ///int MsgPublisher_RobotError();  현재 미사용 : DRHWInterface::OnLogAlarm 에서 바로 퍼블리싱 함.
        static void OnHommingCompletedCB();
        static void OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause);
        static void OnMonitoringDataCB(const LPMONITORING_DATA pData);
        static void OnTpInitializingCompletedCB();

        static void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO);
        static void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus);
        static void OnMonitoringStateCB(const ROBOT_STATE eState);
        static void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl);
        static void OnLogAlarm(LPLOG_ALARM pLogAlarm);


    private:
        ros::NodeHandle private_nh_;

        std::string m_strRobotName;
        std::string m_strRobotModel;
        std::string m_strRobotGripper;

        //----- Service ---------------------------------------------------------------
        ros::ServiceServer m_nh_move_service[10];
        ros::ServiceServer m_nh_io_service[8];
        ros::ServiceServer m_nh_modbus_service[4];
        ros::ServiceServer m_nh_drl_service[4];
        ros::ServiceServer m_nh_tcp_service[4];
        ros::ServiceServer m_nh_tool_service[4];
        ros::ServiceServer m_nh_gripper_service[10];
        ros::ServiceServer m_nh_serial_service[4];

        //----- Publisher -------------------------------------------------------------
        ros::Publisher m_PubRobotState;
        ros::Publisher m_PubRobotError;
        ros::Publisher m_PubtoGazebo;
        ros::Publisher m_PubSerialWrite;

        //----- Subscriber ------------------------------------------------------------
        ros::Subscriber m_sub_joint_trajectory;
        ros::Subscriber m_sub_joint_position;
        ros::Subscriber m_SubSerialRead;

        // ROS Interface
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        std::array<float, NUM_JOINT> cmd_;
        bool bCommand_;
        struct Joint{
            double cmd;
            double pos;
            double vel;
            double eff;
            Joint(): cmd(0), pos(0), vel(0), eff(0) {}
        } joints[NUM_JOINT];

        //----- SIG Handler --------------------------------------------------------------
        void sigint_handler( int signo);
        void trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
        void positionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

        //----- Threads ------------------------------------------------------------------
        boost::thread m_th_subscribe;   //subscribe thread
        boost::thread m_th_publisher;   //publisher thread
        static void thread_subscribe(ros::NodeHandle nh);
        static void thread_publisher(DRHWInterface* pDRHWInterface, ros::NodeHandle nh, int nPubRate);

        DR_STATE m_stDrState;
        DR_ERROR m_stDrError;

        //----- Service Call-back functions ----------------------------------------------
        //----- MOTION
        bool movej_cb(dsr_msgs::MoveJoint::Request& req, dsr_msgs::MoveJoint::Response& res);
        bool movel_cb(dsr_msgs::MoveLine::Request& req, dsr_msgs::MoveLine::Response& res);
        bool movejx_cb(dsr_msgs::MoveJointx::Request& req, dsr_msgs::MoveJointx::Response& res);
        bool movec_cb(dsr_msgs::MoveCircle::Request& req, dsr_msgs::MoveCircle::Response& res);
        bool movesj_cb(dsr_msgs::MoveSplineJoint::Request& req, dsr_msgs::MoveSplineJoint::Response& res);
        bool movesx_cb(dsr_msgs::MoveSplineTask::Request& req, dsr_msgs::MoveSplineTask::Response& res);
        bool moveb_cb(dsr_msgs::MoveBlending::Request& req, dsr_msgs::MoveBlending::Response& res);
        bool movespiral_cb(dsr_msgs::MoveSpiral::Request& req, dsr_msgs::MoveSpiral::Response& res);
        bool moveperiodic_cb(dsr_msgs::MovePeriodic::Request& req, dsr_msgs::MovePeriodic::Response& res);
        bool movewait_cb(dsr_msgs::MoveWait::Request& req, dsr_msgs::MoveWait::Response& res);
        

        //----- TCP
        bool set_current_tcp_cb(dsr_msgs::SetCurrentTcp::Request& req, dsr_msgs::SetCurrentTcp::Response& res);
        bool get_current_tcp_cb(dsr_msgs::GetCurrentTcp::Request& req, dsr_msgs::GetCurrentTcp::Response& res);
        bool config_create_tcp_cb(dsr_msgs::ConfigCreateTcp::Request& req, dsr_msgs::ConfigCreateTcp::Response& res);
        bool config_delete_tcp_cb(dsr_msgs::ConfigDeleteTcp::Request& req, dsr_msgs::ConfigDeleteTcp::Response& res);

        //----- TOOL
        bool set_current_tool_cb(dsr_msgs::SetCurrentTool::Request& req, dsr_msgs::SetCurrentTool::Response& res);
        bool get_current_tool_cb(dsr_msgs::GetCurrentTool::Request& req, dsr_msgs::GetCurrentTool::Response& res);
        bool config_create_tool_cb(dsr_msgs::ConfigCreateTool::Request& req, dsr_msgs::ConfigCreateTool::Response& res);
        bool config_delete_tool_cb(dsr_msgs::ConfigDeleteTool::Request& req, dsr_msgs::ConfigDeleteTool::Response& res);

        //----- IO
        bool set_digital_output_cb(dsr_msgs::SetCtrlBoxDigitalOutput::Request& req, dsr_msgs::SetCtrlBoxDigitalOutput::Response& res);
        bool get_digital_input_cb(dsr_msgs::GetCtrlBoxDigitalInput::Request& req, dsr_msgs::GetCtrlBoxDigitalInput::Response& res);
        bool set_tool_digital_output_cb(dsr_msgs::SetToolDigitalOutput::Request& req, dsr_msgs::SetToolDigitalOutput::Response& res);
        bool get_tool_digital_input_cb(dsr_msgs::GetToolDigitalInput::Request& req, dsr_msgs::GetToolDigitalInput::Response& res);

        bool set_analog_output_cb(dsr_msgs::SetCtrlBoxAnalogOutput::Request& req, dsr_msgs::SetCtrlBoxAnalogOutput::Response& res);
        bool get_analog_input_cb(dsr_msgs::GetCtrlBoxAnalogInput::Request& req, dsr_msgs::GetCtrlBoxAnalogInput::Response& res);
        bool set_analog_output_type_cb(dsr_msgs::SetCtrlBoxAnalogOutputType::Request& req, dsr_msgs::SetCtrlBoxAnalogOutputType::Response& res);
        bool set_analog_input_type_cb(dsr_msgs::SetCtrlBoxAnalogInputType::Request& req, dsr_msgs::SetCtrlBoxAnalogInputType::Response& res);

        //----- MODBUS
        bool set_modbus_output_cb(dsr_msgs::SetModbusOutput::Request& req, dsr_msgs::SetModbusOutput::Response& res);
        bool get_modbus_input_cb(dsr_msgs::GetModbusInput::Request& req, dsr_msgs::GetModbusInput::Response& res);
        bool config_create_modbus_cb(dsr_msgs::ConfigCreateModbus::Request& req, dsr_msgs::ConfigCreateModbus::Response& res);
        bool config_delete_modbus_cb(dsr_msgs::ConfigDeleteModbus::Request& req, dsr_msgs::ConfigDeleteModbus::Response& res);

        //----- DRL        
        bool drl_pause_cb(dsr_msgs::DrlPause::Request& req, dsr_msgs::DrlPause::Response& res);
        bool drl_start_cb(dsr_msgs::DrlStart::Request& req, dsr_msgs::DrlStart::Response& res);
        bool drl_stop_cb(dsr_msgs::DrlStop::Request& req, dsr_msgs::DrlStop::Response& res);
        bool drl_resume_cb(dsr_msgs::DrlResume::Request& req, dsr_msgs::DrlResume::Response& res);

        //----- Gripper
        bool robotiq_2f_open_cb(dsr_msgs::Robotiq2FOpen::Request& req, dsr_msgs::Robotiq2FOpen::Response& res);
        bool robotiq_2f_close_cb(dsr_msgs::Robotiq2FClose::Request& req, dsr_msgs::Robotiq2FClose::Response& res);
        bool robotiq_2f_move_cb(dsr_msgs::Robotiq2FMove::Request& req, dsr_msgs::Robotiq2FMove::Response& res);

        //----- Serial
        bool serial_send_data_cb(dsr_msgs::SerialSendData::Request& req, dsr_msgs::SerialSendData::Response& res);

    };
}

#endif // end