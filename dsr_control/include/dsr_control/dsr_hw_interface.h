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
#define _DEBUG_DSR_CTL      1

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
#include <dsr_msgs/JogMultiAxis.h>

// service
//system
#include <dsr_msgs/SetRobotMode.h>
#include <dsr_msgs/GetRobotMode.h>
#include <dsr_msgs/SetRobotSystem.h>
#include <dsr_msgs/GetRobotSystem.h>
#include <dsr_msgs/GetRobotState.h>
#include <dsr_msgs/SetRobotSpeedMode.h>
#include <dsr_msgs/GetRobotSpeedMode.h>
#include <dsr_msgs/GetCurrentPose.h>
#include <dsr_msgs/SetSafeStopResetType.h>
#include <dsr_msgs/GetLastAlarm.h>

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
#include <dsr_msgs/Jog.h>
#include <dsr_msgs/JogMulti.h>
#include <dsr_msgs/MovePause.h>
#include <dsr_msgs/MoveStop.h>
#include <dsr_msgs/MoveResume.h>
#include <dsr_msgs/Trans.h>
#include <dsr_msgs/Fkin.h>
#include <dsr_msgs/Ikin.h>
#include <dsr_msgs/SetRefCoord.h>
#include <dsr_msgs/MoveHome.h>
#include <dsr_msgs/CheckMotion.h>
#include <dsr_msgs/ChangeOperationSpeed.h>
#include <dsr_msgs/EnableAlterMotion.h>
#include <dsr_msgs/AlterMotion.h>
#include <dsr_msgs/DisableAlterMotion.h>
#include <dsr_msgs/SetSingularityHandling.h>

//----- auxiliary_control
#include <dsr_msgs/GetControlMode.h>          
#include <dsr_msgs/GetControlSpace.h>         
#include <dsr_msgs/GetCurrentPosj.h>          
#include <dsr_msgs/GetCurrentVelj.h>          
#include <dsr_msgs/GetDesiredPosj.h>
#include <dsr_msgs/GetDesiredVelj.h>          
#include <dsr_msgs/GetCurrentPosx.h>          
#include <dsr_msgs/GetCurrentToolFlangePosx.h>
#include <dsr_msgs/GetCurrentVelx.h>          
#include <dsr_msgs/GetDesiredPosx.h>
#include <dsr_msgs/GetDesiredVelx.h>          
#include <dsr_msgs/GetCurrentSolutionSpace.h> 
#include <dsr_msgs/GetCurrentRotm.h>          
#include <dsr_msgs/GetJointTorque.h>          
#include <dsr_msgs/GetExternalTorque.h>      
#include <dsr_msgs/GetToolForce.h>            
#include <dsr_msgs/GetSolutionSpace.h>
#include <dsr_msgs/GetOrientationError.h>

//----- force/stiffness
#include <dsr_msgs/ParallelAxis1.h>
#include <dsr_msgs/ParallelAxis2.h>
#include <dsr_msgs/AlignAxis1.h>
#include <dsr_msgs/AlignAxis2.h>
#include <dsr_msgs/IsDoneBoltTightening.h>
#include <dsr_msgs/ReleaseComplianceCtrl.h>
#include <dsr_msgs/TaskComplianceCtrl.h>
#include <dsr_msgs/SetStiffnessx.h>
#include <dsr_msgs/CalcCoord.h>
#include <dsr_msgs/SetUserCartCoord1.h>
#include <dsr_msgs/SetUserCartCoord2.h>
#include <dsr_msgs/SetUserCartCoord3.h>
#include <dsr_msgs/OverwriteUserCartCoord.h>
#include <dsr_msgs/GetUserCartCoord.h>
#include <dsr_msgs/SetDesiredForce.h>
#include <dsr_msgs/ReleaseForce.h>
#include <dsr_msgs/CheckPositionCondition.h>
#include <dsr_msgs/CheckForceCondition.h>
#include <dsr_msgs/CheckOrientationCondition1.h>
#include <dsr_msgs/CheckOrientationCondition2.h>
#include <dsr_msgs/CoordTransform.h>
#include <dsr_msgs/GetWorkpieceWeight.h>
#include <dsr_msgs/ResetWorkpieceWeight.h>

//io
#include <dsr_msgs/SetCtrlBoxDigitalOutput.h>
#include <dsr_msgs/GetCtrlBoxDigitalInput.h>
#include <dsr_msgs/SetToolDigitalOutput.h>
#include <dsr_msgs/GetToolDigitalInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutput.h>
#include <dsr_msgs/GetCtrlBoxAnalogInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutputType.h>
#include <dsr_msgs/SetCtrlBoxAnalogInputType.h>
#include <dsr_msgs/GetCtrlBoxDigitalOutput.h>
#include <dsr_msgs/GetToolDigitalOutput.h>

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
#include <dsr_msgs/GetDrlState.h>


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
#include <dsr_msgs/SetToolShape.h>

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

//_____ defines for Dooan Robot Controller _______________
#define POINT_COUNT         6

// solution space
#define DR_SOL_MIN          0
#define DR_SOL_MAX          7

// posb seg_type
#define DR_LINE             0
#define DR_CIRCLE           1

// move reference
#define DR_BASE             0
#define DR_TOOL             1
#define DR_WORLD            2
#define DR_TC_USER_MIN      101
#define DR_TC_USER_MAX      200

// move mod
#define DR_MV_MOD_ABS       0
#define DR_MV_MOD_REL       1

// move reaction
#define DR_MV_RA_NONE       0
#define DR_MV_RA_DUPLICATE  0
#define DR_MV_RA_OVERRIDE   1

// move command type
#define DR_MV_COMMAND_NORM  0

// movesx velocity
#define DR_MVS_VEL_NONE     0
#define DR_MVS_VEL_CONST    1

// motion state
#define DR_STATE_IDLE       0
#define DR_STATE_INIT       1
#define DR_STATE_BUSY       2
#define DR_STATE_BLEND      3
#define DR_STATE_ACC        4
#define DR_STATE_CRZ        5
#define DR_STATE_DEC        6

// axis
#define DR_AXIS_X           0
#define DR_AXIS_Y           1
#define DR_AXIS_Z           2
#define DR_AXIS_A          10
#define DR_AXIS_B          11
#define DR_AXIS_C          12

// collision sensitivity
#define DR_COLSENS_DEFAULT 20
#define DR_COLSENS_MIN      1   
#define DR_COLSENS_MAX    300

// speed
#define DR_OP_SPEED_MIN     1
#define DR_OP_SPEED_MAX   100

// stop
#define DR_QSTOP_STO        0
#define DR_QSTOP            1
#define DR_SSTOP            2
#define DR_HOLD             3

#define DR_STOP_FIRST       DR_QSTOP_STO
#define DR_STOP_LAST        DR_HOLD

// condition
#define DR_COND_NONE        -10000

// digital I/O
#define DR_DIO_MIN_INDEX    1
#define DR_DIO_MAX_INDEX    16  

// tool digital I/O
#define DR_TDIO_MIN_INDEX   1
#define DR_TDIO_MAX_INDEX   6

// I/O value
#define ON                  1
#define OFF                 0

// Analog I/O mode
#define DR_ANALOG_CURRENT   0
#define DR_ANALOG_VOLTAGE   1

// modbus type
#define DR_MODBUS_DIG_INPUT     0
#define DR_MODBUS_DIG_OUTPUT    1
#define DR_MODBUS_REG_INPUT     2
#define DR_MODBUS_REG_OUTPUT    3
#define DR_DISCRETE_INPUT       0
#define DR_COIL                 1
#define DR_INPUT_REGISTER       2
#define DR_HOLDING_REGISTER     3

#define DR_MODBUS_ACCESS_MAX    32
#define DR_MAX_MODBUS_NAME_SIZE 32

// tp_popup pm_type
#define DR_PM_MESSAGE           0
#define DR_PM_WARNING           1
#define DR_PM_ALARM             2

// tp_get_user_input type
#define DR_VAR_INT              0
#define DR_VAR_FLOAT            1
#define DR_VAR_STR              2
#define DR_VAR_BOOL             3   

// len
#define DR_VELJ_DT_LEN          6
#define DR_ACCJ_DT_LEN          6

#define DR_VELX_DT_LEN          2
#define DR_ACCX_DT_LEN          2

#define DR_ANGLE_DT_LEN         2
#define DR_COG_DT_LEN           3
#define DR_WEIGHT_DT_LEN        3
#define DR_VECTOR_DT_LEN        3
#define DR_ST_DT_LEN            6
#define DR_FD_DT_LEN            6
#define DR_DIR_DT_LEN           6
#define DR_INERTIA_DT_LEN       6
#define DR_VECTOR_U1_LEN        3
#define DR_VECTOR_V1_LEN        3

#define DR_AVOID                0
#define DR_TASK_STOP            1

#define DR_FIFO                 0
#define DR_LIFO                 1

#define DR_FC_MOD_ABS           0
#define DR_FC_MOD_REL           1

#define DR_GLOBAL_VAR_TYPE_BOOL         0
#define DR_GLOBAL_VAR_TYPE_INT          1
#define DR_GLOBAL_VAR_TYPE_FLOAT        2
#define DR_GLOBAL_VAR_TYPE_STR          3
#define DR_GLOBAL_VAR_TYPE_POSJ         4
#define DR_GLOBAL_VAR_TYPE_POSX         5
#define DR_GLOBAL_VAR_TYPE_UNKNOWN      6

#define DR_IE_SLAVE_GPR_ADDR_START      0
#define DR_IE_SLAVE_GPR_ADDR_END       23
#define DR_IE_SLAVE_GPR_ADDR_END_BIT   63

#define DR_DPOS                         0
#define DR_DVEL                         1

#define DR_HOME_TARGET_MECHANIC         0
#define DR_HOME_TARGET_USER             1

#define DR_MV_ORI_TEACH                 0    
#define DR_MV_ORI_FIXED                 1    
#define DR_MV_ORI_RADIAL                2    

#define DR_MV_APP_NONE                  0
#define DR_MV_APP_WELD                  1
//________________________________________________________

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
    float   fCurrentToolPosx[NUM_TASK];

    int     nActualMode;
    int     nActualSpace;
    
    float   fJointAbs[NUM_JOINT];
    float   fJointErr[NUM_JOINT];
    float   fTargetPosj[NUM_JOINT];
    float   fTargetVelj[NUM_JOINT];
    float   fCurrentVelj[NUM_JOINT];

    float   fTaskErr[NUM_TASK];
    float   fTargetPosx[NUM_TASK];
    float   fTargetVelx[NUM_TASK];
    float   fCurrentVelx[NUM_TASK];
    int     nSolutionSpace;
    float   fRotationMatrix[3][3];

    float   fDynamicTor[NUM_JOINT];
    float   fActualJTS[NUM_JOINT];
    float   fActualEJT[NUM_JOINT];
    float   fActualETT[NUM_JOINT];

    double  dSyncTime;
    int     nActualBK[NUM_JOINT];
    int     nActualBT[NUM_BUTTON];
    float   fActualMC[NUM_JOINT];
    float   fActualMT[NUM_JOINT];
    bool    bCtrlBoxDigitalOutput[16];
    bool    bCtrlBoxDigitalInput[16];
    bool    bFlangeDigitalOutput[6];
    bool    bFlangeDigitalInput[6];

    int     nRegCount;
    string  strModbusSymbol[100];
    int     nModbusValue[100];
  
    int     nAccessControl;
    bool    bHommingCompleted;
    bool    bTpInitialized;
    bool    bMasteringNeed;
    bool    bDrlStopped;
    bool    bDisconnected;

    //--- The following variables have been updated since version M2.50 or higher. ---
	//ROBOT_MONITORING_WORLD
	float   fActualW2B[6];
	float   fCurrentPosW[2][6];
	float   fCurrentVelW[6];
	float   fWorldETT[6];
	float   fTargetPosW[6];
	float   fTargetVelW[6];
	float   fRotationMatrixWorld[3][3];

	//ROBOT_MONITORING_USER
	int     iActualUCN;
	int     iParent;
	float   fCurrentPosU[2][6];
	float   fCurrentVelU[6];
	float   fUserETT[6];
	float   fTargetPosU[6];
	float   fTargetVelU[6];
	float   fRotationMatrixUser[3][3];

    //READ_CTRLIO_INPUT_EX
	float   fActualAI[6];
	bool    bActualSW[3];
	bool    bActualSI[2];
	int     iActualAT[2];

	//READ_CTRLIO_OUTPUT_EX
	float   fTargetAO[2];
	int     iTargetAT[2];

	//READ_ENCODER_INPUT
	bool    bActualES[2];
	int     iActualED[2];
	bool    bActualER[2];
    //---------------------------------------------------------------------------------

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
        static void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
        static void OnTpInitializingCompletedCB();

        static void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO);
        static void OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO);
        static void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus);
        static void OnMonitoringStateCB(const ROBOT_STATE eState);
        static void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl);
        static void OnLogAlarm(LPLOG_ALARM pLogAlarm);

        std::string GetRobotName();
        std::string GetRobotModel();

    private:
        int  m_nVersionDRCF;
        bool m_bIsEmulatorMode; 

        ros::NodeHandle private_nh_;

        std::string m_strRobotName;
        std::string m_strRobotModel;
        std::string m_strRobotGripper;

        //----- Service ---------------------------------------------------------------
        ros::ServiceServer m_nh_system[14];
        ros::ServiceServer m_nh_motion_service[32];
        ros::ServiceServer m_nh_aux_control_service[32];
        ros::ServiceServer m_nh_force_service[32];

        ros::ServiceServer m_nh_io_service[10];
        ros::ServiceServer m_nh_modbus_service[4];
        ros::ServiceServer m_nh_drl_service[10];
        ros::ServiceServer m_nh_tcp_service[4];
        ros::ServiceServer m_nh_tool_service[5];
        ros::ServiceServer m_nh_gripper_service[10];
        ros::ServiceServer m_nh_serial_service[4];

        //----- Publisher -------------------------------------------------------------
        ros::Publisher m_PubRobotState;
        ros::Publisher m_PubRobotError;
        ros::Publisher m_PubtoGazebo;
        ros::Publisher m_PubSerialWrite;
        ros::Publisher m_PubJogMultiAxis;

        //----- Subscriber ------------------------------------------------------------
        ros::Subscriber m_sub_joint_trajectory;
        ros::Subscriber m_sub_joint_position;
        ros::Subscriber m_SubSerialRead;
        ros::Subscriber m_sub_jog_multi_axis;

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

        void jogCallback(const dsr_msgs::JogMultiAxis::ConstPtr& msg);

        //----- Threads ------------------------------------------------------------------
        boost::thread m_th_subscribe;   //subscribe thread
        boost::thread m_th_publisher;   //publisher thread
        static void thread_subscribe(ros::NodeHandle nh);
        static void thread_publisher(DRHWInterface* pDRHWInterface, ros::NodeHandle nh, int nPubRate);

        DR_STATE m_stDrState;
        DR_ERROR m_stDrError;

        //----- Service Call-back functions ----------------------------------------------.
        //----- System
        bool set_robot_mode_cb(dsr_msgs::SetRobotMode::Request& req, dsr_msgs::SetRobotMode::Response& res);
        bool get_robot_mode_cb(dsr_msgs::GetRobotMode::Request& req, dsr_msgs::GetRobotMode::Response& res);
        bool set_robot_system_cb(dsr_msgs::SetRobotSystem::Request& req, dsr_msgs::SetRobotSystem::Response& res);
        bool get_robot_system_cb(dsr_msgs::GetRobotSystem::Request& req, dsr_msgs::GetRobotSystem::Response& res);
        bool get_robot_state_cb(dsr_msgs::GetRobotState::Request& req, dsr_msgs::GetRobotState::Response& res);
        bool set_robot_speed_mode_cb(dsr_msgs::SetRobotSpeedMode::Request& req, dsr_msgs::SetRobotSpeedMode::Response& res);
        bool get_robot_speed_mode_cb(dsr_msgs::GetRobotSpeedMode::Request& req, dsr_msgs::GetRobotSpeedMode::Response& res);
        bool get_current_pose_cb(dsr_msgs::GetCurrentPose::Request& req, dsr_msgs::GetCurrentPose::Response& res);
        bool set_safe_stop_reset_type_cb(dsr_msgs::SetSafeStopResetType::Request& req, dsr_msgs::SetSafeStopResetType::Response& res);
        bool get_last_alarm_cb(dsr_msgs::GetLastAlarm::Request& req, dsr_msgs::GetLastAlarm::Response& res);
    
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
        bool jog_cb(dsr_msgs::Jog::Request& req, dsr_msgs::Jog::Response& res);
        bool jog_multi_cb(dsr_msgs::JogMulti::Request& req, dsr_msgs::JogMulti::Response& res);
        bool move_stop_cb(dsr_msgs::MoveStop::Request& req, dsr_msgs::MoveStop::Response& res);
        bool move_resume_cb(dsr_msgs::MoveResume::Request& req, dsr_msgs::MoveResume::Response& res);
        bool move_pause_cb(dsr_msgs::MovePause::Request& req, dsr_msgs::MovePause::Response& res);
        bool trans_cb(dsr_msgs::Trans::Request& req, dsr_msgs::Trans::Response& res);
        bool fkin_cb(dsr_msgs::Fkin::Request& req, dsr_msgs::Fkin::Response& res);
        bool ikin_cb(dsr_msgs::Ikin::Request& req, dsr_msgs::Ikin::Response& res);
        bool set_ref_coord_cb(dsr_msgs::SetRefCoord::Request& req, dsr_msgs::SetRefCoord::Response& res);
        bool move_home_cb(dsr_msgs::MoveHome::Request& req, dsr_msgs::MoveHome::Response& res);
        bool check_motion_cb(dsr_msgs::CheckMotion::Request& req, dsr_msgs::CheckMotion::Response& res);
        bool change_operation_speed_cb(dsr_msgs::ChangeOperationSpeed::Request& req, dsr_msgs::ChangeOperationSpeed::Response& res);
        bool enable_alter_motion_cb(dsr_msgs::EnableAlterMotion::Request& req, dsr_msgs::EnableAlterMotion::Response& res);
        bool alter_motion_cb(dsr_msgs::AlterMotion::Request& req, dsr_msgs::AlterMotion::Response& res);
        bool disable_alter_motion_cb(dsr_msgs::DisableAlterMotion::Request& req, dsr_msgs::DisableAlterMotion::Response& res);
        bool set_singularity_handling_cb(dsr_msgs::SetSingularityHandling::Request& req, dsr_msgs::SetSingularityHandling::Response& res);

        //----- auxiliary_control
        bool get_control_mode_cb(dsr_msgs::GetControlMode::Request& req, dsr_msgs::GetControlMode::Response& res);               
        bool get_control_space_cb(dsr_msgs::GetControlSpace::Request& req, dsr_msgs::GetControlSpace::Response& res);              

        bool get_current_posj_cb(dsr_msgs::GetCurrentPosj::Request& req, dsr_msgs::GetCurrentPosj::Response& res);               
        bool get_current_velj_cb(dsr_msgs::GetCurrentVelj::Request& req, dsr_msgs::GetCurrentVelj::Response& res);               
        bool get_desired_posj_cb(dsr_msgs::GetDesiredPosj::Request& req, dsr_msgs::GetDesiredPosj::Response& res);
        bool get_desired_velj_cb(dsr_msgs::GetDesiredVelj::Request& req, dsr_msgs::GetDesiredVelj::Response& res);              

        bool get_current_posx_cb(dsr_msgs::GetCurrentPosx::Request& req, dsr_msgs::GetCurrentPosx::Response& res);               
        bool get_current_velx_cb(dsr_msgs::GetCurrentVelx::Request& req, dsr_msgs::GetCurrentVelx::Response& res);               
        bool get_desired_posx_cb(dsr_msgs::GetDesiredPosx::Request& req, dsr_msgs::GetDesiredPosx::Response& res);
        bool get_desired_velx_cb(dsr_msgs::GetDesiredVelx::Request& req, dsr_msgs::GetDesiredVelx::Response& res);               

        bool get_current_tool_flange_posx_cb(dsr_msgs::GetCurrentToolFlangePosx::Request& req, dsr_msgs::GetCurrentToolFlangePosx::Response& res);                                                          
        bool get_current_solution_space_cb(dsr_msgs::GetCurrentSolutionSpace::Request& req, dsr_msgs::GetCurrentSolutionSpace::Response& res);     
        bool get_current_rotm_cb(dsr_msgs::GetCurrentRotm::Request& req, dsr_msgs::GetCurrentRotm::Response& res);               
        bool get_joint_torque_cb(dsr_msgs::GetJointTorque::Request& req, dsr_msgs::GetJointTorque::Response& res);               
        bool get_external_torque_cb(dsr_msgs::GetExternalTorque::Request& req, dsr_msgs::GetExternalTorque::Response& res);           
        bool get_tool_force_cb(dsr_msgs::GetToolForce::Request& req, dsr_msgs::GetToolForce::Response& res);                 
        bool get_solution_space_cb(dsr_msgs::GetSolutionSpace::Request& req, dsr_msgs::GetSolutionSpace::Response& res);
        bool get_orientation_error_cb(dsr_msgs::GetOrientationError::Request& req, dsr_msgs::GetOrientationError::Response& res);

        //----- force/stiffness
        bool parallel_axis1_cb(dsr_msgs::ParallelAxis1::Request& req, dsr_msgs::ParallelAxis1::Response& res);
        bool parallel_axis2_cb(dsr_msgs::ParallelAxis2::Request& req, dsr_msgs::ParallelAxis2::Response& res);
        bool align_axis1_cb(dsr_msgs::AlignAxis1::Request& req, dsr_msgs::AlignAxis1::Response& res);
        bool align_axis2_cb(dsr_msgs::AlignAxis2::Request& req, dsr_msgs::AlignAxis2::Response& res);
        bool is_done_bolt_tightening_cb(dsr_msgs::IsDoneBoltTightening::Request& req, dsr_msgs::IsDoneBoltTightening::Response& res);
        bool release_compliance_ctrl_cb(dsr_msgs::ReleaseComplianceCtrl::Request& req, dsr_msgs::ReleaseComplianceCtrl::Response& res);
        bool task_compliance_ctrl_cb(dsr_msgs::TaskComplianceCtrl::Request& req, dsr_msgs::TaskComplianceCtrl::Response& res);
        bool set_stiffnessx_cb(dsr_msgs::SetStiffnessx::Request& req, dsr_msgs::SetStiffnessx::Response& res);
        bool calc_coord_cb(dsr_msgs::CalcCoord::Request& req, dsr_msgs::CalcCoord::Response& res);
        bool set_user_cart_coord1_cb(dsr_msgs::SetUserCartCoord1::Request& req, dsr_msgs::SetUserCartCoord1::Response& res);
        bool set_user_cart_coord2_cb(dsr_msgs::SetUserCartCoord2::Request& req, dsr_msgs::SetUserCartCoord2::Response& res);
        bool set_user_cart_coord3_cb(dsr_msgs::SetUserCartCoord3::Request& req, dsr_msgs::SetUserCartCoord3::Response& res);
        bool overwrite_user_cart_coord_cb(dsr_msgs::OverwriteUserCartCoord::Request& req, dsr_msgs::OverwriteUserCartCoord::Response& res);
        bool get_user_cart_coord_cb(dsr_msgs::GetUserCartCoord::Request& req, dsr_msgs::GetUserCartCoord::Response& res);
        bool set_desired_force_cb(dsr_msgs::SetDesiredForce::Request& req, dsr_msgs::SetDesiredForce::Response& res);
        bool release_force_cb(dsr_msgs::ReleaseForce::Request& req, dsr_msgs::ReleaseForce::Response& res);
        bool check_position_condition_cb(dsr_msgs::CheckPositionCondition::Request& req, dsr_msgs::CheckPositionCondition::Response& res);
        bool check_force_condition_cb(dsr_msgs::CheckForceCondition::Request& req, dsr_msgs::CheckForceCondition::Response& res);
        bool check_orientation_condition1_cb(dsr_msgs::CheckOrientationCondition1::Request& req, dsr_msgs::CheckOrientationCondition1::Response& res);
        bool check_orientation_condition2_cb(dsr_msgs::CheckOrientationCondition2::Request& req, dsr_msgs::CheckOrientationCondition2::Response& res);
        bool coord_transform_cb(dsr_msgs::CoordTransform::Request& req, dsr_msgs::CoordTransform::Response& res);
        bool get_workpiece_weight_cb(dsr_msgs::GetWorkpieceWeight::Request& req, dsr_msgs::GetWorkpieceWeight::Response& res);
        bool reset_workpiece_weight_cb(dsr_msgs::ResetWorkpieceWeight::Request& req, dsr_msgs::ResetWorkpieceWeight::Response& res);

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
        bool set_tool_shape_cb(dsr_msgs::SetToolShape::Request& req, dsr_msgs::SetToolShape::Response& res);

        //----- IO
        bool set_digital_output_cb(dsr_msgs::SetCtrlBoxDigitalOutput::Request& req, dsr_msgs::SetCtrlBoxDigitalOutput::Response& res);
        bool get_digital_output_cb(dsr_msgs::GetCtrlBoxDigitalOutput::Request& req, dsr_msgs::GetCtrlBoxDigitalOutput::Response& res);
        bool get_digital_input_cb(dsr_msgs::GetCtrlBoxDigitalInput::Request& req, dsr_msgs::GetCtrlBoxDigitalInput::Response& res);
        bool set_tool_digital_output_cb(dsr_msgs::SetToolDigitalOutput::Request& req, dsr_msgs::SetToolDigitalOutput::Response& res);
        bool get_tool_digital_output_cb(dsr_msgs::GetToolDigitalOutput::Request& req, dsr_msgs::GetToolDigitalOutput::Response& res);
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
        bool get_drl_state_cb(dsr_msgs::GetDrlState::Request& req, dsr_msgs::GetDrlState::Response& res);

        //----- Gripper
        bool robotiq_2f_open_cb(dsr_msgs::Robotiq2FOpen::Request& req, dsr_msgs::Robotiq2FOpen::Response& res);
        bool robotiq_2f_close_cb(dsr_msgs::Robotiq2FClose::Request& req, dsr_msgs::Robotiq2FClose::Response& res);
        bool robotiq_2f_move_cb(dsr_msgs::Robotiq2FMove::Request& req, dsr_msgs::Robotiq2FMove::Response& res);

        //----- Serial
        bool serial_send_data_cb(dsr_msgs::SerialSendData::Request& req, dsr_msgs::SerialSendData::Response& res);

    };
}

#endif // end