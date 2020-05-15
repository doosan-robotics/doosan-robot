/*********************************************************************
 *
 * class of doosan robot control 
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

#ifndef __DSR_ROBOT_H__
#define __DSR_ROBOT_H__

#include <ros/ros.h>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>

#include <dsr_msgs/RobotError.h>
#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/RobotStop.h>
#include <dsr_msgs/ModbusState.h>

#include <dsr_msgs/SetRobotMode.h>
#include <dsr_msgs/GetRobotMode.h>
#include <dsr_msgs/SetRobotSystem.h>
#include <dsr_msgs/GetRobotSystem.h>
#include <dsr_msgs/SetRobotSpeedMode.h>
#include <dsr_msgs/GetRobotSpeedMode.h>
#include <dsr_msgs/SetSafeStopResetType.h>
#include <dsr_msgs/GetCurrentPose.h>
//#include <dsr_msgs/GetLastAlarm.h>

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


#include <dsr_msgs/ConfigCreateTcp.h>
#include <dsr_msgs/ConfigDeleteTcp.h>
#include <dsr_msgs/GetCurrentTcp.h>
#include <dsr_msgs/SetCurrentTcp.h>

#include <dsr_msgs/SetCurrentTool.h>
#include <dsr_msgs/GetCurrentTool.h>
#include <dsr_msgs/ConfigCreateTool.h>
#include <dsr_msgs/ConfigDeleteTool.h>
#include <dsr_msgs/SetToolShape.h>

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

#include <dsr_msgs/SetModbusOutput.h>
#include <dsr_msgs/GetModbusInput.h>
#include <dsr_msgs/ConfigCreateModbus.h>
#include <dsr_msgs/ConfigDeleteModbus.h>

#include <dsr_msgs/DrlPause.h>
#include <dsr_msgs/DrlStart.h>
#include <dsr_msgs/DrlStop.h>
#include <dsr_msgs/DrlResume.h>
#include <dsr_msgs/GetDrlState.h>

#include <dsr_msgs/Robotiq2FOpen.h>
#include <dsr_msgs/Robotiq2FClose.h>
#include <dsr_msgs/Robotiq2FMove.h>
#include <dsr_msgs/SerialSendData.h>

#include "DRFL.h"
#include "DRFC.h"
#include "DRFS.h"

using namespace std;

namespace DSR_Robot{
    class CDsrRobot
    {
        public:
            CDsrRobot(ros::NodeHandle nh, std::string robotID="dsr01", std::string robotModel="m1013");
            virtual ~CDsrRobot();

            int stop(int nMode = STOP_TYPE_QUICK);
            //----- system
            int set_robot_mode(int robot_mode = ROBOT_MODE_MANUAL);
            int get_robot_mode();
            int set_robot_system(int robot_system = ROBOT_SYSTEM_VIRTUAL);
            int get_robot_system();
            int set_robot_speed_mode(int speed_mode = SPEED_NORMAL_MODE);
            int get_robot_speed_mode();
            int set_safe_stop_reset_type(int reset_type = SAFE_STOP_RESET_TYPE_DEFAULT);
            int get_current_pose(int space_type = ROBOT_SPACE_JOINT);
            int get_current_solution_space();
            //int get_last_alarm();
            //----- sync motion
            int movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, float fBlendingRadius = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0);

            int movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);

            int movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f,
                       int nMoveMode = MOVE_MODE_ABSOLUTE);

            int movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);

            int moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE);

            int move_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                            int nTaskAxis = TASK_AXIS_Z, int nMoveReference = MOVE_REFERENCE_TOOL);

            int move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, 
                              int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL);
 
            int jog(int jog_axis, int move_reference = MOVE_REFERENCE_BASE, int speed = 10);

            int jog_multi(float jog_axis[NUM_TASK], int move_reference = MOVE_REFERENCE_BASE, int speed = 10);

            //----- async motion
            int amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int amovel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int amovejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, float fBlendingRadius = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0);

            int amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);

            int amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f,
                       int nMoveMode = MOVE_MODE_ABSOLUTE);

            int amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);

            int amoveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE);

            int amove_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                            int nTaskAxis = TASK_AXIS_Z, int nMoveReference = MOVE_REFERENCE_TOOL);

            int amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, 
                              int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL);

            //----- Motion Wait
            int move_wait();

            //----- TCP
            int config_create_tcp(string strName, float fTargetPos[NUM_TASK]);
            int config_delete_tcp(string strName);
            int set_current_tcp(string strName);
            string get_current_tcp();

            //----- TOOL
            int config_create_tool(string strName, float fTargetWeight, float fTargetCog[3], float fTargetInertia[NUM_TASK]);
            int config_delete_tool(string strName);
            int set_current_tool(string strName);
            string get_current_tool();

            //----- IO
            int set_digital_output(int nGpioIndex, bool bGpioValue);
            int get_digital_input(int nGpioIndex);
            int set_tool_digital_output(int nGpioIndex, bool bGpioValue);
            int get_tool_digital_input(int nGpioIndex);
            int set_analog_output(int nGpioChannel, float fGpioValue);
            int get_analog_input(int nGpioChannel);
            int set_analog_output_type(int nGpioChannel, int nGpioMode);
            int set_analog_input_type(int nGpioChannel, int nGpioMode);

            //----- MODBUS
            int config_create_modbus(string strName, string strIP, int nPort, int nRegType, int nRegIndex, int nRegValue = 0, int nSlaveID =255);
            int config_delete_modbus(string strName);
            int set_modbus_output(string strName, int nValue);
            int get_modbus_input(string strName);

            //----- DRL        
            int drl_start(int nRobotSystem, string strCode);
            int drl_stop(int nStopMode = STOP_TYPE_QUICK);
            int drl_pause();
            int drl_resume();
            int get_drl_state();

        private:
            int _movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType); 
            int _movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);             
            int _movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);
            int _movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSolSpace, int nSyncType);
            int _move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, int nRepeat, int nMoveReference, int nSyncType);
            int _move_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nTaskAxis, int nMoveReference, int nSyncType);
            int _movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime, int nMoveMode, int nSyncType);
            int _movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveReference, int nMoveMode, int nVelOpt, int nSyncType);
            int _moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveReference, int nMoveMode, int nSyncType);
            
            //void thread_subscriber();
            //void msgRobotState_cb(const dsr_msgs::RobotState::ConstPtr& msg);
            ///boost::thread m_thread_sub;

            std::string m_strSrvNamePrefix;
            std::string m_strTopicNamePrefix; 
    };
}
#endif // end