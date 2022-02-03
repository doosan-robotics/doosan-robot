cmake_minimum_required(VERSION 2.8.3)
project(dsr_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
    FILES
    LogAlarm.msg
    ModbusState.msg
    RobotError.msg
    RobotState.msg
    RobotStop.msg
    JogMultiAxis.msg
    AlterMotionStream.msg
    ServoJStream.msg
    ServoLStream.msg
    SpeedJStream.msg
    SpeedLStream.msg

    RobotStateRT.msg
    ServoJRTStream.msg
    ServoLRTStream.msg
    SpeedJRTStream.msg
    SpeedLRTStream.msg
    TorqueRTStream.msg
)

add_service_files(
    FILES

    ./system/SetRobotMode.srv
    ./system/GetRobotMode.srv
    ./system/SetRobotSystem.srv
    ./system/GetRobotSystem.srv
    ./system/SetRobotSpeedMode.srv
    ./system/GetRobotSpeedMode.srv
    ./system/GetCurrentPose.srv
    ./system/SetSafeStopResetType.srv
    ./system/GetLastAlarm.srv
    ./system/GetRobotState.srv
    ./system/SetRobotControl.srv
    ./system/ManageAccessControl.srv
    ./system/ReleaseProtectiveStop.srv

    ./motion/MoveJoint.srv
    ./motion/MoveLine.srv
    ./motion/MoveJointx.srv
    ./motion/MoveCircle.srv
    ./motion/MoveSplineJoint.srv
    ./motion/MoveSplineTask.srv
    ./motion/MoveBlending.srv
    ./motion/MoveSpiral.srv
    ./motion/MovePeriodic.srv
    ./motion/MoveWait.srv
    ./motion/Jog.srv
    ./motion/JogMulti.srv
    ./motion/MoveStop.srv
    ./motion/MovePause.srv
    ./motion/MoveResume.srv
    ./motion/Trans.srv
    ./motion/Fkin.srv
    ./motion/Ikin.srv
    ./motion/IkinEx.srv
    ./motion/SetRefCoord.srv
    ./motion/MoveHome.srv
    ./motion/CheckMotion.srv
    ./motion/ChangeOperationSpeed.srv
    ./motion/EnableAlterMotion.srv
    ./motion/AlterMotion.srv
    ./motion/DisableAlterMotion.srv
    ./motion/SetSingularityHandling.srv

    ./aux_control/GetControlMode.srv              
    ./aux_control/GetControlSpace.srv             
    ./aux_control/GetCurrentPosj.srv              
    ./aux_control/GetDesiredPosj.srv
    ./aux_control/GetCurrentVelj.srv              
    ./aux_control/GetDesiredVelj.srv              
    ./aux_control/GetCurrentPosx.srv              
    ./aux_control/GetCurrentToolFlangePosx.srv    
    ./aux_control/GetCurrentVelx.srv              
    ./aux_control/GetDesiredPosx.srv
    ./aux_control/GetDesiredVelx.srv              
    ./aux_control/GetCurrentSolutionSpace.srv     
    ./aux_control/GetCurrentRotm.srv              
    ./aux_control/GetJointTorque.srv              
    ./aux_control/GetExternalTorque.srv          
    ./aux_control/GetToolForce.srv                
    ./aux_control/GetSolutionSpace.srv
    ./aux_control/GetOrientationError.srv

    ./force/ParallelAxis1.srv
    ./force/ParallelAxis2.srv
    ./force/AlignAxis1.srv
    ./force/AlignAxis2.srv
    ./force/IsDoneBoltTightening.srv
    ./force/ReleaseComplianceCtrl.srv
    ./force/TaskComplianceCtrl.srv
    ./force/SetStiffnessx.srv
    ./force/CalcCoord.srv
    ./force/SetUserCartCoord1.srv
    ./force/SetUserCartCoord2.srv
    ./force/SetUserCartCoord3.srv
    ./force/OverwriteUserCartCoord.srv
    ./force/GetUserCartCoord.srv
    ./force/SetDesiredForce.srv
    ./force/ReleaseForce.srv
    ./force/CheckPositionCondition.srv
    ./force/CheckForceCondition.srv
    ./force/CheckOrientationCondition1.srv
    ./force/CheckOrientationCondition2.srv
    ./force/CoordTransform.srv
    ./force/GetWorkpieceWeight.srv
    ./force/ResetWorkpieceWeight.srv
    
    ./tool/ConfigCreateTool.srv
    ./tool/ConfigDeleteTool.srv
    ./tool/SetCurrentTool.srv
    ./tool/GetCurrentTool.srv
    ./tool/SetToolShape.srv

    ./tcp/ConfigCreateTcp.srv
    ./tcp/ConfigDeleteTcp.srv
    ./tcp/SetCurrentTcp.srv
    ./tcp/GetCurrentTcp.srv

    ./io/SetToolDigitalOutput.srv
    ./io/GetToolDigitalOutput.srv
    ./io/GetToolDigitalInput.srv
    ./io/SetCtrlBoxDigitalOutput.srv
    ./io/GetCtrlBoxDigitalOutput.srv  
    ./io/GetCtrlBoxDigitalInput.srv
    ./io/SetCtrlBoxAnalogInputType.srv
    ./io/SetCtrlBoxAnalogOutputType.srv
    ./io/SetCtrlBoxAnalogOutput.srv
    ./io/GetCtrlBoxAnalogInput.srv
    
    ./modbus/ConfigCreateModbus.srv
    ./modbus/ConfigDeleteModbus.srv
    ./modbus/SetModbusOutput.srv
    ./modbus/GetModbusInput.srv

    ./drl/DrlStart.srv
    ./drl/DrlStop.srv
    ./drl/DrlPause.srv
    ./drl/DrlResume.srv
    ./drl/GetDrlState.srv

    ./gripper/Robotiq2FClose.srv
    ./gripper/Robotiq2FOpen.srv
    ./gripper/Robotiq2FMove.srv
    ./gripper/SerialSendData.srv

    ./realtime/ConnectRTControl.srv
    ./realtime/DisconnectRTControl.srv
    ./realtime/GetRTControlInputDataList.srv
    ./realtime/GetRTControlInputVersionList.srv
    ./realtime/GetRTControlOutputDataList.srv
    ./realtime/GetRTControlOutputVersionList.srv
    ./realtime/ReadDataRT.srv
    ./realtime/SetAccJRT.srv
    ./realtime/SetAccXRT.srv
    ./realtime/SetRTControlInput.srv
    ./realtime/SetRTControlOutput.srv
    ./realtime/SetVelJRT.srv
    ./realtime/SetVelXRT.srv
    ./realtime/StartRTControl.srv
    ./realtime/StopRTControl.srv
    ./realtime/WriteDataRT.srv

)

generate_messages(
    DEPENDENCIES
    std_msgs
) 

catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime
)
