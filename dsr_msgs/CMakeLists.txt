cmake_minimum_required(VERSION 2.8.3)
project(dsr_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
    FILES
    RobotError.msg
    RobotState.msg
    RobotStop.msg
)

add_service_files(
    FILES
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

    ./tool/ConfigCreateTool.srv
    ./tool/ConfigDeleteTool.srv
    ./tool/SetCurrentTool.srv
    ./tool/GetCurrentTool.srv

    ./tcp/ConfigCreateTcp.srv
    ./tcp/ConfigDeleteTcp.srv
    ./tcp/SetCurrentTcp.srv
    ./tcp/GetCurrentTcp.srv

    ./io/SetToolDigitalOutput.srv
    ./io/GetToolDigitalInput.srv
    ./io/SetCtrlBoxDigitalOutput.srv
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

    ./gripper/Robotiq2FClose.srv
    ./gripper/Robotiq2FOpen.srv
    ./gripper/Robotiq2FMove.srv
    ./gripper/SerialClose.srv
    ./gripper/SerialOpen.srv
    ./gripper/SerialSendData.srv

)

generate_messages(
    DEPENDENCIES
    std_msgs
) 

catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime
)
