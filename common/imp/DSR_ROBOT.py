#-*- coding: utf-8 -*-

# ##
# @mainpage
# @file     DSR_ROBOT.py
# @brief    Doosan Robotics ROS service I/F module
# @author   kabdol2<kabkyoum.kim@doosan.com>   
# @version  0.21
# @Last update date     2021-04-06
# @details
#
# history
#  - support multiple robots  
#
__ROS__ = True

import rospy
import os
import threading, time
from std_msgs.msg import String,Int32,Int32MultiArray,Float32,Float64,Float32MultiArray,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import sys
sys.dont_write_bytecode = True

#import numpy as np

from DRFC import *
from DR_common import *

from dsr_msgs.msg import *
from dsr_msgs.srv import *

from math import *

import DR_init
#print("robot_id ={0}".format(DR_init.__dsr__id) )
#print("robot_model ={0}".format(DR_init.__dsr__model) )

_robot_id    = DR_init.__dsr__id
_robot_model = DR_init.__dsr__model
_srv_name_prefix   = '/' + _robot_id + _robot_model  
_topic_name_prefix = _srv_name_prefix

print("_robot_id ={0}".format(_robot_id))
print("_robot_model ={0}".format(_robot_model))
print("_srv_name_prefix ={0}".format(_srv_name_prefix))
print("_topic_name_prefix ={0}".format(_topic_name_prefix))

############### connect to dsr_control (ros service) ####################################################################### 
####rospy.wait_for_service(_srv_name_prefix +"/motion/move_joint")

#  system Operations
_ros_set_robot_mode             = rospy.ServiceProxy(_srv_name_prefix +"/system/set_robot_mode", SetRobotMode)
_ros_get_robot_mode             = rospy.ServiceProxy(_srv_name_prefix +"/system/get_robot_mode", GetRobotMode)
_ros_set_robot_system           = rospy.ServiceProxy(_srv_name_prefix +"/system/set_robot_system", SetRobotSystem)
_ros_get_robot_system           = rospy.ServiceProxy(_srv_name_prefix +"/system/get_robot_system", GetRobotSystem)
_ros_get_robot_state            = rospy.ServiceProxy(_srv_name_prefix +"/system/get_robot_state", GetRobotState)
_ros_set_robot_speed_mode       = rospy.ServiceProxy(_srv_name_prefix +"/system/set_robot_speed_mode", SetRobotSpeedMode)
_ros_get_robot_speed_mode       = rospy.ServiceProxy(_srv_name_prefix +"/system/get_robot_speed_mode", GetRobotSpeedMode)
_ros_set_safe_stop_reset_type   = rospy.ServiceProxy(_srv_name_prefix +"/system/set_safe_stop_reset_type", SetSafeStopResetType)
_ros_get_last_alarm             = rospy.ServiceProxy(_srv_name_prefix +"/system/get_last_alarm", GetLastAlarm)
_ros_get_current_pose           = rospy.ServiceProxy(_srv_name_prefix +"/system/get_current_pose", GetCurrentPose)
_ros_set_robot_control          = rospy.ServiceProxy(_srv_name_prefix +"/system/set_robot_control", SetRobotControl)
_ros_manage_access_control      = rospy.ServiceProxy(_srv_name_prefix +"/system/manage_access_control", ManageAccessControl)

#  motion Operations
_ros_movej                      = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_joint", MoveJoint)
_ros_movel                      = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_line", MoveLine)
_ros_movejx                     = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_jointx", MoveJointx)
_ros_movec                      = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_circle", MoveCircle)
_ros_movesj                     = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_spline_joint", MoveSplineJoint)
_ros_movesx                     = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_spline_task", MoveSplineTask)
_ros_moveb                      = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_blending", MoveBlending)
_ros_move_spiral                = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_spiral", MoveSpiral)
_ros_move_periodic              = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_periodic", MovePeriodic)
_ros_move_wait                  = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_wait", MoveWait)
_ros_jog                        = rospy.ServiceProxy(_srv_name_prefix +"/motion/jog", Jog)
_ros_jog_multi                  = rospy.ServiceProxy(_srv_name_prefix +"/motion/jog_multi", JogMulti)
_ros_trans                      = rospy.ServiceProxy(_srv_name_prefix +"/motion/trans", Trans)
_ros_fkin                       = rospy.ServiceProxy(_srv_name_prefix +"/motion/fkin", Fkin)
_ros_ikin                       = rospy.ServiceProxy(_srv_name_prefix +"/motion/ikin", Ikin)
_ros_set_ref_coord              = rospy.ServiceProxy(_srv_name_prefix +"/motion/set_ref_coord", SetRefCoord)
_ros_move_home                  = rospy.ServiceProxy(_srv_name_prefix +"/motion/move_home", MoveHome)
_ros_check_motion               = rospy.ServiceProxy(_srv_name_prefix +"/motion/check_motion", CheckMotion)
_ros_change_operation_speed     = rospy.ServiceProxy(_srv_name_prefix +"/motion/change_operation_speed", ChangeOperationSpeed)
_ros_enable_alter_motion        = rospy.ServiceProxy(_srv_name_prefix +"/motion/enable_alter_motion", EnableAlterMotion)
_ros_alter_motion               = rospy.ServiceProxy(_srv_name_prefix +"/motion/alter_motion", AlterMotion)
_ros_disable_alter_motion       = rospy.ServiceProxy(_srv_name_prefix +"/motion/disable_alter_motion", DisableAlterMotion)


# Auxiliary Control Operations
_ros_get_control_mode               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_control_mode", GetControlMode)
_ros_get_control_space              = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_control_space", GetControlSpace)

_ros_get_current_posj               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_current_posj", GetCurrentPosj)
_ros_get_current_velj               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_current_velj", GetCurrentVelj)
_ros_get_desired_posj               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_desired_posj", GetDesiredPosj)
_ros_get_desired_velj               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_desired_velj", GetDesiredVelj)

_ros_get_current_posx               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_current_posx", GetCurrentPosx)
_ros_get_current_velx               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_current_velx", GetCurrentVelx)    
_ros_get_desired_posx               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_desired_posx", GetDesiredPosx)
_ros_get_desired_velx               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_desired_velx", GetDesiredVelx)

_ros_get_current_tool_flange_posx   = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_current_tool_flange_posx", GetCurrentToolFlangePosx)

_ros_get_current_solution_space     = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_current_solution_space", GetCurrentSolutionSpace)
_ros_get_current_rotm               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_current_rotm", GetCurrentRotm)    
_ros_get_joint_torque               = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_joint_torque", GetJointTorque)
_ros_get_external_torque            = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_external_torque", GetExternalTorque)
_ros_get_tool_force                 = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_tool_force", GetToolForce)
_ros_get_solution_space             = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_solution_space", GetSolutionSpace)
_ros_get_orientation_error          = rospy.ServiceProxy(_srv_name_prefix +"/aux_control/get_orientation_error", GetOrientationError)

# Force/Stiffness Control & others Operations
_ros_get_workpiece_weight        = rospy.ServiceProxy(_srv_name_prefix +"/force/get_workpiece_weight", GetWorkpieceWeight)
_ros_reset_workpiece_weight      = rospy.ServiceProxy(_srv_name_prefix +"/force/reset_workpiece_weight", ResetWorkpieceWeight)
_ros_set_singularity_handling    = rospy.ServiceProxy(_srv_name_prefix +"/motion/set_singularity_handling", SetSingularityHandling)

_ros_parallel_axis1              = rospy.ServiceProxy(_srv_name_prefix +"/force/parallel_axis1", ParallelAxis1)
_ros_parallel_axis2              = rospy.ServiceProxy(_srv_name_prefix +"/force/parallel_axis2", ParallelAxis2)
_ros_align_axis1                 = rospy.ServiceProxy(_srv_name_prefix +"/force/align_axis1", AlignAxis1)
_ros_align_axis2                 = rospy.ServiceProxy(_srv_name_prefix +"/force/align_axis2", AlignAxis2)
_ros_is_done_bolt_tightening     = rospy.ServiceProxy(_srv_name_prefix +"/force/is_done_bolt_tightening", IsDoneBoltTightening)
_ros_release_compliance_ctrl     = rospy.ServiceProxy(_srv_name_prefix +"/force/release_compliance_ctrl", ReleaseComplianceCtrl)
_ros_task_compliance_ctrl        = rospy.ServiceProxy(_srv_name_prefix +"/force/task_compliance_ctrl", TaskComplianceCtrl)
_ros_set_stiffnessx              = rospy.ServiceProxy(_srv_name_prefix +"/force/set_stiffnessx", SetStiffnessx)
_ros_calc_coord                  = rospy.ServiceProxy(_srv_name_prefix +"/force/calc_coord", CalcCoord)
_ros_set_user_cart_coord1        = rospy.ServiceProxy(_srv_name_prefix +"/force/set_user_cart_coord1", SetUserCartCoord1)
_ros_set_user_cart_coord2        = rospy.ServiceProxy(_srv_name_prefix +"/force/set_user_cart_coord2", SetUserCartCoord2)
_ros_set_user_cart_coord3        = rospy.ServiceProxy(_srv_name_prefix +"/force/set_user_cart_coord3", SetUserCartCoord3)
_ros_overwrite_user_cart_coord   = rospy.ServiceProxy(_srv_name_prefix +"/force/overwrite_user_cart_coord", OverwriteUserCartCoord)
_ros_get_user_cart_coord         = rospy.ServiceProxy(_srv_name_prefix +"/force/get_user_cart_coord", GetUserCartCoord)
_ros_set_desired_force           = rospy.ServiceProxy(_srv_name_prefix +"/force/set_desired_force", SetDesiredForce)
_ros_release_force               = rospy.ServiceProxy(_srv_name_prefix +"/force/release_force", ReleaseForce)
_ros_check_position_condition    = rospy.ServiceProxy(_srv_name_prefix +"/force/check_position_condition", CheckPositionCondition)
_ros_check_force_condition       = rospy.ServiceProxy(_srv_name_prefix +"/force/check_force_condition", CheckForceCondition)
_ros_check_orientation_condition1= rospy.ServiceProxy(_srv_name_prefix +"/force/check_orientation_condition1", CheckOrientationCondition1)
_ros_check_orientation_condition2= rospy.ServiceProxy(_srv_name_prefix +"/force/check_orientation_condition2", CheckOrientationCondition2)
_ros_coord_transform             = rospy.ServiceProxy(_srv_name_prefix +"/force/coord_transform", CoordTransform)

#  GPIO Operations
_ros_set_digital_output         = rospy.ServiceProxy(_srv_name_prefix +"/io/set_digital_output", SetCtrlBoxDigitalOutput)
_ros_get_digital_input          = rospy.ServiceProxy(_srv_name_prefix +"/io/get_digital_input", GetCtrlBoxDigitalInput)
_ros_set_tool_digital_output    = rospy.ServiceProxy(_srv_name_prefix +"/io/set_tool_digital_output", SetToolDigitalOutput)
_ros_get_tool_digital_input     = rospy.ServiceProxy(_srv_name_prefix +"/io/get_tool_digital_input", GetToolDigitalInput)
_ros_set_analog_output          = rospy.ServiceProxy(_srv_name_prefix +"/io/set_analog_output", SetCtrlBoxAnalogOutput)
_ros_get_analog_input           = rospy.ServiceProxy(_srv_name_prefix +"/io/get_analog_input", GetCtrlBoxAnalogInput)
_ros_set_mode_analog_output     = rospy.ServiceProxy(_srv_name_prefix +"/io/set_analog_output_type", SetCtrlBoxAnalogOutputType)
_ros_set_mode_analog_input      = rospy.ServiceProxy(_srv_name_prefix +"/io/set_analog_input_type", SetCtrlBoxAnalogInputType)
_ros_get_digital_output         = rospy.ServiceProxy(_srv_name_prefix +"/io/get_digital_output", GetCtrlBoxDigitalOutput)
_ros_get_tool_digital_output    = rospy.ServiceProxy(_srv_name_prefix +"/io/get_tool_digital_output", GetToolDigitalOutput)

#  Modbus Operations
_ros_set_modbus_output          = rospy.ServiceProxy(_srv_name_prefix +"/modbus/set_modbus_output", SetModbusOutput)
_ros_get_modbus_input           = rospy.ServiceProxy(_srv_name_prefix +"/modbus/get_modbus_input", GetModbusInput)
_ros_add_modbus_signal          = rospy.ServiceProxy(_srv_name_prefix +"/modbus/config_create_modbus", ConfigCreateModbus)
_ros_del_modbus_signal          = rospy.ServiceProxy(_srv_name_prefix +"/modbus/config_delete_modbus", ConfigDeleteModbus)

# TCP Operations
_ros_set_current_tcp            = rospy.ServiceProxy(_srv_name_prefix +"/tcp/set_current_tcp", SetCurrentTcp)        #new
_ros_get_current_tcp            = rospy.ServiceProxy(_srv_name_prefix +"/tcp/get_current_tcp", GetCurrentTcp)        
_ros_config_create_tcp          = rospy.ServiceProxy(_srv_name_prefix +"/tcp/config_create_tcp", ConfigCreateTcp)    #new
_ros_config_delete_tcp          = rospy.ServiceProxy(_srv_name_prefix +"/tcp/config_delete_tcp", ConfigDeleteTcp)    #new

# Tool Operations
_ros_set_current_tool           = rospy.ServiceProxy(_srv_name_prefix +"/tool/set_current_tool", SetCurrentTool)     #new
_ros_get_current_tool           = rospy.ServiceProxy(_srv_name_prefix +"/tool/get_current_tool", GetCurrentTool)      
_ros_config_create_tool         = rospy.ServiceProxy(_srv_name_prefix +"/tool/config_create_tool", ConfigCreateTool) #new
_ros_config_delete_tool         = rospy.ServiceProxy(_srv_name_prefix +"/tool/config_delete_tool", ConfigDeleteTool) #new
_ros_set_tool_shape             = rospy.ServiceProxy(_srv_name_prefix +"/tool/set_tool_shape", SetToolShape)         #new

# DRL Operations
_ros_drl_pause                  = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_pause", DrlPause)                   #new
_ros_drl_resume                 = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_resume", DrlResume)                 #new
_ros_drl_start                  = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_start", DrlStart)                   #new
_ros_drl_stop                   = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_stop", DrlStop)                     #new
_ros_get_drl_state              = rospy.ServiceProxy(_srv_name_prefix +"/drl/get_drl_state", GetDrlState)

########################################################################################################################################


# point count
POINT_COUNT = 6

# solution space
DR_SOL_MIN = 0
DR_SOL_MAX = 7

# posb seg_type
DR_LINE   = 0
DR_CIRCLE = 1

# move reference
DR_BASE        = 0
DR_TOOL        = 1
DR_WORLD       = 2
DR_TC_USER_MIN = 101
DR_TC_USER_MAX = 200

# move mod
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# move reaction
DR_MV_RA_NONE      = 0
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE  = 1

# move command type
DR_MV_COMMAND_NORM = 0

# movesx velocity
DR_MVS_VEL_NONE    = 0
DR_MVS_VEL_CONST   = 1

# motion state
DR_STATE_IDLE   = 0
DR_STATE_INIT   = 1
DR_STATE_BUSY   = 2
DR_STATE_BLEND  = 3
DR_STATE_ACC    = 4
DR_STATE_CRZ    = 5
DR_STATE_DEC    = 6

# axis
DR_AXIS_X = 0
DR_AXIS_Y = 1
DR_AXIS_Z = 2
DR_AXIS_A = 10
DR_AXIS_B = 11
DR_AXIS_C = 12

# collision sensitivity
DR_COLSENS_DEFAULT = 20
DR_COLSENS_MIN = 1   #10   2017/11/08 변경 
DR_COLSENS_MAX = 300 #100  2017/11/08 변경

# speed
DR_OP_SPEED_MIN = 1
DR_OP_SPEED_MAX = 100

# stop
DR_QSTOP_STO = 0
DR_QSTOP     = 1
DR_SSTOP     = 2
DR_HOLD      = 3

DR_STOP_FIRST = DR_QSTOP_STO
DR_STOP_LAST = DR_HOLD

# condition
DR_COND_NONE = -10000

# digital I/O
DR_DIO_MIN_INDEX = 1
DR_DIO_MAX_INDEX = 16   #8 16개로 확장됨 2017/08/18  

# tool digital I/O
DR_TDIO_MIN_INDEX = 1
DR_TDIO_MAX_INDEX = 6

# I/O value
ON = 1
OFF = 0

# Analog I/O mode
DR_ANALOG_CURRENT = 0
DR_ANALOG_VOLTAGE = 1

# modbus type
DR_MODBUS_DIG_INPUT  = 0
DR_MODBUS_DIG_OUTPUT = 1
DR_MODBUS_REG_INPUT  = 2
DR_MODBUS_REG_OUTPUT = 3
DR_DISCRETE_INPUT    = DR_MODBUS_DIG_INPUT
DR_COIL              = DR_MODBUS_DIG_OUTPUT
DR_INPUT_REGISTER    = DR_MODBUS_REG_INPUT
DR_HOLDING_REGISTER  = DR_MODBUS_REG_OUTPUT

DR_MODBUS_ACCESS_MAX    = 32
DR_MAX_MODBUS_NAME_SIZE = 32

# tp_popup pm_type
DR_PM_MESSAGE = 0
DR_PM_WARNING = 1
DR_PM_ALARM   = 2
DR_TP_POPUP_BUTTON_TYPE_STOP_RESUME = 0 # add 2019/04/01
DR_TP_POPUP_BUTTON_TYPE_STOP        = 1 # add 2019/04/01

# tp_get_user_input type
DR_VAR_INT   = 0
DR_VAR_FLOAT = 1
DR_VAR_STR   = 2
DR_VAR_BOOL  = 3 # add 2020/01/29

# len
DR_VELJ_DT_LEN = 6
DR_ACCJ_DT_LEN = 6

DR_VELX_DT_LEN = 2
DR_ACCX_DT_LEN = 2

DR_ANGLE_DT_LEN   = 2
DR_COG_DT_LEN     = 3
DR_WEIGHT_DT_LEN  = 3
DR_VECTOR_DT_LEN  = 3
DR_ST_DT_LEN      = 6
DR_FD_DT_LEN      = 6
DR_DIR_DT_LEN     = 6
DR_INERTIA_DT_LEN = 6
DR_VECTOR_U1_LEN  = 3
DR_VECTOR_V1_LEN  = 3

# set_singular_handling mode 
DR_AVOID     = 0
DR_TASK_STOP = 1
DR_VAR_VEL   = 2        #add 2019/04/01 

# object container type
DR_FIFO      = 0
DR_LIFO      = 1

# set_desired_force mod
DR_FC_MOD_ABS = 0
DR_FC_MOD_REL = 1

# global variable type
DR_GLOBAL_VAR_TYPE_BOOL   = 0
DR_GLOBAL_VAR_TYPE_INT    = 1
DR_GLOBAL_VAR_TYPE_FLOAT  = 2
DR_GLOBAL_VAR_TYPE_STR    = 3
DR_GLOBAL_VAR_TYPE_POSJ   = 4
DR_GLOBAL_VAR_TYPE_POSX   = 5
DR_GLOBAL_VAR_TYPE_UNKNOWN= 6

# Industrial Ethernet(EtherNet/IP, PROFINET) Slave address
DR_IE_SLAVE_GPR_ADDR_START     = 0
DR_IE_SLAVE_GPR_ADDR_END       =23
DR_IE_SLAVE_GPR_ADDR_END_BIT   =63

# 경로 수정 기능
DR_DPOS = 0
DR_DVEL = 1

# Homing 기능
DR_HOME_TARGET_MECHANIC = 0
DR_HOME_TARGET_USER     = 1

# movec ori 옵션 
DR_MV_ORI_TEACH  = 0    #교시자세
DR_MV_ORI_FIXED  = 1    #고정자세
DR_MV_ORI_RADIAL = 2    #원주구속자세

# app_type
DR_MV_APP_NONE   = 0
DR_MV_APP_WELD   = 1

# =============================================================================================
# global variable

DR_CONFIG_PRT_EXT_RESULT = False
DR_CONFIG_PRT_RESULT     = False

_g_blend_state  = False
_g_blend_radius = 0.0

_g_velj = [0.0] * DR_VELJ_DT_LEN
_g_accj = [0.0] * DR_ACCJ_DT_LEN

_g_velx = [0.0] * DR_VELX_DT_LEN
_g_velx[0]= 0.0
_g_velx[1]= DR_COND_NONE

_g_accx = [0.0] * DR_ACCX_DT_LEN
_g_accx[0]= 0.0
_g_accx[1]= DR_COND_NONE

_g_coord = DR_BASE
_g_drl_result_th = None

#_g_tp_lock = threading.Lock()       # only 1 execution allowed with TP

_g_test_cnt =0
_g_test_max =0

_g_analog_output_mode_ch1 = -1
_g_analog_output_mode_ch2 = -1

#DR_SUB_PROGRAM_LIST = dict()    #서브 프로그램 저장 리스트

########################################################################################################################################

def wait(second):
    time.sleep(second)

def print_ext_result(str):
    if DR_CONFIG_PRT_EXT_RESULT:
        # print("[{0}] / {1}".format(strftime("%Y-%m-%d %H:%M:%S", gmtime()), str))
        print("__{0}".format(str))

def print_result(str):
    if DR_CONFIG_PRT_RESULT:
        # print("[{0}] / {1}".format(strftime("%Y-%m-%d %H:%M:%S", gmtime()), str))
        print("__{0}".format(str))

def _check_valid_vel_acc_joint(vel, acc, time):
    if float(time) == 0.0:
        for item in vel:
            if float(item) != 0.0:
                break
        else:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v (0.0, when time = 0.0)", True)

        for item in acc:
            if float(item) != 0.0:
                break
        else:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a (0.0, when time = 0.0)", True)

    return

def _check_valid_vel_acc_task(vel, acc, time):

    if float(time) == 0.0:
        if vel[0] <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel1, v1 (0.0, when time = 0.0)", True)
        else:
            if (vel[1]!=DR_COND_NONE) and (vel[1]<=0):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel2, v2 (0.0, when time = 0.0)", True)

        if acc[0] <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc1, a1 (0.0, when time = 0.0)", True)
        else:
            if (acc[1]!=DR_COND_NONE) and (acc[1]<=0):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc2, a2 (0.0, when time = 0.0)", True)

    return


def set_velj(vel):
    vel_list = None

    # vel
    if type(vel) == int or type(vel) == float:
        if vel <= 0:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

        vel_list = [vel] * DR_VELJ_DT_LEN
    elif type(vel) == list and len(vel) == POINT_COUNT:
        vel_list = vel

        if is_number(vel_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

        for item in vel:
            if item <= 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel")
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

    # set global velj
    global _g_velj

    _g_velj = vel_list

    print_result("0 = set_velj(vel:{0})".format(dr_form(vel)))
    return 0

def set_accj(acc):
    acc_list = None

    # acc
    if type(acc) == int or type(acc) == float:
        acc_list = [acc] * DR_ACCJ_DT_LEN

        if acc <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc")
    elif type(acc) == list and len(acc) == POINT_COUNT:
        acc_list = acc

        if is_number(acc_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc")

        for item in acc:
            if item <= 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc")
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc")

    # set global velj
    global _g_accj

    _g_accj = acc_list

    print_result("0 = set_accj(acc:{0})".format(dr_form(acc)))
    return 0

def set_velx(vel1, vel2=DR_COND_NONE):
    #print("vel1={0}, vel2={1}".format(vel1, vel2))

    # vel1
    if type(vel1) != int and type(vel1) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel1")

    if vel1 <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel1")

    # vel2
    if type(vel2) != int and type(vel2) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel2")

    if vel2 != DR_COND_NONE:
        if vel2 <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel2")

    # set global velx
    global _g_velx

    _g_velx = [vel1, vel2]

    print_result("0 = set_velx(vel1:{0}, vel2:{1})".format(dr_form(vel1), dr_form(vel2)))
    return 0

def set_accx(acc1, acc2=DR_COND_NONE):
    #print("acc1={0}, acc2={1}".format(acc1, acc2))

    # acc1
    if type(acc1) != int and type(acc1) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc1")

    if acc1 <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc1")

    # acc2
    if type(acc2) != int and type(acc2) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc2")

    if acc2 != DR_COND_NONE:
        if acc2 <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc2")

    # set global accx
    global _g_accx

    _g_accx = [acc1, acc2]

    print_result("0 = set_accx(acc1:{0}, acc2:{1})".format(dr_form(acc1), dr_form(acc2)))
    return 0

# convert : list -> Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res

# convert : Float64MultiArray -> list
def _ros_Float64MultiArrayTolist(multi_arr_f64):
    _res = []
    for i in range( len(multi_arr_f64) ):
        _res.append( list(multi_arr_f64[i].data) )   
    #print(_res)
    #print(len(_res))
    return _res

##### MATH ################################################################################################################################

class mat(list):
    ##
    # @brief      생성자
    # @details    list에서 상속받은 class이므로, list 객체를 초기화한다.
    # @return     없음
    # @exception  없음
    #
    def __init__(self, data):
        list.__init__(self, data)

    ##
    # @brief      matrix의 dimension을 구한다.
    # @param      mat - matrix (2 dimension list)
    # @return     row, col
    #               row - row의 개수
    #               col - col의 개수
    # @exception  없음
    #
    def _get_dimension(self, mat):
        row = len(mat)
        col = len(mat[0])

        # print("row={0}, col={1}".format(row, col))
        return row, col

    ##
    # @brief      operator - (단항)
    # @details    자신(matrix)의 모든 item을 negative 계산한 새로운 matrix를 리턴한다.
    # @return     matrix - negative 계산한 새로운 matrix
    # @exception  없음
    #
    def __neg__(self):
        row, col = self._get_dimension(self)

        result = [[0] * col for x in range(row)]

        for i in range(row):
            for j in range(col):
                result[i][j] = -self[i][j]

        return mat(result)

    ##
    # @brief      operator +
    # @details    자신(matrix)과 other의 matrix + 연산을 계산한 새로운 matrix를 리턴한다.
    # @param      other - matrix (2 dimension list)
    # @return     matrix - + 계산한 matrix
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #
    def __add__(self, other):
        row_a, col_a = self._get_dimension(self)
        row_b, col_b = self._get_dimension(other)

        if row_a != row_b or col_a != col_b:
            raise DR_Error(DR_ERROR_TYPE, "Inconsistant type : self, other")

        result = [[0] * col_a for x in range(row_a)]

        for i in range(row_a):
            for j in range(col_b):
                result[i][j] = self[i][j] + other[i][j]

        return mat(result)

    ##
    # @brief      operator - (이항)
    # @details    자신(matrix)과 other의 matrix - 연산을 계산한 새로운 matrix를 리턴한다.
    # @param      other - matrix (2 dimension list)
    # @return     matrix - -를 계산한 matrix
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #
    def __sub__(self, other):
        row_a, col_a = self._get_dimension(self)
        row_b, col_b = self._get_dimension(other)

        if row_a != row_b or col_a != col_b:
            raise DR_Error(DR_ERROR_TYPE, "Inconsistant type : self, other")

        result = [[0] * col_a for x in range(row_a)]

        for i in range(row_a):
            for j in range(col_b):
                result[i][j] = self[i][j] - other[i][j]

        return mat(result)

    '''
    __matmul__(@) 연산자 오버로딩은 python3.5 이상 부터 지원 가능   
    현재 리눅스에서는 python3.2를 쓰고 있어서 연산자 오버로딩 사용할 수 없어서
    matrix_mul(A, B) 함수를 이용하여 matrix 곱하기를 수행함. by kabdol2 2017/04/27        
    '''

    ##
    # @brief      operator @
    # @details    자신(matrix)과 other의 matrix 행렬곱 연산을 계산한 새로운 matrix를 리턴한다.
    # @param      other - matrix (2 dimension list)
    # @return     matrix - 행렬곱 계산한 matrix
    # @exception  없음
    #
    def __matmul__(self, other):
        row_a, col_a = self._get_dimension(self)
        row_b, col_b = self._get_dimension(other)

        result = [[0] * col_b for x in range(row_a)]

        for i in range(row_a):
            for j in range(col_b):
                temp = 0

                for k in range(row_b):
                    temp += self[i][k] * other[k][j]

                result[i][j] = temp

        return mat(result)

def r2d(x):
    if type(x) != int and type(x) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : x")

    return degrees(x)

def d2r(x):
    if type(x) != int and type(x) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : x")

    return radians(x)

def _rotm2eul(rotm, flip=0):
    # rotm
    if type(rotm) != list or len(rotm) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotm")

    for item in rotm:
        if type(item) != list or len(item) != 3:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotm[item]")

    if is_number(rotm) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotm({0})".format(rotm))

    # rotation matrix (-1 ~ 1)
    for i in range(3):
        for j in range(3):
            if rotm[i][j] < -1 or rotm[i][j] > 1:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotm[item]")

    # rotation matrix (epsilon 미만의 값은 0으로 처리)
    for i in range(3):
        for j in range(3):
            if abs(rotm[i][j]) < sys.float_info.epsilon:
                rotm[i][j] = 0

    # flip
    if type(flip) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : flip")

    if flip != 0 and flip != 1:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : flip({0})".format(flip))

    r11 = rotm[0][0]
    r12 = rotm[0][1]
    r13 = rotm[0][2]
    r21 = rotm[1][0]
    r22 = rotm[1][1]
    r23 = rotm[1][2]
    r31 = rotm[2][0]
    r32 = rotm[2][1]
    r33 = rotm[2][2]

    # calculate rx, ry, rz
    if abs(r13) < sys.float_info.epsilon and abs(r23) < sys.float_info.epsilon:
        rx = 0
        sp = 0
        cp = 1

        ry = atan2(cp * r13 + sp * r23, r33)
        rz = atan2(-sp * r11 + cp * r21, -sp * r12 + cp * r22)
    else:
        if flip == 0:
            rx = atan2(r23, r13)
            # ry = atan2(sqrt(r13 * r13 + r23 * r23), r33)
            # rz = atan2(r32, -r31)
        else: # -> (flip == 1)
            rx = atan2(-r23, -r13)
            #ry = atan2(-sqrt(r13 * r13 + r23 * r23), r33)
            #rz = atan2(-r32, r31)
        
        sp = sin(rx)
        cp = cos(rx)

        ry = atan2(cp * r13 + sp * r23, r33)
        rz = atan2(-sp * r11 + cp * r21, -sp * r12 + cp * r22)
       

    eulv = [r2d(rx), r2d(ry), r2d(rz)]

    return eulv

def _eul2rotm(eulv):
    # eulv
    if type(eulv) != list or len(eulv) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : eulv")

    if is_number(eulv) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : eulv({0})".format(eulv))

    rotm = [[0] * 3 for i in range(3)]

    rx_r = d2r(eulv[0])
    ry_r = d2r(eulv[1])
    rz_r = d2r(eulv[2])

    sin_rx = sin(rx_r)
    cos_rx = cos(rx_r)

    sin_ry = sin(ry_r)
    cos_ry = cos(ry_r)

    sin_rz = sin(rz_r)
    cos_rz = cos(rz_r)

    # rotation matrix
    rotm[0][0] = (cos_rx * cos_ry * cos_rz) - (sin_rx * sin_rz)
    rotm[0][1] = -(cos_rx * cos_ry * sin_rz) - (sin_rx * cos_rz)
    rotm[0][2] = cos_rx * sin_ry

    rotm[1][0] = (sin_rx * cos_ry * cos_rz) + (cos_rx * sin_rz)
    rotm[1][1] = -(sin_rx * cos_ry * sin_rz) + (cos_rx * cos_rz)
    rotm[1][2] = sin_rx * sin_ry
    
    rotm[2][0] = -(sin_ry * cos_rz)
    rotm[2][1] = sin_ry * sin_rz
    rotm[2][2] = cos_ry

    return rotm

def cal_matrix(matrix):
    row = len(matrix)
    col = len(matrix[0])

    return row, col

def matrix_mul(A, B):
    row_a, col_a = cal_matrix(A)
    row_b, col_b = cal_matrix(B)

    result = [[0] * col_b for x in range(row_a)]

    for i in range(row_a):
        for j in range(col_b):
            temp = 0
            for k in range(row_b):
                temp += A[i][k] * B[k][j]

            result[i][j] = temp

    return result

def transpose(mat):
    row = len(mat)
    col = len(mat[0])

    # row -> col, col -> row
    result = [[0] * row for x in range(col)]

    for i in range(row):
        for j in range(col):
            result[j][i] = mat[i][j]

    return result

def htrans(posx1, posx2):
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]
    rx1 = _posx1[3]
    ry1 = _posx1[4]
    rz1 = _posx1[5]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]
    rx2 = _posx2[3]
    ry2 = _posx2[4]
    rz2 = _posx2[5]

    # trans, rotm
    trans1 = [x1, y1, z1]
    rotm1 = _eul2rotm([rx1, ry1, rz1])

    trans2 = [x2, y2, z2]
    rotm2 = _eul2rotm([rx2, ry2, rz2])

    # result
    #result_rotm = mat(rotm1) @ mat(rotm2)  #support in python3.5
    result_rotm = matrix_mul(mat(rotm1), mat(rotm2))
    
    transp_trans1 = transpose([trans1])
    transp_trans2 = transpose([trans2])

    #temp = mat(transp_trans1) + (mat(rotm1) @ mat(transp_trans2))  #support in python3.5
    temp = mat(transp_trans1) + matrix_mul(mat(rotm1), mat(transp_trans2)) 

    result_trans = transpose(temp)

    result_eulv = _rotm2eul(result_rotm)

    result_posx = result_trans[0] + result_eulv
    # result_posx = posx(result_posx)

    #--------------------------------------------------------------------------------------
    #DRCF._report_line_off_sleep(0.0001)
    #DRCF._line_call_back_func_enable()
    #--------------------------------------------------------------------------------------
    #DRCF.python_sleep(_G_DRL_MATH_SLEEP)    #cpu 과부화 회피용

    return result_posx

##### SYSTEM ##############################################################################################################################
def set_robot_mode(robot_mode):
    if type(robot_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_mode")

    # ROS service call
    if __ROS__:
        srv = _ros_set_robot_mode(robot_mode)
        ret = 0 if (srv.success == True) else -1
    return ret

def get_robot_mode():

    # ROS service call
    if __ROS__:
        srv = _ros_get_robot_mode()
    return srv.robot_mode

def set_robot_system(robot_system):
    if type(robot_system) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_system")

    # ROS service call
    if __ROS__:
        srv = _ros_set_robot_system(robot_system)
        ret = 0 if (srv.success == True) else -1
    return ret

def get_robot_system():
    # ROS service call
    if __ROS__:
        srv = _ros_get_robot_system()
    return srv.robot_system

def get_robot_state():
    # ROS service call
    if __ROS__:
        srv = _ros_get_robot_state()
    return srv.robot_state

def set_robot_speed_mode(speed_mode):
    if type(speed_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed_mode")

    # ROS service call
    if __ROS__:
        srv = _ros_set_robot_speed_mode(speed_mode)
        ret = 0 if (srv.success == True) else -1
    return ret

def get_robot_speed_mode():
    # ROS service call
    if __ROS__:
        srv = _ros_get_robot_speed_mode()
    return srv.speed_mode

def set_safe_stop_reset_type(reset_type):
    if type(reset_type) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : reset_type")

    # ROS service call
    if __ROS__:
        srv = _ros_set_safe_stop_reset_type(reset_type)
        ret = 0 if (srv.success == True) else -1
    return ret

def get_current_pose(space_type):
    if type(space_type) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : space_type")

    # ROS service call
    if __ROS__:
        srv = _ros_get_current_pose(space_type)
    return srv.pos

def set_robot_control(robot_control):
    if type(robot_control) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_control")

    # ROS service call
    if __ROS__:
        srv = _ros_set_robot_control(robot_control)
        ret = 0 if (srv.success == True) else -1
    return ret

def manage_access_control(access_control):
    if type(access_control) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : access_control")

    # ROS service call
    if __ROS__:
        srv = _ros_manage_access_control(access_control)
        ret = 0 if (srv.success == True) else -1
    return ret

def get_current_solution_space():
    # ROS service call
    if __ROS__:
        srv = _ros_get_current_solution_space()
    return srv.solution_space

def get_last_alarm():
     #ROS service call
    if __ROS__:
        srv = _ros_get_last_alarm()
    return srv.log_alarm

##### Auxiliary Control ##############################################################################################################################
def get_control_mode():
    if __ROS__:
        srv = _ros_get_control_mode()  
        mode = srv.control_mode
    else:
        # C function call
        mode = PythonMgr.py_get_control_mode()
        # check result
        _check_ext_result(mode)

    #print_result("{0} = get_control_mode()".format(mode))
    return mode

def get_control_space():
    if __ROS__:
        srv = _ros_get_control_space()  
        space = srv.space
    else:
        # C function call
        space = PythonMgr.py_get_control_space()
        # check result
        _check_ext_result(space)

    return space

def get_current_posj():
    if __ROS__:
        srv = _ros_get_current_posj()  
        pos = list(srv.pos)  # Convert tuple to list
    else:
        # C function call
        pos = PythonMgr.py_get_current_posj()
        # check result
        _check_ext_result(pos)
    
    # set posj
    cur_pos = posj(pos)
    #print_result("{0} = get_current_posj()".format(cur_pos))

    return cur_pos

def get_current_velj():
    if __ROS__:
        srv = _ros_get_current_velj()  
        vel = list(srv.joint_speed)  # Convert tuple to list
    else:
        # C function call
        vel = PythonMgr.py_get_current_velj()
        # check result
        _check_ext_result(vel)

    #print_result("{0} = get_current_velj()".format(dr_form(vel)))

    return vel

def get_desired_posj():
    if __ROS__:
        srv = _ros_get_desired_posj()  
        pos = list(srv.pos)  # Convert tuple to list
    else:
        # C function call
        pos = PythonMgr.py_get_desired_posj()
        # check result
        _check_ext_result(pos)

    # set posj
    desired_pos = posj(pos)
    #print_ext_result("{0} = get_desired_posj()".format(desired_pos))

    return desired_pos

def get_desired_velj():
    if __ROS__:
        srv = _ros_get_desired_velj()  
        vel = list(srv.joint_vel)  # Convert tuple to list
    else:
        # C function call
        vel = PythonMgr.py_get_desired_velj()
        # check result
        _check_ext_result(vel)

    #print_result("{0} = get_desired_velj()".format(dr_form(vel)))

    return vel

def get_current_posx(ref=None):

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref != DR_BASE and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS__:
        srv = _ros_get_current_posx(_ref)  
        posx_info = _ros_Float64MultiArrayTolist(srv.task_pos_info) # Convert Float64MultiArray to list
        pos = []
        for i in range(POINT_COUNT):
            pos.append(posx_info[0][i])
        sol = int(round( posx_info[0][6] ))
    else:
        # C function call
        ret = PythonMgr.py_get_current_posx(_ref)
        # check return
        _check_ext_result(ret)
        # wait for job to be finished
        proc_id = ret
        _wait_result(proc_id)
        # get result
        pos, sol  = PythonMgr.py_get_result(proc_id)

    conv_posx = posx(pos)
    #print_result("({0}, {1}) = get_current_posx(ref:{2})".format(conv_posx, sol, _ref))
    return conv_posx, sol

def get_current_tool_flange_posx(ref=None):

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS__:
        srv = _ros_get_current_tool_flange_posx(_ref)  
        pos = list(srv.pos)  # Convert tuple to list
    else:
        # C function call
        pos = PythonMgr.py_get_current_tool_flange_posx(_ref)
        # check result
        _check_ext_result(pos)

    # set posx type
    cur_pos = posx(pos)
    #print_result("{0} = get_current_tool_flange_posx(ref:{1})".format(cur_pos, _ref))
    return cur_pos

def get_current_velx(ref=None):
    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS__:
        srv = _ros_get_current_velx(_ref)  
        vel = srv.vel
    else:
        # C function call
        vel = PythonMgr.py_get_current_velx(_ref)
        # check result
        _check_ext_result(vel)

    #print_result("{0} = get_current_velx(ref:{1})".format(dr_form(vel), _ref))
    return vel

def get_desired_posx(ref=None):
    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS__:
        srv = _ros_get_desired_posx(_ref)  
        pos = list(srv.pos)  # Convert tuple to list
    else:
        # C function call
        ret = PythonMgr.py_get_desired_posx(_ref)
        # check return
        _check_ext_result(ret)
        # wait for job to be finished
        proc_id = ret
        _wait_result(proc_id)
        # get result
        pos = PythonMgr.py_get_result(proc_id)

    conv_posx = posx(pos)

    return conv_posx

def get_desired_velx(ref=None):
    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS__:
        srv = _ros_get_desired_velx(_ref)  
        vel = list(srv.vel)  # Convert tuple to list  
    else:
        # C function call
        vel = PythonMgr.py_get_desired_velx(_ref)
        # check result
        _check_ext_result(vel)

    #print_result("{0} = get_desired_velx(ref:{1})".format(dr_form(vel), _ref))
    return vel

def get_current_solution_space():
    if __ROS__:
        srv = _ros_get_current_solution_space()  
        sol_space = srv.sol_space  
    else:
        # C function call
        sol_space = PythonMgr.py_get_current_solution_space()
        # check result
        _check_ext_result(sol_space)

    #print_result("{0} = get_current_solution_space()".format(sol_space))
    return sol_space

def get_current_rotm(ref=None):

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS__:
        srv = _ros_get_current_rotm(_ref)  
        rotm = _ros_Float64MultiArrayTolist(srv.rot_matrix) # Convert Float64MultiArray to list 
    else:
        # C function call
        rotm = PythonMgr.py_get_current_rotm(_ref)
        # check result
        _check_ext_result(rotm)

    #print_result("{0} = get_current_rotm(ref:{1})".format(dr_form(rotm), _ref))
    return rotm

def get_joint_torque():
    if __ROS__:
        srv = _ros_get_joint_torque()  
        torque = list(srv.jts)  # Convert tuple to list  
    else:
        # C function call
        torque = PythonMgr.py_get_joint_torque()
        # check result
        _check_ext_result(torque)

    #print_result("{0} = get_joint_torque()".format(dr_form(torque)))
    return torque

def get_external_torque():
    if __ROS__:
        srv = _ros_get_external_torque()  
        torque = list(srv.ext_torque)  # Convert tuple to list  
    else:
        # C function call
        torque = PythonMgr.py_get_external_torque()
        # check result
        _check_ext_result(torque)

    #print_result("{0} = get_external_torque()".format(dr_form(torque)))
    return torque

def get_tool_force(ref=None):

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS__:
        srv = _ros_get_tool_force(_ref)  
        force = list(srv.tool_force)  # Convert tuple to list  
    else:
        # C function call
        force = PythonMgr.py_get_tool_force(_ref)
        # check result
        _check_ext_result(force)

    #print_result("{0} = get_tool_force(ref:{1})".format(dr_form(force), _ref))
    return force

def get_solution_space(pos):
    _pos = get_posj(pos)

    if __ROS__:
        srv = _ros_get_solution_space(_pos)  
        sol = srv.sol_space
    else:
        # C function call
        ret = PythonMgr.py_get_solution_space(_pos)
        # check return
        _check_ext_result(ret)
        # wait for job to be finished
        proc_id = ret
        _wait_result(proc_id)
        # get result
        sol = PythonMgr.py_get_result(proc_id)

    return sol

def get_orientation_error(xd, xc, axis):
    # xd, xc
    _xd = get_posx(xd)
    _xc = get_posx(xc)

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
        axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    if __ROS__:
        srv = _ros_get_orientation_error(_xd, _xc, axis)  
        ret = srv.ori_error
    else:
        # C function call
        ret = PythonMgr.py_get_orientation_error(_xd, _xc, axis)

    return ret


##### MOTION ##############################################################################################################################
def trans(pos, delta, ref=None, ref_out=DR_BASE):
    # pos, delta
    _pos = get_posx(pos)
    _delta = get_posx(delta)
    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref
    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    # check ref_out
    if type(ref_out) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")
    if ref_out != DR_BASE and ref_out != DR_WORLD and (ref_out < DR_TC_USER_MIN or ref_out > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref_out({0})".format(ref_out))
    # ROS service call
    if __ROS__: 
        srv = _ros_trans(_pos, _delta, _ref, ref_out)
        pos = srv.trans_pos
    else:   
        # C function call
        ret = PythonMgr.py_trans(_pos, _delta, _ref, ref_out)
        # check return
        _check_ext_result(ret)
        # wait for job to be finished
        proc_id = ret
        _wait_result(proc_id)
        # get result
        pos = PythonMgr.py_get_result(proc_id)
    #trans_posx = posx(pos)
    return pos

def fkin(pos, ref=None):

    _pos = get_posj(pos)

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # ROS service call
    if __ROS__: 
        srv = _ros_fkin(_pos, _ref)
        ret = srv.conv_posx
    else:   
        ret = PythonMgr.py_fkin(_pos, _ref)
    return ret

def ikin(pos, sol_space, ref=None):

    _pos = get_posx(pos)

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # sol_space
    if type(sol_space) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol_space")

    if sol_space < DR_SOL_MIN or sol_space > DR_SOL_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol_space({0})".format(sol_space))

    # ROS service call
    if __ROS__: 
        srv = _ros_ikin(_pos, sol_space, _ref)
        ret = srv.conv_posj
    else:   
        ret = PythonMgr.py_ikin(_pos, sol_space, _ref)
    return ret

def set_ref_coord(coord):
    # coord
    if type(coord) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : coord")

    if coord != DR_BASE and coord != DR_TOOL and coord != DR_WORLD and not (DR_TC_USER_MIN <= coord <= DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : coord({0})".format(coord))

    # ROS service call
    if __ROS__: 
        srv = _ros_set_ref_coord(coord)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:   
        ret = PythonMgr.py_set_ref_coord(coord)

    # set global accx
    global _g_coord

    _g_coord = coord

    #print_result("{0} = set_ref_coord(coord:{1})".format(ret, coord))
    return ret

def movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
    return ret
def amovej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
    return ret
def _movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_posj(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius< 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__: 
        srv = _ros_movej(_pos, _vel[0], _acc[0], _time, _radius, mod, ra, _async)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:   
        ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
    return ret

def movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=0)
    return ret 
def amovejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=1)
    return ret 
def _movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_posx(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type, ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # sol
    if type(sol) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol")

    if sol < DR_SOL_MIN or sol > DR_SOL_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__: 
        srv = _ros_movejx(_pos, _vel[0], _acc[0], _time, _radius, _ref, mod, ra, sol, _async)   
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:    
        ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
        print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
    return ret

def movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=0)
    return ret
def amovel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=1)
    return ret
def _movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_normal_pos(pos, def_type=posx)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ���� 
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    #print("_vel[0]={0}, _vel[1]={1}".format(_vel[0],_vel[1]))

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    #print("_acc[0]={0}, _acc[1]={1}".format(_acc[0],_acc[1]))

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # qcommand
    if type(_pos) == posx:
        qcommand = 0
    else:
        qcommand = 1

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__: 
        srv = _ros_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, _async) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:    
        ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
    return ret

def movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=0)
    return ret
def amovec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=1)
    return ret
def _movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, _async=0):

    # _pos1, _pos2
    
    _pos1 = get_normal_pos(pos1, def_type=posx)
    _pos2 = get_normal_pos(pos2, def_type=posx)

    if type(_pos1) != type(_pos2):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos1, ps2")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _angle
    temp = get_param(angle, an)
    if temp == None:
        _angle = [0, 0]
    else:
        if type(temp) == int or type(temp) == float:
            _angle = [temp, 0]
        elif type(temp) == list and len(temp) == DR_ANGLE_DT_LEN:
            _angle = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : angle, an")

    if is_number(_angle) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

    for item in _angle:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # qcommand
    if type(_pos1) == posx:
        qcommand = 0
    else:
        qcommand = 1

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        # make multi pos
        _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
        #print(_circle_pos)
        srv = _ros_movec(_circle_pos, _vel, _acc, _time, _radius, _ref, mod, _angle[0], _angle[1], ra, _async) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:   
        ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                         .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
    return ret

def movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=0)
    return ret
def amovesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=1)
    return ret
def _movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):
    # pos_list
    if type(pos_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

    for item in pos_list:
        if type(item) != posj:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        # make multi pos
        _spline_posj = _ros_listToFloat64MultiArray(pos_list)
        srv = _ros_movesj(_spline_posj, len(_spline_posj), _vel, _acc, _time, mod, _async)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:    
        ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
        print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                         .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
    return ret

def movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=0)
    return ret
def amovesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=1)
    return ret
def _movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, _async=0):
    # pos_list
    if type(pos_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

    for item in pos_list:
        if type(item) != posx:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # vel_opt
    if type(vel_opt) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel_opt")

    if vel_opt != DR_MVS_VEL_NONE and vel_opt != DR_MVS_VEL_CONST:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel_opt")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        _spline_posx = _ros_listToFloat64MultiArray(pos_list)
        srv = _ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
        print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                        .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
    return ret

def moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=0)
    return ret
def amoveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=1)
    return ret
def _moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):

    # seg_list
    if type(seg_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list")

    if len(seg_list) == 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : seg_list")

    _seg_list = []
    if __ROS__:
        for seg in seg_list:
            _seg_list.append(seg.to_list())
    else:
        for seg in seg_list:
            if type(seg) != posb:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list(item)")
            else:
                _seg_list.append(seg.to_list())

    if __ROS__:
        print(len(_seg_list))
        _ros_seg_list = []
        _tmp_list = []

        for s in range(0, len(_seg_list)):
            #print(s)
            for elemnt in range(0, len(_seg_list[s])):
                if _seg_list[s][elemnt] == None:
                    _seg_list[s][elemnt] = [0.0]*POINT_COUNT
         
            # make [pos1] + [pos2] + [type] + [radius]
            _tmp_list = _seg_list[s][1] + _seg_list[s][2] + [_seg_list[s][0]] + [_seg_list[s][3]]
            _ros_seg_list.append(_tmp_list)
            
    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS Service call
    if __ROS__:
        seg = _ros_listToFloat64MultiArray(_ros_seg_list)
        srv = _ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, _ref, mod, _async)    
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
        print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                         .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
    return ret

def move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=0)
    return ret
def amove_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=1)
    return ret
def _move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, _async=0):
    # rev
    if type(rev) != int and type(rev) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rev")

    if rev <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rev (Ranges: rev > 0)")

    # rmax
    if type(rmax) != int and type(rmax) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rmax")

    if rmax <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value: rmax (Ranges: rmax > 0)")

    # lmax
    if type(lmax) != int and type(lmax) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : lmax")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELX_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCX_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis")

    # ref
    if type(ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if ref != DR_BASE and ref != DR_TOOL and (ref < DR_TC_USER_MIN or ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        srv = _ros_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:    
        ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
        print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                         .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
    return ret

def move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, _async=0)
    return ret
def amove_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, _async=1)
    return ret
def _move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL, _async=0):
    _amp = [POINT_COUNT]
    _period = [POINT_COUNT]
    _atime =0.0
    _repeat =0.0
    _ref=0 

    # amp : float[6] 
    if type(amp) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : amp")
    if len(amp) != POINT_COUNT:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : amp")
    _amp =amp

    # period : float or float[6] 
    if (type(period) != int) and (type(period) != float) and (type(period) != list):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : period")

    if (type(period) == int) or (type(period) == float):
        _period = [period] * POINT_COUNT
    else: #list �� ��� 
        if len(period) != POINT_COUNT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : period")
        _period = period

    # atime 
    if atime == None:
        _atime = 0.0
    else:
        if (type(atime) != int) and (type(atime) != float):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : atime")
        if atime < 0.0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : atime")
        _atime = atime

    # repeat 
    if repeat == None:
        _repeat = 1
    else:
        if (type(repeat) != int) and (type(repeat) != float): 
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : repeat")
        if repeat < 0.0: 
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : repeat")
        _repeat = repeat

    # ref
    if ref == None:
        #_ref = _g_coord
        _ref = DR_TOOL
    else:
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if ref < DR_BASE or ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
        _ref = ref

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        srv = _ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
        print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                         .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
    return ret

def  move_home(target=None):

    # target
    if target == None:
        _target = DR_HOME_TARGET_MECHANIC
    else:
        _target = target

    if __ROS__:
        srv = _ros_move_home(_target)    
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_move_home(_target)

    return ret

def mwait(time=0):
    ret = _move_wait(time)
    return ret
def _move_wait(time):
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time")

    # ROS service call
    if __ROS__:
        srv = _ros_move_wait()  #ROS 에서는 time 인자를 사용하지 않음. 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:    
        ret = PythonMgr.py_mwait(time)
        print_ext_result("{0} = PythonMgr.py_mwait(time:{1})".format(ret, dr_form(time)))
    return ret

def check_motion():

    if __ROS__:
        srv = _ros_check_motion()  
        ret = srv.status
    else:    
        # C function call
        ret = PythonMgr.py_check_motion()

    return ret

def change_operation_speed(speed):
    if type(speed) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")

    if speed < DR_OP_SPEED_MIN or speed > DR_OP_SPEED_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : speed({0})".format(speed))

    if __ROS__:
        srv = _ros_change_operation_speed(speed)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else: 
        # C function call
        ret = PythonMgr.py_change_operation_speed(speed)
 
    return ret

def enable_alter_motion(n, mode, ref=None, limit_dPOS=None, limit_dPOS_per=None):
    # n
    if type(n) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : n")
    if n < 0:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : (n>=0)")
    _n = n #*20

    # mode
    if mode < DR_DPOS or mode > DR_DVEL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))
    _mode = mode

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if(None==limit_dPOS):
        _limit_dPOS = [DR_COND_NONE, DR_COND_NONE]
    else:
        _limit_dPOS = limit_dPOS

    if(None==limit_dPOS_per):
        _limit_dPOS_per = [DR_COND_NONE, DR_COND_NONE]
    else:
        _limit_dPOS_per = limit_dPOS_per

    if __ROS__:
        srv = _ros_enable_alter_motion(_n, _mode, _ref, _limit_dPOS, _limit_dPOS_per)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:    
        # C function call
        ret = PythonMgr.py_enable_alter_motion(_n, _mode, _ref, _limit_dPOS, _limit_dPOS_per)

    return ret

def alter_motion(dposx):

    # _dposx
    if (type(dposx) == posx):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : dposx")

    _dposx = get_normal_pos(dposx, def_type=posx)

    if __ROS__:
        srv = _ros_alter_motion(_dposx)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_alter_motion(_dposx)

    return ret

def disable_alter_motion():

    if __ROS__:
        srv = _ros_disable_alter_motion()  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:        
        # C function call
        ret = PythonMgr.py_disable_alter_motion()

    return ret

def jog(jog_axis, ref=0, speed=0):
    if type(jog_axis) != int and type(jog_axis) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis")
    
    if type(ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if type(speed) != int and type(speed) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
    # ROS service call
    if __ROS__:
        srv = _ros_jog(jog_axis, ref, speed)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def jog_multi(jog_axis_list, ref=0, speed=0):
    if type(jog_axis_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis_list")
   
    if type(ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if type(speed) != int and type(speed) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
    # ROS service call
    if __ROS__:
        srv = _ros_jog_multi(jog_axis_list, ref, speed)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret


##### force/stiffness Control #########################################################################################################################
def get_workpiece_weight():

    print("================================> get_workpiece_weight()")
    if __ROS__:
        srv = _ros_get_workpiece_weight()  
        ret = srv.weight
    else:
        # C function call
        ret = PythonMgr.py_get_workpiece_weight()

    return ret

def reset_workpiece_weight():

    if __ROS__:
        srv = _ros_reset_workpiece_weight()  
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_reset_workpiece_weight()

    return ret

def set_singular_handling(mode = DR_AVOID):
    if type(mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mode")

    if mode < DR_AVOID or mode > DR_VAR_VEL:    #DR_TASK_STOP
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))

    if __ROS__:
        srv = _ros_set_singularity_handling(mode)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_set_singular_handling(mode)

    return ret
def set_singularity_handling(mode = DR_AVOID):
    ret = set_singular_handling(mode)
    return ret

def parallel_axis(*args, **kargs):
    len_args = len(args)
    len_kargs = len(kargs)
    _nType = 0

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "axis" and key != "ref":
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, vect, axis, ref")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _vect = get_kargs(kargs, "vect")
    _axis = get_kargs(kargs, "axis")
    _ref = get_kargs(kargs, "ref")

    # check parameter combination
    if len_args == 0:
        if len_kargs == 2:
            if _vect == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 3:
            _nType = 3
            if _vect == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 4:
            if _x1 == None or _x2 == None or _x3 == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 5:
            _nType = 2
            if _x1 == None or _x2 == None or _x3 == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 1:
        if len_kargs != 1:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        _vect = args[0]

        if _axis == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 2:
        if len_kargs == 1:  # +ref
            _nType = 3
            _vect = args[0]
            _axis = args[1]
        else:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        _vect = args[0]
        _axis = args[1]

    elif len_args == 3:
        if len_kargs == 0:  # +ref
            _nType = 3
            _vect = args[0]
            _axis = args[1]
            _ref  = args[2]
        else:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]

            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 4:
        if len_kargs == 1:  # + ref
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]
        else:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]

    elif len_args == 5:
        if len_kargs == 0:  # + ref
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]
            _ref = args[4]
        else:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # check parameter type, value
    if _x1 != None:
        _x1 = get_posx(_x1)
        _x2 = get_posx(_x2)
        _x3 = get_posx(_x3)
    else:
        if type(_vect) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
        if len(_vect) != DR_VECTOR_DT_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))

    if type(_axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))

    # C function call
    if _nType == 2: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS__:
            srv = _ros_parallel_axis1(_x1, _x2, _x3, _axis, _ref)  
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_parallel_axis(_x1, _x2, _x3, _axis, _ref)
        #print_ext_result("{0} = PythonMgr.py_parallel_axis_ref(x1:{1}, x2:{2}, x3:{3}, axis:{4}, ref:{5})".format(ret, _x1, _x2, _x3, _axis, _ref))
    elif _nType == 3: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS__:
            srv = _ros_parallel_axis2(_vect, _axis, _ref)  
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_parallel_axis_ex(_vect, _axis, _ref)
        #print_ext_result("{0} = PythonMgr.py_parallel_axis_ex_ref(vector:{1}, axis:{2}, ref:{3})".format(ret, dr_form(_vect), _axis, _ref))
    else:   # 기존
        _ref = DR_BASE
        if _x1 != None:
            if __ROS__:
                srv = _ros_parallel_axis1(_x1, _x2, _x3, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_parallel_axis(_x1, _x2, _x3, _axis, _ref)
            #print_ext_result("{0} = PythonMgr.py_parallel_axis(x1:{1}, x2:{2}, x3:{3}, axis:{4}), ref:{5})".format(ret, _x1, _x2, _x3, _axis, _ref))
        else:
            if __ROS__:
                srv = _ros_parallel_axis2(_vect, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_parallel_axis_ex(_vect, _axis, _ref)
            #print_ext_result("{0} = PythonMgr.py_parallel_axis_ex(vector:{1}, axis:{2}, ref:{3})".format(ret, dr_form(_vect), _axis, _ref))
    return ret

def align_axis(*args, **kargs):
    len_args = len(args)
    len_kargs = len(kargs)
    _nType = 0

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "pos" and key != "axis" and key != "ref":
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _vect = get_kargs(kargs, "vect")
    _pos = get_kargs(kargs, "pos")
    _axis = get_kargs(kargs, "axis")
    _ref = get_kargs(kargs, "ref")

    # check parameter combination
    if len_args == 0:
        if len_kargs == 3:
            if _vect == None or _pos == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 5:
            if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 6:
            _nType = 2
            if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 4:
            _nType = 3
            if _vect == None or _pos == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 1:
        if len_kargs != 2:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        _vect = args[0]

        if _pos == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        if _axis == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 2:
        if len_kargs != 1:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        _vect = args[0]
        _pos = args[1]

        if _axis == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 3:
        if len_kargs == 0:
            _vect = args[0]
            _pos = args[1]
            _axis = args[2]
        elif len_kargs == 1:
            _nType = 3
            _vect = args[0]
            _pos = args[1]
            _axis = args[2]
        elif len_kargs == 2:
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]

            if _pos == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 4:
        if len_kargs == 0:
            _nType = 3
            _vect = args[0]
            _pos = args[1]
            _axis = args[2]
            _ref = args[3]
        else:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]

            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 5:
        if len_kargs == 0:
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]
            _axis = args[4]
        elif len_kargs == 1:    # + ref
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]
            _axis = args[4]

    elif len_args == 6:
        if len_kargs != 0:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        _nType = 2
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _pos = args[3]
        _axis = args[4]
        _ref  = args[5]

    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # check parameter type, value
    if _x1 != None:
        _x1 = get_posx(_x1)
        _x2 = get_posx(_x2)
        _x3 = get_posx(_x3)
    else:
        if type(_vect) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
        if len(_vect) != DR_VECTOR_DT_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))

    norm_pos = get_posx(_pos)

    if type(_axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))

    # C function call
    if _nType == 2: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS__:
            srv = _ros_align_axis1(_x1, _x2, _x3, norm_pos, _axis, _ref)  
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_align_axis(_x1, _x2, _x3, norm_pos, _axis, _ref)

        #print_ext_result("{0} = PythonMgr.py_align_axis(x1:{1}, x2:{2}, x3:{3}, pos:{4}, axis:{5}, ref{6})" \
        #                 .format(ret, _x1, _x2, _x3, norm_pos, _axis, _ref))
    elif _nType == 3: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS__:
            srv = _ros_align_axis2(_vect, norm_pos, _axis, _ref)  
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_align_axis_ex(_vect, norm_pos, _axis, _ref)

        #print_ext_result("{0} = PythonMgr.py_align_axis_ex(vect:{1}, pos{2}, axis:{3}, ref{4})" \
        #                 .format(ret, dr_form(_vect), norm_pos, _axis, _ref))
    else:   # 기존
        _ref = DR_BASE
        if _x1 != None:
            if __ROS__:
                srv = _ros_align_axis1(_x1, _x2, _x3, norm_pos, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_align_axis(_x1, _x2, _x3, norm_pos, _axis, _ref)
            #print_ext_result("{0} = PythonMgr.py_align_axis(x1:{1}, x2:{2}, x3:{3}, pos:{4}, axis:{5}, ref{6})" \
            #                 .format(ret, _x1, _x2, _x3, norm_pos, _axis, _ref))
        else:

            if __ROS__:
                srv = _ros_align_axis2(_vect, norm_pos, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_align_axis_ex(_vect, norm_pos, _axis, _ref)
            #print_ext_result("{0} = PythonMgr.py_align_axis_ex(vect:{1}, pos{2}, axis:{3}, ref{4})" \
            #                 .format(ret, dr_form(_vect), norm_pos, _axis, _ref))
    return ret

def is_done_bolt_tightening(m=0, timeout=0, axis=None):
    # m
    if type(m) != int and type(m) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : m")

    # timeout
    if type(timeout) != int and type(timeout) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : timeout")

    if timeout < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : timeout({0})".format(timeout))

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    if __ROS__:
        srv = _ros_is_done_bolt_tightening(m, timeout, axis)  
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_is_done_bolt_tightening(m, timeout, axis)
        #print_ext_result("{0} = PythonMgr.py_is_done_bolt_tightening(m:{1}, timeout:{2}, axis:{3})" \
        #                 .format(ret, dr_form(m), dr_form(timeout), axis))

    return ret

def release_compliance_ctrl():

    if __ROS__:
        srv = _ros_release_compliance_ctrl()  
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_release_compliance_ctrl()
        #print_ext_result("{0} = PythonMgr.py_release_compliance_ctrl()".format(ret))

    return ret

def task_compliance_ctrl(stx=[3000, 3000, 3000, 200, 200, 200], time=0):
    # st
    if type(stx) != list or len(stx) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")

    if is_number(stx) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))

    # _ref
    _ref = _g_coord

    # time
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:     # -1 : motion default
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    if __ROS__:
        srv = _ros_task_compliance_ctrl(stx, _ref, _time)  
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_task_compliance_ctrl(stx, _ref, _time)
        #print_ext_result("{0} = PythonMgr.py_task_compliance_ctrl(stx:{1}, ref:{2}, time:{3})" \
        #                 .format(ret, dr_form(stx), _ref, dr_form(_time)))

    return ret

def set_stiffnessx(stx=[500, 500, 500, 100, 100, 100], time=0):
    # st
    if type(stx) != list or len(stx) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")

    if is_number(stx) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))

    # _ref
    _ref = _g_coord

    # time
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:     # -1 : motion default
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    if __ROS__:
        srv = _ros_set_stiffnessx(stx, _ref, _time)  
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_set_stiffnessx(stx, _ref, _time)
        #print_ext_result("{0} = PythonMgr.py_set_stiffnessx(stx:{1}, ref:{2}, time:{3})".format(ret, dr_form(stx), _ref, dr_form(_time)))

    return ret

def calc_coord(*args, **kargs):

    #arg(x1, ref, mod)                    arg =0 , kargs =3    _nType=1
    #arg(x1, x2, ref, mod)                arg =0 , kargs =4    _nType=2
    #arg(x1, x2, x3, ref, mod)            arg =0 , kargs =5    _nType=3
    #arg(x1, x2, x3, x4, ref, mod)        arg =0 , kargs =6    _nType=4

    #arg(?, ref, mod)                     arg =1 , kargs =2    _nType=1
    #arg(?, ?, ref, mod)                  arg =2 , kargs =2    _nType=2
    #arg(?, ?, ?, ref, mod)               arg =3 , kargs =2    _nType=3
    #arg(?, ?, ?, ?, ref, mod)            arg =4 , kargs =2    _nType=4

    #arg(?, ?, ?)                         arg =3 , kargs =0    _nType=1
    #arg(?, ?, ?, ?)                      arg =4 , kargs =0    _nType=2
    #arg(?, ?, ?, ?, ?)                   arg =5 , kargs =0    _nType=3
    #arg(?, ?, ?, ?, ?, ?)                arg =6 , kargs =0    _nType=4

    len_args = len(args)
    len_kargs = len(kargs)
    _cnt_pos = 0

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "x4" and key != "ref" and key != "mod":
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, x4, ref, mod")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _x4 = get_kargs(kargs, "x4")
    _ref = get_kargs(kargs, "ref")
    _mod = get_kargs(kargs, "mod")

    if(_x1 == None):
        _x1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if(_x2 == None):
        _x2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if(_x3 == None):
        _x3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if(_x4 == None):
        _x4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if len_args == 0 and len_kargs ==3:
        _cnt_pos = 1
        print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
    elif len_args == 0 and len_kargs ==4:
        _cnt_pos = 2
        print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
    elif len_args == 0 and len_kargs ==5:
        _cnt_pos = 3
        print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
    elif len_args == 0 and len_kargs ==6:
        _cnt_pos = 4
        print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

    elif len_args == 1 and len_kargs ==2:
        _cnt_pos = 1
        _x1 = args[0]
        print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
    elif len_args == 2 and len_kargs ==2:
        _cnt_pos = 2
        _x1 = args[0]
        _x2 = args[1]
        print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
    elif len_args == 3 and len_kargs ==2:
        _cnt_pos = 3
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
    elif len_args == 4 and len_kargs ==2:
        _cnt_pos = 4
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _x4 = args[3]
        print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

    elif len_args == 3 and len_kargs ==0:
        _cnt_pos = 1
        _x1 = args[0]
        _ref =args[1]
        _mod =args[2]
        print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
    elif len_args == 4 and len_kargs ==0:
        _cnt_pos = 2
        _x1 = args[0]
        _x2 = args[1]
        _ref =args[2]
        _mod =args[3]
        print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
    elif len_args == 5 and len_kargs ==0:
        _cnt_pos = 3
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _ref =args[3]
        _mod =args[4]
        print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
    elif len_args == 6 and len_kargs ==0:
        _cnt_pos = 4
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _x4 = args[3]
        _ref =args[4]
        _mod =args[5]
        print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")

    print("FINAL : x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

    # check parameter type, value
    if _x1 != None: 
        _x1 = get_posx(_x1)
    if _x2 != None: 
        _x2 = get_posx(_x2)
    if _x3 != None: 
        _x3 = get_posx(_x3)
    if _x4 != None: 
        _x4 = get_posx(_x4)
    if _ref !=None:
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    if _mod != None:
        if _mod != 0 and _mod != 1:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(_mod))

    if __ROS__:
        srv = _ros_calc_coord(_cnt_pos, _x1, _x2, _x3, _x4, _ref, _mod)  
        pos = list(srv.conv_posx)  # Convert tuple to list 
    else:
        # C function call
        ret = PythonMgr.py_calc_coord(_cnt_pos, _x1, _x2, _x3, _x4, _ref, _mod)
        if PY_EXT_RET_UNSUPPORT_CMD == ret:
            raise DR_Error(DR_ERROR_TYPE, "unsupported command")
        # check return
        _check_ext_result(ret)

        # wait for job to be finished
        proc_id = ret
        _wait_result(proc_id)

        # get result
        pos = PythonMgr.py_get_result(proc_id)
    
    conv_posx = posx(pos)

    return conv_posx

def set_user_cart_coord(*args, **kargs):
    len_args = len(args)
    len_kargs = len(kargs)
    _nType =0  

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "pos" and key != "u1" and key != "v1" and key != "v1" and key != "ref":
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, pos, u1, v1, ref")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _pos= get_kargs(kargs, "pos")
    _u1 = get_kargs(kargs, "u1")
    _v1 = get_kargs(kargs, "v1")
    _ref = get_kargs(kargs, "ref")

    # check parameter combination
    #set_user_cart_coord(x1=?, x2=?, x3=?, pos=?)   arg =0 , kargs =4    _nType=0
    #set_user_cart_coord(u1=?, v1=?, pos=?)         arg =0 , kargs =3 *  _nType=1
    #set_user_cart_coord(x1=?, x2=?, x3=?)          arg =0 , kargs =3 *  _nType=2

    #set_user_cart_coord(x1=?, x2=?, x3=?, ?)       arg =1 , kargs =3    _nType=0
    #set_user_cart_coord(u1=?, v1=?, ?)             arg =1 , kargs =2    _nType=1

    #set_user_cart_coord(?, ?, pos=?)               arg =2 , kargs =1    _nType=1

    #set_user_cart_coord(?, ?, ?, pos=?)            arg =3 , kargs =1    _nType=0
    #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=1
    #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=2  

    #set_user_cart_coord(?, ?, ?, ?)                arg =4 , kargs =0    _nType=0

#----- 신규 추가 명령 2019/11/27 ----------------------------------------------------------------------------------------------------------------
    #set_user_cart_coord(pos=?, ref=?)         arg =0 , kargs =2    _nType=5
    #set_user_cart_coord(pos=?, ?)             arg =1 , kargs =1 *  _nType=5 #python syntax error : positional argument follows keyword argument
    #set_user_cart_coord(?, ref=?)             arg =1 , kargs =1 *  _nType=5
    #set_user_cart_coord(?, ?)                 arg =2 , kargs =0    _nType=5
#------------------------------------------------------------------------------------------------------------------------------------------------

    if len_args == 0 and len_kargs ==2:
        print("new commnad  len_args == 0 and len_kargs ==2")
        #_pos
        #_ref
        print("_pos={},_ref={}".format(_pos, _ref))
        _nType = 5
    elif len_args == 1 and len_kargs ==1:
        print("new commnad  len_args == 1 and len_kargs ==1")
        _pos = args[0]
        #_ref
        print("_pos={},_ref={}".format(_pos, _ref))
        _nType = 5
    elif len_args == 2 and len_kargs ==0:
        print("new commnad  len_args == 2 and len_kargs ==0")
        _pos = args[0]
        _ref = args[1]
        print("_pos={},_ref={}".format(_pos, _ref))
        _nType = 5
    else:
        if len_args == 0:
            if len_kargs == 5:
                if _x1 == None or _x2 == None or _x3 == None or _pos == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _nType = 3
            elif len_kargs == 4:
                if _ref != None:
                    _nType = 4
                else:
                    if _x1 == None or _x2 == None or _x3 == None or _pos == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 0
            elif len_kargs == 3:
                if _u1 != None: 
                    if _v1 == None or _pos == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 1
                elif _x1 != None:
                    if _x2 == None or _x3 == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 2
                else: 
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 1:
            if len_kargs == 3:
                if _x1 == None or _x2 == None or _x3 == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _nType = 0
                _pos= args[0]
            elif len_kargs == 2:
                if _u1 == None or _v1 == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _nType = 1
                _pos= args[0]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 2:
            if len_kargs == 1:
                if _pos != None: 
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                else:
                    _nType = 1
                    _u1 = args[0]
                    _v1 = args[1]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 3:
            if len_kargs == 1:
                if _ref != None:
                    _nType = 4
                    _u1 = args[0]
                    _v1 = args[1]
                    _pos = args[2]
                else:
                    if _pos != None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    else:
                        _nType = 0
                        _x1 = args[0]
                        _x2 = args[1]
                        _x3 = args[2]
            elif len_kargs == 0:
                if len(args[0]) == 3:
                    _nType = 1
                    _u1 = args[0]
                    _v1 = args[1]
                    _pos= args[2]
                elif len(args[0]) == 6:
                    _nType = 2
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 4:
            if len_kargs == 0:
                _nType = 0
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos= args[3]
            elif len_kargs == 1:
                _nType = 3
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos= args[3]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 5:
            if len_kargs == 0:
                _nType = 0
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos= args[3]
                _ref= args[4]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # check parameter type, value
    if _x1 != None: 
        _x1 = get_posx(_x1)
    if _x2 != None: 
        _x2 = get_posx(_x2)
    if _x3 != None: 
        _x3 = get_posx(_x3)
    if _pos != None: 
        _pos = get_posx(_pos)
    if _u1 !=None:
        if type(_u1) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : u1")
        if len(_u1) != DR_VECTOR_U1_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : u1({0})".format(_u1))
    if _v1 !=None:
        if type(_v1) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : v1")
        if len(_v1) != DR_VECTOR_V1_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : v1({0})".format(_v1))

    # C function call
    if _nType == 0:
        _ref = DR_BASE
        if __ROS__:
            srv = _ros_set_user_cart_coord2(_x1, _x2, _x3, _pos, _ref)  
            ret = srv.id
        else:
            ret = PythonMgr.py_set_user_cart_coord(_x1, _x2, _x3, _pos, _ref)
            #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord(x1:{1}, x2:{2}, x3:{3}, pos:{4}, ref:{5})".format(ret, _x1, _x2, _x3, _pos, _ref))
    elif _nType == 1:
        _ref = DR_BASE
        if __ROS__:
            srv = _ros_set_user_cart_coord3(_u1, _v1, _pos, _ref)  
            ret = srv.id
        else:
            ret = PythonMgr.py_set_user_cart_coord_ex(_u1, _v1, _pos, _ref)
            #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex(u1:{1}, v1:{2}, pos:{3}, ref:{4})".format(ret, _u1, _v1, _pos, _ref))
    elif _nType == 2:   #현재 미사용
        #_ref = DR_BASE
        #if __ROS__:
        #    srv = _ros_set_user_cart_coord?(_x1, _x2, _x3, _ref)  
        #    ret = srv.id
        #else:
        #    ret = PythonMgr.py_set_user_cart_coord_ex2(_x1, _x2, _x3, _ref)
        #    #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex2(x1:{1}, x2:{2}, x3:{3}, ref:{4})".format(ret, _x1, _x2, _x3, _ref))
        pass
    elif _nType == 3:   # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS__:
            srv = _ros_set_user_cart_coord2(_x1, _x2, _x3, _pos, _ref)  
            ret = srv.id
        else:
            ret = PythonMgr.py_set_user_cart_coord(_x1, _x2, _x3, _pos, _ref)
            #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord(x1:{1}, x2:{2}, x3:{3}, pos:{4}, ref:{5})".format(ret, _x1, _x2, _x3, _pos, _ref))
    elif _nType == 4:   # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS__:
            srv = _ros_set_user_cart_coord3(_u1, _v1, _pos, _ref)  
            ret = srv.id
        else:
            ret = PythonMgr.py_set_user_cart_coord_ex(_u1, _v1, _pos, _ref)
            #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex(u1:{1}, v1:{2}, pos:{3}, ref:{4})".format(ret, _u1, _v1, _pos, _ref))
    elif _nType == 5:   # (pos, ref) 20191127 신규 추가 
        #print("new command !!!!!!!!!!!!!!!!")
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS__:
            srv = _ros_set_user_cart_coord1(_pos, _ref)  
            ret = srv.id
        else:
            ret = PythonMgr.py_set_user_cart_coord_ex3(_pos, _ref)
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    return ret

def overwrite_user_cart_coord(id, pos, ref=None, gol=None):

    # id
    _id = id

    # pos
    _pos = get_posx(pos)

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # gol : #1 RCT 요청에 의하여 gol 무조건 0으로 변경 2019/12/09
    #if gol == None:
    #    _gol = 0
    #else:
    #    _gol = gol
    #    if _gol != 0 and _gol != 1:
    #        raise DR_Error(DR_ERROR_VALUE, "Invalid value : gol")
    _gol = 0

    # C function call
    if __ROS__:
        #srv = _ros_overwrite_user_cart_coord(_id, _pos, _ref, _gol)  
        srv = _ros_overwrite_user_cart_coord(_id, _pos, _ref)
        ret = srv.id
    else:
        ret = PythonMgr.py_overwrite_user_cart_coord(_id, _pos, _ref, _gol)
        if PY_EXT_RET_UNSUPPORT_CMD == ret:
            raise DR_Error(DR_ERROR_TYPE, "unsupported command")

    return ret

def get_user_cart_coord(id):

    print("==============================> get_user_cart_coord(id)")
    if __ROS__:
        print("==============================> get_user_cart_coord(id) 111")
        srv = _ros_get_user_cart_coord(id)  

        print("==============================> get_user_cart_coord(id) 222")
        pos = list(srv.conv_posx)  # Convert tuple to list 

        print("==============================> get_user_cart_coord(id) 333")
        ref = srv.ref
        print("==============================> get_user_cart_coord(id) 444")
    else:
        # C function call
        ret = PythonMgr.py_get_user_cart_coord(id)
        if PY_EXT_RET_UNSUPPORT_CMD == ret:
            raise DR_Error(DR_ERROR_TYPE, "unsupported command")
        # check return
        _check_ext_result(ret)
        # wait for job to be finished
        proc_id = ret
        _wait_result(proc_id)
        # get result
        pos, ref  = PythonMgr.py_get_result(proc_id)
    conv_posx = posx(pos)

    print("==============================> get_user_cart_coord(id) 555 conv_posx={}, ref={}".format(conv_posx, ref))
    return conv_posx, ref

def set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[0, 0, 0, 0, 0, 0], time=0, mod=DR_FC_MOD_ABS):
    # df
    if type(fd) != list or len(fd) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : fd")

    if is_number(fd) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : fd({0})".format(fd))

    # dir
    if type(dir) != list or len(dir) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : dir")

    if is_number(dir) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : dir({0})".format(dir))

    # _ref
    _ref = _g_coord

    # time
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:     # -1 : motion default
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    # _mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    _mod = mod

    if __ROS__:
        srv = _ros_set_desired_force(fd, dir, _ref, _time, _mod)  
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_set_desired_force(fd, dir, _ref, _time, _mod)
        #print_ext_result("{0} = PythonMgr.py_set_desired_force(fd:{1}, dir:{2}, ref:{3}, time:{4}, mod:{5})" \
        #                 .format(ret, dr_form(fd), dr_form(dir), _ref, dr_form(_time), _mod))

    return ret

def release_force(time=0):
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    if __ROS__:
        srv = _ros_release_force(_time)  
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_release_force(_time)
        #print_ext_result("{0} = PythonMgr.py_release_force({1})".format(ret, dr_form(_time)))

    return ret

#def check_position_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
def check_position_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None, mod= DR_MV_MOD_ABS, pos=None):
    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    # min
    if type(min) != int and type(min) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")

    # max
    if type(max) != int and type(max) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

    if min == DR_COND_NONE and max == DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))

    # min < max check
    if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
        if min > max:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(mod))

    # _pos : DR_MV_MOD_REL 인 경우에는 반드시 pos 가 필요하고, otherwise 불필요
    if(mod == DR_MV_MOD_REL):
        if(pos==None):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
        else:
            _pos = get_normal_pos(pos, def_type=posx)
    else:
        if(pos!=None):
            _pos = get_normal_pos(pos, def_type=posx)
        else:
            pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            _pos = get_normal_pos(pos, def_type=posx)

    # check axis
    if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
        _ref = DR_TOOL

    if __ROS__:
        srv = _ros_check_position_condition(axis, min, max, _ref, mod, _pos)  
        ret = srv.success   #True or False 
    else:
        # C function call
        ret = PythonMgr.py_check_position_condition(axis, min, max, _ref, mod, _pos)
        #print_ext_result("{0} = PythonMgr.py_check_position_condition(axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6})" \
        #                 .format(ret, axis, dr_form(min), dr_form(max), _ref, mod, _pos))

    return ret

def check_force_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
        axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    # min
    if type(min) != int and type(min) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")

    # max
    if type(max) != int and type(max) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

    if min == DR_COND_NONE and max == DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

    # min check : min 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
    if min != DR_COND_NONE:
        if min < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min (Ranges: min({0}) >= 0)".format(min))

    # max check : max 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
    if max != DR_COND_NONE:
        if max < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: max (Ranges: max({0}) >= 0)".format(max))

    # min < max check
    if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
        if min > max:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # check axis
    if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
        _ref = DR_TOOL

    if __ROS__:
        srv = _ros_check_force_condition(axis, min, max, _ref)  
        ret = srv.success   #True or False 
    else:
        # C function call
        ret = PythonMgr.py_check_force_condition(axis, min, max, _ref)
        #print_ext_result("{0} = PythonMgr.py_check_force_condition(axis:{1}, min:{2}, max:{3}, ref:{4})" \
        #                 .format(ret, axis, dr_form(min), dr_form(max), _ref))

    return ret

def check_orientation_condition(axis, min=None, max=None, ref=None, mod = None, pos=None):
    _cmd_type = 0

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    # min, max check type
    if(min != None):
        if type(min) == posx:
            _cmd_type = 0
        elif type(min) == posj:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min is not posj")
        elif type(min) == list and len(min) == POINT_COUNT:
            _cmd_type = 0
        elif type(min) == int or type(min) == float:
            _cmd_type = 1
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
    if(max != None):
        if type(max) == posx:
            _cmd_type = 0
        elif type(max) == posj:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max is not posj")
        elif type(max) == list and len(max) == POINT_COUNT:
            _cmd_type = 0
        elif type(max) == int or type(max) == float:
            _cmd_type = 1
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

    if _cmd_type == 0:

        # _pos_min
        if(min != None):
            _pos_min = get_normal_pos(min, def_type=posx)
        else:
            _pos_min = [DR_COND_NONE]*POINT_COUNT

        # _pos_max
        if(max != None):
            _pos_max = get_normal_pos(max, def_type=posx)
        else:
            _pos_max = [DR_COND_NONE]*POINT_COUNT

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # mod
        if mod == None:
            mod = DR_MV_MOD_ABS
        else:
            if type(mod) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")

            if mod != DR_MV_MOD_ABS:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_ABS".format(mod))

        print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}".format(_cmd_type, axis, _pos_min, _pos_max, _ref, mod) )

    elif _cmd_type == 1:
        # min
        if min == None:
            min = DR_COND_NONE
        else:
            if type(min) != int and type(min) != float:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
        # max
        if max == None:
            max = DR_COND_NONE
        else:
            if type(max) != int and type(max) != float:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

        if min == DR_COND_NONE and max == DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

        # min < max check
        if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
            if min > max:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # mod
        if mod == None:
            mod = DR_MV_MOD_REL
        else:
            if type(mod) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")

            if mod != DR_MV_MOD_REL:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_REL".format(mod))

        # _pos
        if(pos==None):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
        else:
            _pos = get_normal_pos(pos, def_type=posx)

        #print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6}".format(_cmd_type, axis, min, max, _ref, mod, _pos) )

    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")


    # C function call
    if _cmd_type == 0:
        if __ROS__:
            srv = _ros_check_orientation_condition1(axis, _pos_min, _pos_max, _ref, mod)  
            ret = srv.success   #True or False 
        else:
            ret = PythonMgr.py_check_orientation_condition(axis, _pos_min, _pos_max, _ref, mod)
            #print_ext_result("{0} = PythonMgr.py_check_orientation_condition(axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5})" \
            #                 .format(ret, axis, _pos_min, _pos_max, _ref, mod))
    else:
        if __ROS__:
            srv = _ros_check_orientation_condition2(axis, min, max, _ref, mod, _pos)  
            ret = srv.success   #True or False 
        else:
            ret = PythonMgr.py_check_orientation_condition2(axis, min, max, _ref, mod, _pos)
            #print_ext_result("{0} = PythonMgr.py_check_orientation_condition2(axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6})" \
            #                .format(ret, axis, dr_form(min), dr_form(max), _ref, mod, _pos))

    return ret

def coord_transform(pose_in, ref_in=DR_BASE, ref_out=None):

    # _pos
    _pos = get_normal_pos(pose_in, def_type=posx)

    # ref_in
    if type(ref_in) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_in")

    # ref_out
    if type(ref_out) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")

    if __ROS__:
        srv = _ros_coord_transform(_pos, ref_in, ref_out)  
        pos = list(srv.conv_posx)  # Convert tuple to list 
    else:
        # C function call
        ret = PythonMgr.py_coord_transformed_posx(_pos, ref_in, ref_out)
        #print_ext_result("{0} = PythonMgr.py_coord_transformed_posx(pose_in:{1},ref_in:{2},ref_out:{3})".format(ret, _pos, ref_in, ref_out))
        # check return
        _check_ext_result(ret)
        # wait for job to be finished
        proc_id = ret
        _wait_result(proc_id)
        # get result
        pos = PythonMgr.py_get_result(proc_id)
 
    trans_posx = posx(pos)

    #print_result("{0} = coord_transform(pose_in:{1},ref_in:{2},ref_out:{3})".format(trans_posx, _pos, ref_in, ref_out))
    return trans_posx




##### I/O #########################################################################################################################

def get_digital_input(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    # ROS service call
    if __ROS__:
        srv = _ros_get_digital_input(index)
        value = srv.value
    else:
        value = PythonMgr.py_get_digital_input(index)
        print_ext_result("{0} = PythonMgr.py_get_digital_input(index:{1})".format(value, index))
    return value

def get_analog_input(ch):
    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    # ROS service call
    if __ROS__:
        srv = _ros_get_analog_input(ch)
        value = srv.value
    else:
        value = PythonMgr.py_get_analog_input(ch)
        print_ext_result("{0} = PythonMgr.py_get_analog_input(index:{1})".format(value, ch))
    return value

def get_tool_digital_input(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    # ROS service call
    if __ROS__:
        srv = _ros_get_tool_digital_input(index)
        value = srv.value
    else:
        value = PythonMgr.py_get_tool_digital_input(index)
        print_ext_result("{0} = PythonMgr.py_get_tool_digital_input(index:{1})".format(value, index))
    return value

def set_digital_output(index, val=None):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if val != None:
        if type(val) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")
    
        if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
    
        if val != ON and val != OFF:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    else: # # val ���ڰ� ������ simple style
        if (index < (-DR_DIO_MAX_INDEX)) or (index > DR_DIO_MAX_INDEX) or (index==0): # -16~+16
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
        else:
            if index < 0:
                index = index*(-1) 
                val = OFF
            else:
                index = index
                val = ON

    # ROS service call
    if __ROS__:
        srv = _ros_set_digital_output(index, val) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_digital_output(index, val)
        print_ext_result("{0} = PythonMgr.py_set_digital_output(index:{1}, val:{2})".format(ret, index, val))
    return ret

def get_digital_output(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    # ROS service call
    if __ROS__:
        srv = _ros_get_digital_output(index)
        value = srv.value
    else:
        value = PythonMgr.py_get_digital_output(index)
        print_ext_result("{0} = PythonMgr.py_get_digital_output(index:{1})".format(value, index))
    return value


def set_analog_output(ch, val):
    global _g_analog_output_mode_ch1
    global _g_analog_output_mode_ch2

    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if type(val) != int and type(val) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    #if val < 0 or val > 20.0:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    if ch == 1:
        if _g_analog_output_mode_ch1 == DR_ANALOG_CURRENT:
            if val < 4 or val > 20.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        elif _g_analog_output_mode_ch1 == DR_ANALOG_VOLTAGE:
            if val < 0 or val > 10.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: 
            raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch1) is not set")
    if ch == 2:
        if _g_analog_output_mode_ch2 == DR_ANALOG_CURRENT:
            if val < 4 or val > 20.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        elif _g_analog_output_mode_ch2 == DR_ANALOG_VOLTAGE:
            if val < 0 or val > 10.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: 
            raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch2) is not set")

    # ROS service call
    if __ROS__:
        srv = _ros_set_analog_output(ch, val) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_analog_output(ch, val)
        print_ext_result("{0} = PythonMgr.py_set_analog_output(ch:{1}, val:{2})".format(ret, ch, val))   
    return ret

def set_mode_analog_output(ch, mod):
    global _g_analog_output_mode_ch1
    global _g_analog_output_mode_ch2

    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    if ch == 1: 
        _g_analog_output_mode_ch1 = mod
    if ch == 2: 
        _g_analog_output_mode_ch2 = mod

    # ROS service call
    if __ROS__:
        srv = _ros_set_mode_analog_output(ch, mod)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_mode_analog_output(ch, mod)
        print_ext_result("{0} = PythonMgr.py_set_mode_analog_output(ch:{1}, mod:{2})".format(ret, ch, mod))
    return ret

def set_mode_analog_input(ch, mod):
    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ROS service call
    if __ROS__:
        srv = _ros_set_mode_analog_input(ch, mod)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_mode_analog_input(ch, mod)
        print_ext_result("{0} = PythonMgr.py_set_mode_analog_input(ch:{1}, mod:{2})".format(ret, ch, mod))
    return ret

def set_tool_digital_output(index, val=None):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if val != None:
        if type(val) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

        if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

        if val != ON and val != OFF:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    else: # val ���ڰ� ������ simple style
        if (index < (-DR_TDIO_MAX_INDEX)) or (index > DR_TDIO_MAX_INDEX) or (index==0): # -6~+6
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
        else:
            if index < 0:
                index = index*(-1) 
                val = OFF
            else:
                index = index
                val = ON

    # ROS service call
    if __ROS__:
        srv = _ros_set_tool_digital_output(index, val)   
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_tool_digital_output(index, val)
        print_ext_result("{0} = PythonMgr.py_set_tool_digital_output(index:{1}, val:{2})".format(ret, index, val))
    return ret

def get_tool_digital_output(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    # ROS service call
    if __ROS__:
        srv = _ros_get_tool_digital_output(index)
        value = srv.value
    else:
        value = PythonMgr.py_get_tool_digital_output(index)
        print_ext_result("{0} = PythonMgr.py_get_tool_digital_output(index:{1})".format(value, index))
    return value


##### Modbus #########################################################################################################################

def add_modbus_signal(ip, port, name, reg_type, index, value=0, slaveid=255):
    # ip
    if type(ip) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ip")

    #try:
    #    ipaddress.ip_address(ip)
    #except:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : ip")

    # port
    if type(port) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : port")

    if port <= 0 or port > 65535:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : port (Ranges: 1 ~ 65535)")

    # name
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # reg_type
    if type(reg_type) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : reg_type")

    if reg_type != DR_MODBUS_DIG_INPUT and reg_type != DR_MODBUS_DIG_OUTPUT and reg_type != DR_MODBUS_REG_INPUT and reg_type != DR_MODBUS_REG_OUTPUT:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : reg_type (Ranges : 0 ~ 3")

    # index
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < 0 or index > 65535:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index (Ranges: 0 ~ 65535)")

    # value
    if type(value) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : value")

    if value < 0 or value > 65535:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : value (Ranges: 0 ~ 65535)")

    # check value
    if reg_type == DR_MODBUS_DIG_OUTPUT or reg_type == DR_MODBUS_REG_OUTPUT:
        _value = value
    else:
        _value = 0

    # slaveid
    if type(slaveid) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : slaveid")
    if slaveid < 0 or slaveid > 255:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")
    elif slaveid > 247 and slaveid < 255:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")

    # ROS service call
    if __ROS__:
        srv = _ros_add_modbus_signal(name, ip, port, reg_type, index, _value, slaveid)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_add_modbus_signal(ip, port, name, reg_type, index, _value, slaveid)
        print_ext_result("{0} = PythonMgr.py_add_modbus_signal(ip:{1}, port:{2}, name:{3}, type:{4}, index:{5}, value:{6}, slaveid:{7})" \
                         .format(ret, ip, port, name, reg_type, index, _value, slaveid))    
    return ret

def del_modbus_signal(name):
    # name
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        srv = _ros_del_modbus_signal(name) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_del_modbus_signal(name)
        print_ext_result("{0} = PythonMgr.py_del_modbus_signal(name:{1})".format(ret, name))
    return ret

def set_modbus_output(iobus, val):
    if type(iobus) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

    if type(val) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

    # ROS service call
    if __ROS__:
        srv = _ros_set_modbus_output(iobus, val) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_modbus_output(iobus, val)
        print_ext_result("{0} = PythonMgr.py_set_modbus_output(iobus:{1}, val:{2})".format(ret, iobus, val))
    return ret

def get_modbus_input(iobus):
    if type(iobus) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

    # ROS service call
    if __ROS__:
        srv = _ros_get_modbus_input(iobus)
        value = srv.value
    else:    
        value = PythonMgr.py_get_modbus_input(iobus)
        print_ext_result("{0} = PythonMgr.py_get_modbus_input(iobus:{1})".format(value, iobus))
    return value


#################################################################################################################################################
def set_tcp(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

    # ROS service call
    if __ROS__:
        srv = _ros_set_current_tcp(name) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_tcp(name)
        print_ext_result("{0} = PythonMgr.py_set_tcp(name:{1})".format(ret, name))
    return ret

def get_tcp():
    # ROS service call
    if __ROS__:
        srv = _ros_get_current_tcp()        
    return srv.info

def set_tool(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

    # ROS service call
    if __ROS__:
        srv = _ros_set_current_tool(name)    
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        ret = PythonMgr.py_set_tool(name)
        print_ext_result("{0} = PythonMgr.py_set_tool(name:{1})".format(ret, name))
    return ret

def get_tool():
    # ROS service call
    if __ROS__:
        srv = _ros_get_current_tool() 
    return srv.info

def set_tool_shape(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    if __ROS__:
        srv = _ros_set_tool_shape(name)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    else:
        # C function call
        ret = PythonMgr.py_set_tool_shape(name)

    return ret

def add_tcp(name, pos):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        srv = _ros_config_create_tcp(name, pos)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def del_tcp(name):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        srv = _ros_config_delete_tcp(name)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def add_tool(name, weight, cog, inertia):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        srv = _ros_config_create_tool(name, weight, cog, inertia) 
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def del_tool(name):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        srv = _ros_config_delete_tool(name)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

###########################################################################################################

def drl_script_run(robotSystem, code):
    if type(code) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        srv = _ros_drl_start(robotSystem, code)
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def drl_script_stop(stop_mode):
    if type(stop_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : stop_mode")
    print("drl_script_stop")
    # ROS service call
    if __ROS__:
        srv = _ros_drl_stop(stop_mode)  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def drl_script_pause():
    # ROS service call
    if __ROS__:
        srv = _ros_drl_pause()  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def drl_script_resume():
    # ROS service call
    if __ROS__:
        srv = _ros_drl_resume()  
        #ret = srv.success
        ret = 0 if (srv.success == True) else -1
    return ret

def get_drl_state():
    # ROS service call
    if __ROS__:
        srv = _ros_get_drl_state()  
        ret = srv.drl_state
    return ret


########################################################################################################################################
########################################################################################################################################
########################################################################################################################################
class CDsrRobot:
    def __init__(self, robot_id='dsr01', robot_model='m1013'):
        self._robot_id = robot_id
        self._robot_model = robot_model
        print("CDsrRobot(): self._robot_id = %s, self._robot_model = %s" %(self._robot_id, self._robot_model))

        self._srv_name_prefix   = '/' + self._robot_id + self._robot_model


        ############### connect to dsr_control (ros service) ####################################################################### 
        #rospy.wait_for_service(self._srv_name_prefix +"/motion/move_joint")
        
        # system Operations
        self._ros_set_robot_mode            = rospy.ServiceProxy(self._srv_name_prefix +"/system/set_robot_mode", SetRobotMode)
        self._ros_get_robot_mode            = rospy.ServiceProxy(self._srv_name_prefix +"/system/get_robot_mode", GetRobotMode)
        self._ros_set_robot_system          = rospy.ServiceProxy(self._srv_name_prefix +"/system/set_robot_system", SetRobotSystem)
        self._ros_get_robot_system          = rospy.ServiceProxy(self._srv_name_prefix +"/system/get_robot_system", GetRobotSystem)
        self._ros_get_robot_state           = rospy.ServiceProxy(self._srv_name_prefix + "/system/get_robot_state", GetRobotState)
        self._ros_set_robot_speed_mode      = rospy.ServiceProxy(self._srv_name_prefix +"/system/set_robot_speed_mode", SetRobotSpeedMode)
        self._ros_get_robot_speed_mode      = rospy.ServiceProxy(self._srv_name_prefix +"/system/get_robot_speed_mode", GetRobotSpeedMode)
        self._ros_set_safe_stop_reset_type  = rospy.ServiceProxy(self._srv_name_prefix +"/system/set_safe_stop_reset_type", SetSafeStopResetType)
        self._ros_get_last_alarm            = rospy.ServiceProxy(self._srv_name_prefix +"/system/get_last_alarm", GetLastAlarm)
        self._ros_get_current_pose          = rospy.ServiceProxy(self._srv_name_prefix +"/system/get_current_pose", GetCurrentPose)
        self._ros_set_robot_control         = rospy.ServiceProxy(self._srv_name_prefix +"/system/set_robot_control", SetRobotControl)
        self._ros_manage_access_control     = rospy.ServiceProxy(self._srv_name_prefix +"/system/manage_access_control", ManageAccessControl)
        

        #  motion Operations
        self._ros_movej                      = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_joint", MoveJoint)
        self._ros_movel                      = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_line", MoveLine)
        self._ros_movejx                     = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_jointx", MoveJointx)
        self._ros_movec                      = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_circle", MoveCircle)
        self._ros_movesj                     = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_spline_joint", MoveSplineJoint)
        self._ros_movesx                     = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_spline_task", MoveSplineTask)
        self._ros_moveb                      = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_blending", MoveBlending)
        self._ros_move_spiral                = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_spiral", MoveSpiral)
        self._ros_move_periodic              = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_periodic", MovePeriodic)
        self._ros_move_wait                  = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_wait", MoveWait)
        self._ros_jog                        = rospy.ServiceProxy(self._srv_name_prefix +"/motion/jog", Jog)
        self._ros_jog_multi                  = rospy.ServiceProxy(self._srv_name_prefix +"/motion/jog_multi", JogMulti)
        self._ros_trans                      = rospy.ServiceProxy(self._srv_name_prefix +"/motion/trans", Trans)

        self._ros_fkin                       = rospy.ServiceProxy(self._srv_name_prefix +"/motion/fkin", Fkin)
        self._ros_ikin                       = rospy.ServiceProxy(self._srv_name_prefix +"/motion/ikin", Ikin)
        self._ros_set_ref_coord              = rospy.ServiceProxy(self._srv_name_prefix +"/motion/set_ref_coord", SetRefCoord)
        self._ros_move_home                  = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_home", MoveHome)
        self._ros_check_motion               = rospy.ServiceProxy(self._srv_name_prefix +"/motion/check_motion", CheckMotion)
        self._ros_change_operation_speed     = rospy.ServiceProxy(self._srv_name_prefix +"/motion/change_operation_speed", ChangeOperationSpeed)
        self._ros_enable_alter_motion        = rospy.ServiceProxy(self._srv_name_prefix +"/motion/enable_alter_motion", EnableAlterMotion)
        self._ros_alter_motion               = rospy.ServiceProxy(self._srv_name_prefix +"/motion/alter_motion", AlterMotion)
        self._ros_disable_alter_motion       = rospy.ServiceProxy(self._srv_name_prefix +"/motion/disable_alter_motion", DisableAlterMotion)
        self._ros_move_home                  = rospy.ServiceProxy(self._srv_name_prefix +"/motion/move_home", MoveHome)
        self._ros_check_motion               = rospy.ServiceProxy(self._srv_name_prefix +"/motion/check_motion", CheckMotion)
        self._ros_change_operation_speed     = rospy.ServiceProxy(self._srv_name_prefix +"/motion/change_operation_speed", ChangeOperationSpeed)
        self._ros_enable_alter_motion        = rospy.ServiceProxy(self._srv_name_prefix +"/motion/enable_alter_motion", EnableAlterMotion)
        self._ros_alter_motion               = rospy.ServiceProxy(self._srv_name_prefix +"/motion/alter_motion", AlterMotion)
        self._ros_disable_alter_motion       = rospy.ServiceProxy(self._srv_name_prefix +"/motion/disable_alter_motion", DisableAlterMotion)

        #  Auxiliary Control Operations
        self._ros_get_control_mode               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_control_mode", GetControlMode)
        self._ros_get_control_space              = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_control_space", GetControlSpace)

        self._ros_get_current_posj               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_current_posj", GetCurrentPosj)
        self._ros_get_current_velj               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_current_velj", GetCurrentVelj)
        self._ros_get_desired_posj               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_desired_posj", GetDesiredPosj)
        self._ros_get_desired_velj               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_desired_velj", GetDesiredVelj)

        self._ros_get_current_posx               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_current_posx", GetCurrentPosx)
        self._ros_get_current_velx               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_current_velx", GetCurrentVelx)    
        self._ros_get_desired_posx               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_desired_posx", GetDesiredPosx)
        self._ros_get_desired_velx               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_desired_velx", GetDesiredVelx)

        self._ros_get_current_tool_flange_posx   = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_current_tool_flange_posx", GetCurrentToolFlangePosx)
        self._ros_get_current_solution_space     = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_current_solution_space", GetCurrentSolutionSpace)
        self._ros_get_current_rotm               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_current_rotm", GetCurrentRotm)    
        self._ros_get_joint_torque               = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_joint_torque", GetJointTorque)
        self._ros_get_external_torque            = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_external_torque", GetExternalTorque)
        self._ros_get_tool_force                 = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_tool_force", GetToolForce)
        self._ros_get_solution_space             = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_solution_space", GetSolutionSpace)
        self._ros_get_orientation_error          = rospy.ServiceProxy(self._srv_name_prefix +"/aux_control/get_orientation_error", GetOrientationError)


        #  Force/Stiffness Control & others Operations
        self._ros_get_workpiece_weight        = rospy.ServiceProxy(self._srv_name_prefix +"/force/get_workpiece_weight", GetWorkpieceWeight)
        self._ros_reset_workpiece_weight      = rospy.ServiceProxy(self._srv_name_prefix +"/force/reset_workpiece_weight", ResetWorkpieceWeight)
        self._ros_set_singularity_handling    = rospy.ServiceProxy(self._srv_name_prefix +"/motion/set_singularity_handling", SetSingularityHandling)

        self._ros_parallel_axis1              = rospy.ServiceProxy(self._srv_name_prefix +"/force/parallel_axis1", ParallelAxis1)
        self._ros_parallel_axis2              = rospy.ServiceProxy(self._srv_name_prefix +"/force/parallel_axis2", ParallelAxis2)
        self._ros_align_axis1                 = rospy.ServiceProxy(self._srv_name_prefix +"/force/align_axis1", AlignAxis1)
        self._ros_align_axis2                 = rospy.ServiceProxy(self._srv_name_prefix +"/force/align_axis2", AlignAxis2)
        self._ros_is_done_bolt_tightening     = rospy.ServiceProxy(self._srv_name_prefix +"/force/is_done_bolt_tightening", IsDoneBoltTightening)
        self._ros_release_compliance_ctrl     = rospy.ServiceProxy(self._srv_name_prefix +"/force/release_compliance_ctrl", ReleaseComplianceCtrl)
        self._ros_task_compliance_ctrl        = rospy.ServiceProxy(self._srv_name_prefix +"/force/task_compliance_ctrl", TaskComplianceCtrl)
        self._ros_set_stiffnessx              = rospy.ServiceProxy(self._srv_name_prefix +"/force/set_stiffnessx", SetStiffnessx)
        self._ros_calc_coord                  = rospy.ServiceProxy(self._srv_name_prefix +"/force/calc_coord", CalcCoord)
        self._ros_set_user_cart_coord1        = rospy.ServiceProxy(self._srv_name_prefix +"/force/set_user_cart_coord1", SetUserCartCoord1)
        self._ros_set_user_cart_coord2        = rospy.ServiceProxy(self._srv_name_prefix +"/force/set_user_cart_coord2", SetUserCartCoord2)
        self._ros_set_user_cart_coord3        = rospy.ServiceProxy(self._srv_name_prefix +"/force/set_user_cart_coord3", SetUserCartCoord3)
        self._ros_overwrite_user_cart_coord   = rospy.ServiceProxy(self._srv_name_prefix +"/force/overwrite_user_cart_coord", OverwriteUserCartCoord)
        self._ros_get_user_cart_coord         = rospy.ServiceProxy(self._srv_name_prefix +"/force/get_user_cart_coord", GetUserCartCoord)
        self._ros_set_desired_force           = rospy.ServiceProxy(self._srv_name_prefix +"/force/set_desired_force", SetDesiredForce)
        self._ros_release_force               = rospy.ServiceProxy(self._srv_name_prefix +"/force/release_force", ReleaseForce)
        self._ros_check_position_condition    = rospy.ServiceProxy(self._srv_name_prefix +"/force/check_position_condition", CheckPositionCondition)
        self._ros_check_force_condition       = rospy.ServiceProxy(self._srv_name_prefix +"/force/check_force_condition", CheckForceCondition)
        self._ros_check_orientation_condition1= rospy.ServiceProxy(self._srv_name_prefix +"/force/check_orientation_condition1", CheckOrientationCondition1)
        self._ros_check_orientation_condition2= rospy.ServiceProxy(self._srv_name_prefix +"/force/check_orientation_condition2", CheckOrientationCondition2)
        self._ros_coord_transform             = rospy.ServiceProxy(self._srv_name_prefix +"/force/coord_transform", CoordTransform)

        #  GPIO Operations
        self._ros_set_digital_output         = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_digital_output", SetCtrlBoxDigitalOutput)
        self._ros_get_digital_input          = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_digital_input", GetCtrlBoxDigitalInput)
        self._ros_set_tool_digital_output    = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_tool_digital_output", SetToolDigitalOutput)
        self._ros_get_tool_digital_input     = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_tool_digital_input", GetToolDigitalInput)
        self._ros_set_analog_output          = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_analog_output", SetCtrlBoxAnalogOutput)
        self._ros_get_analog_input           = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_analog_input", GetCtrlBoxAnalogInput)
        self._ros_set_mode_analog_output     = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_analog_output_type", SetCtrlBoxAnalogOutputType)
        self._ros_set_mode_analog_input      = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_analog_input_type", SetCtrlBoxAnalogInputType)
        self._ros_get_digital_output         = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_digital_output", GetCtrlBoxDigitalOutput)
        self._ros_get_tool_digital_output    = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_tool_digital_output", GetToolDigitalOutput)

        #  Modbus Operations
        self._ros_set_modbus_output          = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/set_modbus_output", SetModbusOutput)
        self._ros_get_modbus_input           = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/get_modbus_input", GetModbusInput)
        self._ros_add_modbus_signal          = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/config_create_modbus", ConfigCreateModbus)
        self._ros_del_modbus_signal          = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/config_delete_modbus", ConfigDeleteModbus)

        # TCP Operations
        self._ros_set_current_tcp            = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/set_current_tcp", SetCurrentTcp)
        self._ros_get_current_tcp            = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/get_current_tcp", GetCurrentTcp)
        self._ros_config_create_tcp          = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/config_create_tcp", ConfigCreateTcp)
        self._ros_config_delete_tcp          = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/config_delete_tcp", ConfigDeleteTcp)

        # Tool Operations
        self._ros_set_current_tool           = rospy.ServiceProxy(self._srv_name_prefix +"/tool/set_current_tool", SetCurrentTool)
        self._ros_get_current_tool           = rospy.ServiceProxy(self._srv_name_prefix +"/tool/get_current_tool", GetCurrentTool)
        self._ros_config_create_tool         = rospy.ServiceProxy(self._srv_name_prefix +"/tool/config_create_tool", ConfigCreateTool)
        self._ros_config_delete_tool         = rospy.ServiceProxy(self._srv_name_prefix +"/tool/config_delete_tool", ConfigDeleteTool)

        # DRL Operations
        self._ros_drl_pause                  = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_pause", DrlPause)
        self._ros_drl_resume                 = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_resume", DrlResume)
        self._ros_drl_start                  = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_start", DrlStart)
        self._ros_drl_stop                   = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_stop", DrlStop)
        self._ros_get_drl_state              = rospy.ServiceProxy(self._srv_name_prefix +"/drl/get_drl_state", GetDrlState)
        ########################################################################################################################################

        self._g_blend_state = False
        self._g_blend_radius = 0.0

        self._g_velj = [0.0] * DR_VELJ_DT_LEN
        self._g_accj = [0.0] * DR_ACCJ_DT_LEN

        self._g_velx = [0.0] * DR_VELX_DT_LEN
        self._g_velx[0]= 0.0
        self._g_velx[1]= DR_COND_NONE

        self._g_accx = [0.0] * DR_ACCX_DT_LEN
        self._g_accx[0]= 0.0
        self._g_accx[1]= DR_COND_NONE

        self._g_coord = DR_BASE

        self._g_analog_output_mode_ch1 = -1
        self._g_analog_output_mode_ch2 = -1

    ##### SYSTEM ##############################################################################################################################
    def set_robot_mode(self, robot_mode):
        if type(robot_mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_mode")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_robot_mode(robot_mode)
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def get_robot_mode(self):
        # ROS service call
        if __ROS__:
            srv = self._ros_get_robot_mode()
        return srv.robot_mode
        
    def set_robot_system(self, robot_system):
        if type(robot_system) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_system")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_robot_system(robot_system)
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def get_robot_system(self):
        # ROS service call
        if __ROS__:
            srv = self._ros_get_robot_system()
        return srv.robot_system
    
    def get_robot_state(self):
        # ROS service call
        if __ROS__:
            srv = self._ros_get_robot_state()
        return srv.robot_state
    
    def set_robot_speed_mode(self, speed_mode):
        if type(speed_mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed_mode")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_robot_speed_mode(speed_mode)
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def get_robot_speed_mode(self):
        # ROS service call
        if __ROS__:
            srv = self._ros_get_robot_speed_mode()
        return srv.speed_mode
    
    def set_safe_stop_reset_type(self, reset_type):
        if type(reset_type) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : reset_type")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_safe_stop_reset_type(reset_type)
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def get_current_pose(self, space_type):
        if type(space_type) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : space_type")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_current_pose(space_type)
        return srv.pos
    
    def set_robot_control(self, robot_control):
        if type(robot_control) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_control")

        # ROS service call
        if __ROS__:
            srv = self._ros_set_robot_control(robot_control)
            ret = 0 if (srv.success == True) else -1
        return ret

    def manage_access_control(self, access_control):
        if type(access_control) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : access_control")

        # ROS service call
        if __ROS__:
            srv = self._ros_manage_access_control(access_control)
            ret = 0 if (srv.success == True) else -1
        return ret

    def get_current_solution_space(self):
        # ROS service call
        if __ROS__:
            srv = self._ros_get_current_solution_space()            
        return srv.solution_space
    
    def get_last_alarm(self):
         #ROS service call
        if __ROS__:
            srv = self._ros_get_last_alarm()
        return srv.log_alarm
    
    ##### Auxiliary Control ##############################################################################################################################
    def get_control_mode(self):
        if __ROS__:
            srv = self._ros_get_control_mode()  
            mode = srv.control_mode
        else:
            # C function call
            mode = PythonMgr.py_get_control_mode()
            # check result
            _check_ext_result(mode)
    
        #print_result("{0} = get_control_mode()".format(mode))
        return mode
    
    def get_control_space(self):
        if __ROS__:
            srv = self._ros_get_control_space()  
            space = srv.space
        else:
            # C function call
            space = PythonMgr.py_get_control_space()
            # check result
            _check_ext_result(space)
    
        return space
    
    def get_current_posj(self):
        if __ROS__:
            srv = self._ros_get_current_posj()  
            pos = list(srv.pos)  # Convert tuple to list
        else:
            # C function call
            pos = PythonMgr.py_get_current_posj()
            # check result
            _check_ext_result(pos)
        
        # set posj
        cur_pos = posj(pos)
        #print_result("{0} = get_current_posj()".format(cur_pos))
    
        return cur_pos
    
    def get_current_velj(self):
        if __ROS__:
            srv = self._ros_get_current_velj()  
            vel = list(srv.joint_speed)  # Convert tuple to list
        else:
            # C function call
            vel = PythonMgr.py_get_current_velj()
            # check result
            _check_ext_result(vel)
    
        #print_result("{0} = get_current_velj()".format(dr_form(vel)))
    
        return vel
    
    def get_desired_posj(self):
        if __ROS__:
            srv = self._ros_get_desired_posj()  
            pos = list(srv.pos)  # Convert tuple to list
        else:
            # C function call
            pos = PythonMgr.py_get_desired_posj()
            # check result
            _check_ext_result(pos)
    
        # set posj
        desired_pos = posj(pos)
        #print_ext_result("{0} = get_desired_posj()".format(desired_pos))
    
        return desired_pos
    
    def get_desired_velj(self):
        if __ROS__:
            srv = self._ros_get_desired_velj()  
            vel = list(srv.joint_vel)  # Convert tuple to list
        else:
            # C function call
            vel = PythonMgr.py_get_desired_velj()
            # check result
            _check_ext_result(vel)
    
        #print_result("{0} = get_desired_velj()".format(dr_form(vel)))
    
        return vel
    
    def get_current_posx(self, ref=None):
    
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref != DR_BASE and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            
        if __ROS__:
            srv = self._ros_get_current_posx(_ref)  
            posx_info = _ros_Float64MultiArrayTolist(srv.task_pos_info) # Convert Float64MultiArray to list
            pos = []
            for i in range(POINT_COUNT):
                pos.append(posx_info[0][i])
            sol = int(round( posx_info[0][6] ))
        else:
            # C function call
            ret = PythonMgr.py_get_current_posx(_ref)
            # check return
            _check_ext_result(ret)
            # wait for job to be finished
            proc_id = ret
            _wait_result(proc_id)
            # get result
            pos, sol  = PythonMgr.py_get_result(proc_id)
    
        conv_posx = posx(pos)
        #print_result("({0}, {1}) = get_current_posx(ref:{2})".format(conv_posx, sol, _ref))
        return conv_posx, sol
        

    def get_current_tool_flange_posx(self, ref=None):
    
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        if __ROS__:
            srv = self._ros_get_current_tool_flange_posx(_ref)  
            pos = list(srv.pos)  # Convert tuple to list
        else:
            # C function call
            pos = PythonMgr.py_get_current_tool_flange_posx(_ref)
            # check result
            _check_ext_result(pos)
    
        # set posx type
        cur_pos = posx(pos)
        #print_result("{0} = get_current_tool_flange_posx(ref:{1})".format(cur_pos, _ref))
        return cur_pos
    
    def get_current_velx(self, ref=None):
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        if __ROS__:
            srv = self._ros_get_current_velx(_ref)  
            vel = srv.vel
        else:
            # C function call
            vel = PythonMgr.py_get_current_velx(_ref)
            # check result
            _check_ext_result(vel)
    
        #print_result("{0} = get_current_velx(ref:{1})".format(dr_form(vel), _ref))
        return vel
    
    def get_desired_posx(self, ref=None):
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        if __ROS__:
            srv = self._ros_get_desired_posx(_ref)  
            pos = list(srv.pos)  # Convert tuple to list
        else:
            # C function call
            ret = PythonMgr.py_get_desired_posx(_ref)
            # check return
            _check_ext_result(ret)
            # wait for job to be finished
            proc_id = ret
            _wait_result(proc_id)
            # get result
            pos = PythonMgr.py_get_result(proc_id)
    
        conv_posx = posx(pos)
    
        return conv_posx
    
    def get_desired_velx(self, ref=None):
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        if __ROS__:
            srv = self._ros_get_desired_velx(_ref)  
            vel = list(srv.vel)  # Convert tuple to list  
        else:
            # C function call
            vel = PythonMgr.py_get_desired_velx(_ref)
            # check result
            _check_ext_result(vel)
    
        #print_result("{0} = get_desired_velx(ref:{1})".format(dr_form(vel), _ref))
        return vel
    
    def get_current_solution_space(self):
        if __ROS__:
            srv = self._ros_get_current_solution_space()  
            sol_space = srv.sol_space  
        else:
            # C function call
            sol_space = PythonMgr.py_get_current_solution_space()
            # check result
            _check_ext_result(sol_space)
    
        #print_result("{0} = get_current_solution_space()".format(sol_space))
        return sol_space
    
    def get_current_rotm(self, ref=None):
    
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        if __ROS__:
            srv = self._ros_get_current_rotm(_ref)  
            rotm = _ros_Float64MultiArrayTolist(srv.rot_matrix) # Convert Float64MultiArray to list 
        else:
            # C function call
            rotm = PythonMgr.py_get_current_rotm(_ref)
            # check result
            _check_ext_result(rotm)
    
        #print_result("{0} = get_current_rotm(ref:{1})".format(dr_form(rotm), _ref))
        return rotm
    
    def get_joint_torque(self):
        if __ROS__:
            srv = self._ros_get_joint_torque()  
            torque = list(srv.jts)  # Convert tuple to list  
        else:
            # C function call
            torque = PythonMgr.py_get_joint_torque()
            # check result
            _check_ext_result(torque)
    
        #print_result("{0} = get_joint_torque()".format(dr_form(torque)))
        return torque
    
    def get_external_torque(self):
        if __ROS__:
            srv = self._ros_get_external_torque()  
            torque = list(srv.ext_torque)  # Convert tuple to list  
        else:
            # C function call
            torque = PythonMgr.py_get_external_torque()
            # check result
            _check_ext_result(torque)
    
        #print_result("{0} = get_external_torque()".format(dr_form(torque)))
        return torque
    
    def get_tool_force(self, ref=None):
    
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        if __ROS__:
            srv = self._ros_get_tool_force(_ref)  
            force = list(srv.tool_force)  # Convert tuple to list  
        else:
            # C function call
            force = PythonMgr.py_get_tool_force(_ref)
            # check result
            _check_ext_result(force)
    
        #print_result("{0} = get_tool_force(ref:{1})".format(dr_form(force), _ref))
        return force
    
    def get_solution_space(self, pos):
        _pos = get_posj(pos)
    
        if __ROS__:
            srv = self._ros_get_solution_space(_pos)  
            sol = srv.sol_space
        else:
            # C function call
            ret = PythonMgr.py_get_solution_space(_pos)
            # check return
            _check_ext_result(ret)
            # wait for job to be finished
            proc_id = ret
            _wait_result(proc_id)
            # get result
            sol = PythonMgr.py_get_result(proc_id)
    
        return sol
    
    def get_orientation_error(self, xd, xc, axis):
        # xd, xc
        _xd = get_posx(xd)
        _xc = get_posx(xc)
    
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    
        if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
            axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))
    
        if __ROS__:
            srv = self._ros_get_orientation_error(_xd, _xc, axis)  
            ret = srv.ori_error
        else:
            # C function call
            ret = PythonMgr.py_get_orientation_error(_xd, _xc, axis)
    
        return ret
    ##### MOTION ##############################################################################################################################
    def trans(self, pos, delta, ref=None, ref_out=DR_BASE):
        # pos, delta
        _pos = get_posx(pos)
        _delta = get_posx(delta)
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
        if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        # check ref_out
        if type(ref_out) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")
        if ref_out != DR_BASE and ref_out != DR_WORLD and (ref_out < DR_TC_USER_MIN or ref_out > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref_out({0})".format(ref_out))
        # ROS service call
        if __ROS__: 
            srv = self._ros_trans(_pos, _delta, _ref, ref_out)
            pos = srv.trans_pos
        else:   
            # C function call
            ret = PythonMgr.py_trans(_pos, _delta, _ref, ref_out)
            # check return
            _check_ext_result(ret)
            # wait for job to be finished
            proc_id = ret
            _wait_result(proc_id)
            # get result
            pos = PythonMgr.py_get_result(proc_id)
        trans_posx = posx(pos)
        return trans_posx
    
    def fkin(self, pos, ref=None):
    
        _pos = get_posj(pos)
    
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        # ROS service call
        if __ROS__: 
            srv = self._ros_fkin(_pos, _ref)
            ret = srv.conv_posx
        else:   
            ret = PythonMgr.py_fkin(_pos, _ref)
        return ret
    
    def ikin(self, pos, sol_space, ref=None):
    
        _pos = get_posx(pos)
    
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        # sol_space
        if type(sol_space) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol_space")
    
        if sol_space < DR_SOL_MIN or sol_space > DR_SOL_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol_space({0})".format(sol_space))
    
        # ROS service call
        if __ROS__: 
            srv = self._ros_ikin(_pos, sol_space, _ref)
            ret = srv.conv_posj
        else:   
            ret = PythonMgr.py_ikin(_pos, sol_space, _ref)
        return ret
    
    def set_ref_coord(self, coord):
        # coord
        if type(coord) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : coord")
    
        if coord != DR_BASE and coord != DR_TOOL and coord != DR_WORLD and not (DR_TC_USER_MIN <= coord <= DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : coord({0})".format(coord))
    
        # ROS service call
        if __ROS__: 
            srv = self._ros_set_ref_coord(coord)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:   
            ret = PythonMgr.py_set_ref_coord(coord)
    
        # set global accx
        global _g_coord
    
        self._g_coord = coord
    
        #print_result("{0} = set_ref_coord(coord:{1})".format(ret, coord))
        return ret
    def movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
        return ret
    def amovej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
        return ret
    def _movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_posj(pos)
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)
    
        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            #for ros global _g_blend_state
    
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0
    
        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")
    
        if _radius< 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")
    
        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__: 
            srv = self._ros_movej(_pos, _vel[0], _acc[0], _time, _radius, mod, ra, _async)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:   
            ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
        return ret
    
    def movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = self._movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=0)
        return ret 
    def amovejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = self._movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=1)
        return ret 
    def _movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_posx(pos)
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)
    
        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            #for ros global _g_blend_state
    
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0
    
        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")
    
        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")
    
        # _ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type, ra")
    
        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")
    
        # sol
        if type(sol) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol")
    
        if sol < DR_SOL_MIN or sol > DR_SOL_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__: 
            srv = self._ros_movejx(_pos, _vel[0], _acc[0], _time, _radius, _ref, mod, ra, sol, _async)   
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:    
            ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
            print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
        return ret
    
    def movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=0)
        return ret
    def amovel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=1)
        return ret
    def _movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_normal_pos(pos, def_type=posx)
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ���� 
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        #print("_vel[0]={0}, _vel[1]={1}".format(_vel[0],_vel[1]))
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        #print("_acc[0]={0}, _acc[1]={1}".format(_acc[0],_acc[1]))
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)
    
        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            #for ros global _g_blend_state
    
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0
    
        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")
    
        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")
    
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")
    
        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")
    
        # qcommand
        if type(_pos) == posx:
            qcommand = 0
        else:
            qcommand = 1
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__: 
            srv = self._ros_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, _async) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:    
            ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
        return ret
    
    def movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = self._movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=0)
        return ret
    def amovec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = self._movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=1)
        return ret
    def _movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, _async=0):
    
        # _pos1, _pos2
    
        _pos1 = get_normal_pos(pos1, def_type=posx)
        _pos2 = get_normal_pos(pos2, def_type=posx)
    
        if type(_pos1) != type(_pos2):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos1, ps2")
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)
    
        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            #for ros global _g_blend_state
    
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0
    
        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")
    
        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")
    
        # _ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # _angle
        temp = get_param(angle, an)
        if temp == None:
            _angle = [0, 0]
        else:
            if type(temp) == int or type(temp) == float:
                _angle = [temp, 0]
            elif type(temp) == list and len(temp) == DR_ANGLE_DT_LEN:
                _angle = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : angle, an")
    
        if is_number(_angle) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")
    
        for item in _angle:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")
    
        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")
    
        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")
    
        # qcommand
        if type(_pos1) == posx:
            qcommand = 0
        else:
            qcommand = 1
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__:
            # make multi pos
            _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
            #print(_circle_pos)
            srv = self._ros_movec(_circle_pos, _vel, _acc, _time, _radius, _ref, mod, _angle[0], _angle[1], ra, _async) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:   
            ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                             .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
        return ret
    
    def movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._movesj(pos_list, vel, acc, time, mod, v, a, t, _async=0)
        return ret
    def amovesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._movesj(pos_list, vel, acc, time, mod, v, a, t, _async=1)
        return ret
    def _movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):
        # pos_list
        if type(pos_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")
    
        for item in pos_list:
            if type(item) != posj:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__:
            # make multi pos
            _spline_posj = _ros_listToFloat64MultiArray(pos_list)
            srv = self._ros_movesj(_spline_posj, len(_spline_posj), _vel, _acc, _time, mod, _async)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:    
            ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
            print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                             .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
        return ret
    
    def movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = self._movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=0)
        return ret
    def amovesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = self._movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=1)
        return ret
    def _movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, _async=0):
        # pos_list
        if type(pos_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")
    
        for item in pos_list:
            if type(item) != posx:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)
    
        # _ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # vel_opt
        if type(vel_opt) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel_opt")
    
        if vel_opt != DR_MVS_VEL_NONE and vel_opt != DR_MVS_VEL_CONST:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel_opt")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__:
            _spline_posx = _ros_listToFloat64MultiArray(pos_list)
            srv = self._ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
            print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                            .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
        return ret
    
    def moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=0)
        return ret
    def amoveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=1)
        return ret
    def _moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):
    
        # seg_list
        if type(seg_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list")
    
        if len(seg_list) == 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : seg_list")
    
        _seg_list = []
        if __ROS__:
            for seg in seg_list:
                _seg_list.append(seg.to_list())
        else:
            for seg in seg_list:
                if type(seg) != posb:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list(item)")
                else:
                    _seg_list.append(seg.to_list())
    
        if __ROS__:
            print(len(_seg_list))
            _ros_seg_list = []
            _tmp_list = []
    
            for s in range(0, len(_seg_list)):
                #print(s)
                for elemnt in range(0, len(_seg_list[s])):
                    if _seg_list[s][elemnt] == None:	                
                        _seg_list[s][elemnt] = [0.0]*POINT_COUNT
    
                # make [pos1] + [pos2] + [type] + [radius]    
                _tmp_list = _seg_list[s][1] + _seg_list[s][2] + [_seg_list[s][0]] + [_seg_list[s][3]]
                _ros_seg_list.append(_tmp_list)            
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)
    
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS Service call
        if __ROS__:
            seg = _ros_listToFloat64MultiArray(_ros_seg_list)
            srv = self._ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, _ref, mod, _async)    
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
            print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                             .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
        return ret
    
    def move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = self._move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=0)
        return ret
    def amove_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = self._move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=1)
        return ret
    def _move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, _async=0):
        # rev
        if type(rev) != int and type(rev) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rev")
    
        if rev <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : rev (Ranges: rev > 0)")
    
        # rmax
        if type(rmax) != int and type(rmax) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rmax")
    
        if rmax <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: rmax (Ranges: rmax > 0)")
    
        # lmax
        if type(lmax) != int and type(lmax) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : lmax")
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELX_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCX_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)
    
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    
        if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis")
    
        # ref
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if ref != DR_BASE and ref != DR_TOOL and (ref < DR_TC_USER_MIN or ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__:
            srv = self._ros_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:    
            ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
            print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                             .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
        return ret
    
    def move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = self._move_periodic(amp, period, atime, repeat, ref, _async=0)
        return ret
    def amove_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = self._move_periodic(amp, period, atime, repeat, ref, _async=1)
        return ret
    def _move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL, _async=0):
        _amp = [POINT_COUNT]
        _period = [POINT_COUNT]
        _atime =0.0
        _repeat =0.0
        _ref=0 
    
        # amp : float[6] 
        if type(amp) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : amp")
        if len(amp) != POINT_COUNT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : amp")
        _amp =amp
    
        # period : float or float[6] 
        if (type(period) != int) and (type(period) != float) and (type(period) != list):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : period")
    
        if (type(period) == int) or (type(period) == float):
            _period = [period] * POINT_COUNT
        else: #list �� ��� 
            if len(period) != POINT_COUNT:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : period")
            _period = period
    
        # atime 
        if atime == None:
            _atime = 0.0
        else:
            if (type(atime) != int) and (type(atime) != float):
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : atime")
            if atime < 0.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : atime")
            _atime = atime
    
        # repeat 
        if repeat == None:
            _repeat = 1
        else:
            if (type(repeat) != int) and (type(repeat) != float): 
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : repeat")
            if repeat < 0.0: 
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : repeat")
            _repeat = repeat
    
        # ref
        if ref == None:
            #_ref = self._g_coord
            _ref = DR_TOOL
        else:
            if type(ref) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
            if ref < DR_BASE or ref > DR_TC_USER_MAX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
            _ref = ref
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS__:
            srv = self._ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
            print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                             .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
        return ret
    
    def  move_home(self, target=None):
    
        # target
        if target == None:
            _target = DR_HOME_TARGET_MECHANIC
        else:
            _target = target
    
        if __ROS__:
            srv = self._ros_move_home(_target)    
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_move_home(_target)
    
        return ret
        
    def mwait(self, time=0):
        ret = self._move_wait(time)
        return ret
    def _move_wait(self, time):
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")
    
        if time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_move_wait()  #ROS 에서는 time 인자를 사용하지 않음. 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:    
            ret = PythonMgr.py_mwait(time)
            print_ext_result("{0} = PythonMgr.py_mwait(time:{1})".format(ret, dr_form(time)))
        return ret
    
    def check_motion(self):
    
        if __ROS__:
            srv = self._ros_check_motion()  
            ret = srv.status
        else:    
            # C function call
            ret = PythonMgr.py_check_motion()
    
        return ret
    
    def change_operation_speed(self, speed):
        if type(speed) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
    
        if speed < DR_OP_SPEED_MIN or speed > DR_OP_SPEED_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : speed({0})".format(speed))
    
        if __ROS__:
            srv = self._ros_change_operation_speed(speed)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else: 
            # C function call
            ret = PythonMgr.py_change_operation_speed(speed)
    
        return ret
    
    def enable_alter_motion(self, n, mode, ref=None, limit_dPOS=None, limit_dPOS_per=None):
        # n
        if type(n) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : n")
        if n < 0:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : (n>=0)")
        _n = n #*20
    
        # mode
        if mode < DR_DPOS or mode > DR_DVEL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))
        _mode = mode
    
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        if(None==limit_dPOS):
            _limit_dPOS = [DR_COND_NONE, DR_COND_NONE]
        else:
            _limit_dPOS = limit_dPOS
    
        if(None==limit_dPOS_per):
            _limit_dPOS_per = [DR_COND_NONE, DR_COND_NONE]
        else:
            _limit_dPOS_per = limit_dPOS_per
    
        if __ROS__:
            srv = self._ros_enable_alter_motion(_n, _mode, _ref, _limit_dPOS, _limit_dPOS_per)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:    
            # C function call
            ret = PythonMgr.py_enable_alter_motion(_n, _mode, _ref, _limit_dPOS, _limit_dPOS_per)
    
        return ret
    
    def alter_motion(self, dposx):
    
        # _dposx
        if (type(dposx) == posx):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : dposx")
    
        _dposx = get_normal_pos(dposx, def_type=posx)
    
        if __ROS__:
            srv = self._ros_alter_motion(_dposx)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_alter_motion(_dposx)
    
        return ret
    
    def disable_alter_motion(self):
    
        if __ROS__:
            srv = self._ros_disable_alter_motion()  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:        
            # C function call
            ret = PythonMgr.py_disable_alter_motion()
    
        return ret
    def jog(self, jog_axis, ref=0, speed=0):
        if type(jog_axis) != int and type(jog_axis) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis")
        
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if type(speed) != int and type(speed) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_jog(jog_axis, ref, speed)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def jog_multi(self, jog_axis_list, ref=0, speed=0):
        if type(jog_axis_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis_list")
        
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if type(speed) != int and type(speed) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_jog_multi(jog_axis_list, ref, speed)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    ##### force/stiffness Control #########################################################################################################################
    def get_workpiece_weight(self):
    
        if __ROS__:
            srv = self._ros_get_workpiece_weight()  
            ret = srv.weight
        else:
            # C function call
            ret = PythonMgr.py_get_workpiece_weight()
    
        return ret
    
    def reset_workpiece_weight(self):
    
        if __ROS__:
            srv = self._ros_reset_workpiece_weight()  
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_reset_workpiece_weight()
    
        return ret
    
    def set_singular_handling(self, mode = DR_AVOID):
        if type(mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mode")
    
        if mode < DR_AVOID or mode > DR_VAR_VEL:    #DR_TASK_STOP
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))
    
        if __ROS__:
            srv = self._ros_set_singularity_handling(mode)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_set_singular_handling(mode)
    
        return ret
    def set_singularity_handling(self, mode = DR_AVOID):
        ret = set_singular_handling(mode)
        return ret
    
    def parallel_axis(self, *args, **kargs):
        len_args = len(args)
        len_kargs = len(kargs)
        _nType = 0
    
        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "axis" and key != "ref":
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, vect, axis, ref")
    
        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _vect = get_kargs(kargs, "vect")
        _axis = get_kargs(kargs, "axis")
        _ref = get_kargs(kargs, "ref")
    
        # check parameter combination
        if len_args == 0:
            if len_kargs == 2:
                if _vect == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 3:
                _nType = 3
                if _vect == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 4:
                if _x1 == None or _x2 == None or _x3 == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 5:
                _nType = 2
                if _x1 == None or _x2 == None or _x3 == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 1:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
            _vect = args[0]
    
            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 2:
            if len_kargs == 1:  # +ref
                _nType = 3
                _vect = args[0]
                _axis = args[1]
            else:
                if len_kargs != 0:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            _vect = args[0]
            _axis = args[1]
    
        elif len_args == 3:
            if len_kargs == 0:  # +ref
                _nType = 3
                _vect = args[0]
                _axis = args[1]
                _ref  = args[2]
            else:
                if len_kargs != 1:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
    
                if _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 4:
            if len_kargs == 1:  # + ref
                _nType = 2
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]
            else:
                if len_kargs != 0:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]
    
        elif len_args == 5:
            if len_kargs == 0:  # + ref
                _nType = 2
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]
                _ref = args[4]
            else:
                if len_kargs != 0:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        # check parameter type, value
        if _x1 != None:
            _x1 = get_posx(_x1)
            _x2 = get_posx(_x2)
            _x3 = get_posx(_x3)
        else:
            if type(_vect) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
            if len(_vect) != DR_VECTOR_DT_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))
    
        if type(_axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    
        if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))
    
        # C function call
        if _nType == 2: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS__:
                srv = self._ros_parallel_axis1(_x1, _x2, _x3, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_parallel_axis(_x1, _x2, _x3, _axis, _ref)
            #print_ext_result("{0} = PythonMgr.py_parallel_axis_ref(x1:{1}, x2:{2}, x3:{3}, axis:{4}, ref:{5})".format(ret, _x1, _x2, _x3, _axis, _ref))
        elif _nType == 3: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS__:
                srv = self._ros_parallel_axis2(_vect, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_parallel_axis_ex(_vect, _axis, _ref)
            #print_ext_result("{0} = PythonMgr.py_parallel_axis_ex_ref(vector:{1}, axis:{2}, ref:{3})".format(ret, dr_form(_vect), _axis, _ref))
        else:   # 기존
            _ref = DR_BASE
            if _x1 != None:
                if __ROS__:
                    srv = self._ros_parallel_axis1(_x1, _x2, _x3, _axis, _ref)  
                    ret = 0 if (srv.success == True) else -1
                else:
                    ret = PythonMgr.py_parallel_axis(_x1, _x2, _x3, _axis, _ref)
                #print_ext_result("{0} = PythonMgr.py_parallel_axis(x1:{1}, x2:{2}, x3:{3}, axis:{4}), ref:{5})".format(ret, _x1, _x2, _x3, _axis, _ref))
            else:
                if __ROS__:
                    srv = self._ros_parallel_axis2(_vect, _axis, _ref)  
                    ret = 0 if (srv.success == True) else -1
                else:
                    ret = PythonMgr.py_parallel_axis_ex(_vect, _axis, _ref)
                #print_ext_result("{0} = PythonMgr.py_parallel_axis_ex(vector:{1}, axis:{2}, ref:{3})".format(ret, dr_form(_vect), _axis, _ref))
        return ret
    
    def align_axis(self, *args, **kargs):
        len_args = len(args)
        len_kargs = len(kargs)
        _nType = 0
    
        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "pos" and key != "axis" and key != "ref":
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _vect = get_kargs(kargs, "vect")
        _pos = get_kargs(kargs, "pos")
        _axis = get_kargs(kargs, "axis")
        _ref = get_kargs(kargs, "ref")
    
        # check parameter combination
        if len_args == 0:
            if len_kargs == 3:
                if _vect == None or _pos == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 5:
                if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 6:
                _nType = 2
                if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 4:
                _nType = 3
                if _vect == None or _pos == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 1:
            if len_kargs != 2:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
            _vect = args[0]
    
            if _pos == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 2:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
            _vect = args[0]
            _pos = args[1]
    
            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 3:
            if len_kargs == 0:
                _vect = args[0]
                _pos = args[1]
                _axis = args[2]
            elif len_kargs == 1:
                _nType = 3
                _vect = args[0]
                _pos = args[1]
                _axis = args[2]
            elif len_kargs == 2:
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
    
                if _pos == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                if _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 4:
            if len_kargs == 0:
                _nType = 3
                _vect = args[0]
                _pos = args[1]
                _axis = args[2]
                _ref = args[3]
            else:
                if len_kargs != 1:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos = args[3]
    
                if _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        elif len_args == 5:
            if len_kargs == 0:
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos = args[3]
                _axis = args[4]
            elif len_kargs == 1:    # + ref
                _nType = 2
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos = args[3]
                _axis = args[4]
    
        elif len_args == 6:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]
            _axis = args[4]
            _ref  = args[5]
    
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        # check parameter type, value
        if _x1 != None:
            _x1 = get_posx(_x1)
            _x2 = get_posx(_x2)
            _x3 = get_posx(_x3)
        else:
            if type(_vect) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
            if len(_vect) != DR_VECTOR_DT_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))
    
        norm_pos = get_posx(_pos)
    
        if type(_axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
        if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))
    
        # C function call
        if _nType == 2: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
            if __ROS__:
                srv = self._ros_align_axis1(_x1, _x2, _x3, norm_pos, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_align_axis(_x1, _x2, _x3, norm_pos, _axis, _ref)
    
            #print_ext_result("{0} = PythonMgr.py_align_axis(x1:{1}, x2:{2}, x3:{3}, pos:{4}, axis:{5}, ref{6})" \
            #                 .format(ret, _x1, _x2, _x3, norm_pos, _axis, _ref))
        elif _nType == 3: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
            if __ROS__:
                srv = self._ros_align_axis2(_vect, norm_pos, _axis, _ref)  
                ret = 0 if (srv.success == True) else -1
            else:
                ret = PythonMgr.py_align_axis_ex(_vect, norm_pos, _axis, _ref)
    
            #print_ext_result("{0} = PythonMgr.py_align_axis_ex(vect:{1}, pos{2}, axis:{3}, ref{4})" \
            #                 .format(ret, dr_form(_vect), norm_pos, _axis, _ref))
        else:   # 기존
            _ref = DR_BASE
            if _x1 != None:
                if __ROS__:
                    srv = self._ros_align_axis1(_x1, _x2, _x3, norm_pos, _axis, _ref)  
                    ret = 0 if (srv.success == True) else -1
                else:
                    ret = PythonMgr.py_align_axis(_x1, _x2, _x3, norm_pos, _axis, _ref)
                #print_ext_result("{0} = PythonMgr.py_align_axis(x1:{1}, x2:{2}, x3:{3}, pos:{4}, axis:{5}, ref{6})" \
                #                 .format(ret, _x1, _x2, _x3, norm_pos, _axis, _ref))
            else:
            
                if __ROS__:
                    srv = self._ros_align_axis2(_vect, norm_pos, _axis, _ref)  
                    ret = 0 if (srv.success == True) else -1
                else:
                    ret = PythonMgr.py_align_axis_ex(_vect, norm_pos, _axis, _ref)
                #print_ext_result("{0} = PythonMgr.py_align_axis_ex(vect:{1}, pos{2}, axis:{3}, ref{4})" \
                #                 .format(ret, dr_form(_vect), norm_pos, _axis, _ref))
        return ret
    
    def is_done_bolt_tightening(self, m=0, timeout=0, axis=None):
        # m
        if type(m) != int and type(m) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : m")
    
        # timeout
        if type(timeout) != int and type(timeout) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : timeout")
    
        if timeout < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : timeout({0})".format(timeout))
    
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    
        if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))
    
        if __ROS__:
            srv = self._ros_is_done_bolt_tightening(m, timeout, axis)  
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_is_done_bolt_tightening(m, timeout, axis)
            #print_ext_result("{0} = PythonMgr.py_is_done_bolt_tightening(m:{1}, timeout:{2}, axis:{3})" \
            #                 .format(ret, dr_form(m), dr_form(timeout), axis))
    
        return ret
    
    def release_compliance_ctrl(self):
    
        if __ROS__:
            srv = self._ros_release_compliance_ctrl()  
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_release_compliance_ctrl()
            #print_ext_result("{0} = PythonMgr.py_release_compliance_ctrl()".format(ret))
    
        return ret
    
    def task_compliance_ctrl(self, stx=[3000, 3000, 3000, 200, 200, 200], time=0):
        # st
        if type(stx) != list or len(stx) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")
    
        if is_number(stx) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))
    
        # _ref
        _ref = self._g_coord
    
        # time
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")
    
        if time != -1 and time < 0:     # -1 : motion default
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))
    
        if time > 1:
            _time = 1
        else:
            _time = time
    
        if __ROS__:
            srv = self._ros_task_compliance_ctrl(stx, _ref, _time)  
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_task_compliance_ctrl(stx, _ref, _time)
            #print_ext_result("{0} = PythonMgr.py_task_compliance_ctrl(stx:{1}, ref:{2}, time:{3})" \
            #                 .format(ret, dr_form(stx), _ref, dr_form(_time)))
    
        return ret
    
    def set_stiffnessx(self, stx=[500, 500, 500, 100, 100, 100], time=0):
        # st
        if type(stx) != list or len(stx) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")
    
        if is_number(stx) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))
    
        # _ref
        _ref = self._g_coord
    
        # time
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")
    
        if time != -1 and time < 0:     # -1 : motion default
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))
    
        if time > 1:
            _time = 1
        else:
            _time = time
    
        if __ROS__:
            srv = self._ros_set_stiffnessx(stx, _ref, _time)  
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_set_stiffnessx(stx, _ref, _time)
            #print_ext_result("{0} = PythonMgr.py_set_stiffnessx(stx:{1}, ref:{2}, time:{3})".format(ret, dr_form(stx), _ref, dr_form(_time)))
    
        return ret
    
    def calc_coord(self, *args, **kargs):
    
        #arg(x1, ref, mod)                    arg =0 , kargs =3    _nType=1
        #arg(x1, x2, ref, mod)                arg =0 , kargs =4    _nType=2
        #arg(x1, x2, x3, ref, mod)            arg =0 , kargs =5    _nType=3
        #arg(x1, x2, x3, x4, ref, mod)        arg =0 , kargs =6    _nType=4
    
        #arg(?, ref, mod)                     arg =1 , kargs =2    _nType=1
        #arg(?, ?, ref, mod)                  arg =2 , kargs =2    _nType=2
        #arg(?, ?, ?, ref, mod)               arg =3 , kargs =2    _nType=3
        #arg(?, ?, ?, ?, ref, mod)            arg =4 , kargs =2    _nType=4
    
        #arg(?, ?, ?)                         arg =3 , kargs =0    _nType=1
        #arg(?, ?, ?, ?)                      arg =4 , kargs =0    _nType=2
        #arg(?, ?, ?, ?, ?)                   arg =5 , kargs =0    _nType=3
        #arg(?, ?, ?, ?, ?, ?)                arg =6 , kargs =0    _nType=4
    
        len_args = len(args)
        len_kargs = len(kargs)
        _cnt_pos = 0
    
        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "x4" and key != "ref" and key != "mod":
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, x4, ref, mod")
    
        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _x4 = get_kargs(kargs, "x4")
        _ref = get_kargs(kargs, "ref")
        _mod = get_kargs(kargs, "mod")
    
        if(_x1 == None):
            _x1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if(_x2 == None):
            _x2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if(_x3 == None):
            _x3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if(_x4 == None):
            _x4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
        if len_args == 0 and len_kargs ==3:
            _cnt_pos = 1
            print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
        elif len_args == 0 and len_kargs ==4:
            _cnt_pos = 2
            print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
        elif len_args == 0 and len_kargs ==5:
            _cnt_pos = 3
            print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
        elif len_args == 0 and len_kargs ==6:
            _cnt_pos = 4
            print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))
    
        elif len_args == 1 and len_kargs ==2:
            _cnt_pos = 1
            _x1 = args[0]
            print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
        elif len_args == 2 and len_kargs ==2:
            _cnt_pos = 2
            _x1 = args[0]
            _x2 = args[1]
            print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
        elif len_args == 3 and len_kargs ==2:
            _cnt_pos = 3
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
        elif len_args == 4 and len_kargs ==2:
            _cnt_pos = 4
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _x4 = args[3]
            print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))
    
        elif len_args == 3 and len_kargs ==0:
            _cnt_pos = 1
            _x1 = args[0]
            _ref =args[1]
            _mod =args[2]
            print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
        elif len_args == 4 and len_kargs ==0:
            _cnt_pos = 2
            _x1 = args[0]
            _x2 = args[1]
            _ref =args[2]
            _mod =args[3]
            print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
        elif len_args == 5 and len_kargs ==0:
            _cnt_pos = 3
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _ref =args[3]
            _mod =args[4]
            print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
        elif len_args == 6 and len_kargs ==0:
            _cnt_pos = 4
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _x4 = args[3]
            _ref =args[4]
            _mod =args[5]
            print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")
    
        print("FINAL : x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))
    
        # check parameter type, value
        if _x1 != None: 
            _x1 = get_posx(_x1)
        if _x2 != None: 
            _x2 = get_posx(_x2)
        if _x3 != None: 
            _x3 = get_posx(_x3)
        if _x4 != None: 
            _x4 = get_posx(_x4)
        if _ref !=None:
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if _mod != None:
            if _mod != 0 and _mod != 1:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(_mod))
    
        if __ROS__:
            srv = self._ros_calc_coord(_cnt_pos, _x1, _x2, _x3, _x4, _ref, _mod)  
            pos = list(srv.conv_posx)  # Convert tuple to list 
        else:
            # C function call
            ret = PythonMgr.py_calc_coord(_cnt_pos, _x1, _x2, _x3, _x4, _ref, _mod)
            if PY_EXT_RET_UNSUPPORT_CMD == ret:
                raise DR_Error(DR_ERROR_TYPE, "unsupported command")
            # check return
            _check_ext_result(ret)
    
            # wait for job to be finished
            proc_id = ret
            _wait_result(proc_id)
    
            # get result
            pos = PythonMgr.py_get_result(proc_id)
        
        conv_posx = posx(pos)
    
        return conv_posx
    
    def set_user_cart_coord(self, *args, **kargs):
        len_args = len(args)
        len_kargs = len(kargs)
        _nType =0  
    
        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "pos" and key != "u1" and key != "v1" and key != "v1" and key != "ref":
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, pos, u1, v1, ref")
    
        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _pos= get_kargs(kargs, "pos")
        _u1 = get_kargs(kargs, "u1")
        _v1 = get_kargs(kargs, "v1")
        _ref = get_kargs(kargs, "ref")
    
        # check parameter combination
        #set_user_cart_coord(x1=?, x2=?, x3=?, pos=?)   arg =0 , kargs =4    _nType=0
        #set_user_cart_coord(u1=?, v1=?, pos=?)         arg =0 , kargs =3 *  _nType=1
        #set_user_cart_coord(x1=?, x2=?, x3=?)          arg =0 , kargs =3 *  _nType=2
    
        #set_user_cart_coord(x1=?, x2=?, x3=?, ?)       arg =1 , kargs =3    _nType=0
        #set_user_cart_coord(u1=?, v1=?, ?)             arg =1 , kargs =2    _nType=1
    
        #set_user_cart_coord(?, ?, pos=?)               arg =2 , kargs =1    _nType=1
    
        #set_user_cart_coord(?, ?, ?, pos=?)            arg =3 , kargs =1    _nType=0
        #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=1
        #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=2  
    
        #set_user_cart_coord(?, ?, ?, ?)                arg =4 , kargs =0    _nType=0
    
    #----- 신규 추가 명령 2019/11/27 ----------------------------------------------------------------------------------------------------------------
        #set_user_cart_coord(pos=?, ref=?)         arg =0 , kargs =2    _nType=5
        #set_user_cart_coord(pos=?, ?)             arg =1 , kargs =1 *  _nType=5 #python syntax error : positional argument follows keyword argument
        #set_user_cart_coord(?, ref=?)             arg =1 , kargs =1 *  _nType=5
        #set_user_cart_coord(?, ?)                 arg =2 , kargs =0    _nType=5
    #------------------------------------------------------------------------------------------------------------------------------------------------
    
        if len_args == 0 and len_kargs ==2:
            print("new commnad  len_args == 0 and len_kargs ==2")
            #_pos
            #_ref
            print("_pos={},_ref={}".format(_pos, _ref))
            _nType = 5
        elif len_args == 1 and len_kargs ==1:
            print("new commnad  len_args == 1 and len_kargs ==1")
            _pos = args[0]
            #_ref
            print("_pos={},_ref={}".format(_pos, _ref))
            _nType = 5
        elif len_args == 2 and len_kargs ==0:
            print("new commnad  len_args == 2 and len_kargs ==0")
            _pos = args[0]
            _ref = args[1]
            print("_pos={},_ref={}".format(_pos, _ref))
            _nType = 5
        else:
            if len_args == 0:
                if len_kargs == 5:
                    if _x1 == None or _x2 == None or _x3 == None or _pos == None or _ref == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 3
                elif len_kargs == 4:
                    if _ref != None:
                        _nType = 4
                    else:
                        if _x1 == None or _x2 == None or _x3 == None or _pos == None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        _nType = 0
                elif len_kargs == 3:
                    if _u1 != None: 
                        if _v1 == None or _pos == None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        _nType = 1
                    elif _x1 != None:
                        if _x2 == None or _x3 == None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        _nType = 2
                    else: 
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 1:
                if len_kargs == 3:
                    if _x1 == None or _x2 == None or _x3 == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 0
                    _pos= args[0]
                elif len_kargs == 2:
                    if _u1 == None or _v1 == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 1
                    _pos= args[0]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 2:
                if len_kargs == 1:
                    if _pos != None: 
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    else:
                        _nType = 1
                        _u1 = args[0]
                        _v1 = args[1]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 3:
                if len_kargs == 1:
                    if _ref != None:
                        _nType = 4
                        _u1 = args[0]
                        _v1 = args[1]
                        _pos = args[2]
                    else:
                        if _pos != None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        else:
                            _nType = 0
                            _x1 = args[0]
                            _x2 = args[1]
                            _x3 = args[2]
                elif len_kargs == 0:
                    if len(args[0]) == 3:
                        _nType = 1
                        _u1 = args[0]
                        _v1 = args[1]
                        _pos= args[2]
                    elif len(args[0]) == 6:
                        _nType = 2
                        _x1 = args[0]
                        _x2 = args[1]
                        _x3 = args[2]
                    else:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 4:
                if len_kargs == 0:
                    _nType = 0
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                    _pos= args[3]
                elif len_kargs == 1:
                    _nType = 3
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                    _pos= args[3]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 5:
                if len_kargs == 0:
                    _nType = 0
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                    _pos= args[3]
                    _ref= args[4]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        # check parameter type, value
        if _x1 != None: 
            _x1 = get_posx(_x1)
        if _x2 != None: 
            _x2 = get_posx(_x2)
        if _x3 != None: 
            _x3 = get_posx(_x3)
        if _pos != None: 
            _pos = get_posx(_pos)
        if _u1 !=None:
            if type(_u1) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : u1")
            if len(_u1) != DR_VECTOR_U1_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : u1({0})".format(_u1))
        if _v1 !=None:
            if type(_v1) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : v1")
            if len(_v1) != DR_VECTOR_V1_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : v1({0})".format(_v1))
    
        # C function call
        if _nType == 0:
            _ref = DR_BASE
            if __ROS__:
                srv = self._ros_set_user_cart_coord2(_x1, _x2, _x3, _pos, _ref)  
                ret = srv.id
            else:
                ret = PythonMgr.py_set_user_cart_coord(_x1, _x2, _x3, _pos, _ref)
                #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord(x1:{1}, x2:{2}, x3:{3}, pos:{4}, ref:{5})".format(ret, _x1, _x2, _x3, _pos, _ref))
        elif _nType == 1:
            _ref = DR_BASE
            if __ROS__:
                srv = self._ros_set_user_cart_coord3(_u1, _v1, _pos, _ref)  
                ret = srv.id
            else:
                ret = PythonMgr.py_set_user_cart_coord_ex(_u1, _v1, _pos, _ref)
                #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex(u1:{1}, v1:{2}, pos:{3}, ref:{4})".format(ret, _u1, _v1, _pos, _ref))
        elif _nType == 2:   #현재 미사용
            #_ref = DR_BASE
            #if __ROS__:
            #    srv = self._ros_set_user_cart_coord?(_x1, _x2, _x3, _ref)  
            #    ret = srv.id
            #else:
            #    ret = PythonMgr.py_set_user_cart_coord_ex2(_x1, _x2, _x3, _ref)
            #    #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex2(x1:{1}, x2:{2}, x3:{3}, ref:{4})".format(ret, _x1, _x2, _x3, _ref))
            pass
        elif _nType == 3:   # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS__:
                srv = self._ros_set_user_cart_coord2(_x1, _x2, _x3, _pos, _ref)  
                ret = srv.id
            else:
                ret = PythonMgr.py_set_user_cart_coord(_x1, _x2, _x3, _pos, _ref)
                #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord(x1:{1}, x2:{2}, x3:{3}, pos:{4}, ref:{5})".format(ret, _x1, _x2, _x3, _pos, _ref))
        elif _nType == 4:   # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS__:
                srv = self._ros_set_user_cart_coord3(_u1, _v1, _pos, _ref)  
                ret = srv.id
            else:
                ret = PythonMgr.py_set_user_cart_coord_ex(_u1, _v1, _pos, _ref)
                #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex(u1:{1}, v1:{2}, pos:{3}, ref:{4})".format(ret, _u1, _v1, _pos, _ref))
        elif _nType == 5:   # (pos, ref) 20191127 신규 추가 
            #print("new command !!!!!!!!!!!!!!!!")
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS__:
                srv = self._ros_set_user_cart_coord1(_pos, _ref)  
                ret = srv.id
            else:
                ret = PythonMgr.py_set_user_cart_coord_ex3(_pos, _ref)
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
    
        return ret
    
    def overwrite_user_cart_coord(self, id, pos, ref=None, gol=None):
    
        # id
        _id = id
    
        # pos
        _pos = get_posx(pos)
    
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        # gol : #1 RCT 요청에 의하여 gol 무조건 0으로 변경 2019/12/09
        #if gol == None:
        #    _gol = 0
        #else:
        #    _gol = gol
        #    if _gol != 0 and _gol != 1:
        #        raise DR_Error(DR_ERROR_VALUE, "Invalid value : gol")
        _gol = 0
    
        # C function call
        if __ROS__:
            #srv = self._ros_overwrite_user_cart_coord(_id, _pos, _ref, _gol)  
            srv = self._ros_overwrite_user_cart_coord(_id, _pos, _ref)
            ret = srv.id
        else:
            ret = PythonMgr.py_overwrite_user_cart_coord(_id, _pos, _ref, _gol)
            if PY_EXT_RET_UNSUPPORT_CMD == ret:
                raise DR_Error(DR_ERROR_TYPE, "unsupported command")
    
        return ret
    
    def get_user_cart_coord(self, id):
    
        print("==============================> get_user_cart_coord(id)")
        if __ROS__:
            print("==============================> get_user_cart_coord(id) 111")
            srv = self._ros_get_user_cart_coord(id)  
    
            print("==============================> get_user_cart_coord(id) 222")
            pos = list(srv.conv_posx)  # Convert tuple to list 
    
            print("==============================> get_user_cart_coord(id) 333")
            ref = srv.ref
            print("==============================> get_user_cart_coord(id) 444")
        else:
            # C function call
            ret = PythonMgr.py_get_user_cart_coord(id)
            if PY_EXT_RET_UNSUPPORT_CMD == ret:
                raise DR_Error(DR_ERROR_TYPE, "unsupported command")
            # check return
            _check_ext_result(ret)
            # wait for job to be finished
            proc_id = ret
            _wait_result(proc_id)
            # get result
            pos, ref  = PythonMgr.py_get_result(proc_id)
        conv_posx = posx(pos)
    
        print("==============================> get_user_cart_coord(id) 555 conv_posx={}, ref={}".format(conv_posx, ref))
        return conv_posx, ref
    
    def set_desired_force(self, fd=[0, 0, 0, 0, 0, 0], dir=[0, 0, 0, 0, 0, 0], time=0, mod=DR_FC_MOD_ABS):
        # df
        if type(fd) != list or len(fd) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : fd")
    
        if is_number(fd) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : fd({0})".format(fd))
    
        # dir
        if type(dir) != list or len(dir) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : dir")
    
        if is_number(dir) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : dir({0})".format(dir))
    
        # _ref
        _ref = self._g_coord
    
        # time
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")
    
        if time != -1 and time < 0:     # -1 : motion default
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))
    
        if time > 1:
            _time = 1
        else:
            _time = time
    
        # _mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
        _mod = mod
    
        if __ROS__:
            srv = self._ros_set_desired_force(fd, dir, _ref, _time, _mod)  
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_set_desired_force(fd, dir, _ref, _time, _mod)
            #print_ext_result("{0} = PythonMgr.py_set_desired_force(fd:{1}, dir:{2}, ref:{3}, time:{4}, mod:{5})" \
            #                 .format(ret, dr_form(fd), dr_form(dir), _ref, dr_form(_time), _mod))
    
        return ret
    
    def release_force(self, time=0):
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")
    
        if time != -1 and time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))
    
        if time > 1:
            _time = 1
        else:
            _time = time
    
        if __ROS__:
            srv = self._ros_release_force(_time)  
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_release_force(_time)
            #print_ext_result("{0} = PythonMgr.py_release_force({1})".format(ret, dr_form(_time)))
    
        return ret
    
    #def check_position_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
    def check_position_condition(self, axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None, mod= DR_MV_MOD_ABS, pos=None):
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    
        if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))
    
        # min
        if type(min) != int and type(min) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
    
        # max
        if type(max) != int and type(max) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")
    
        if min == DR_COND_NONE and max == DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))
    
        # min < max check
        if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
            if min > max:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))
    
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(mod))
    
        # _pos : DR_MV_MOD_REL 인 경우에는 반드시 pos 가 필요하고, otherwise 불필요
        if(mod == DR_MV_MOD_REL):
            if(pos==None):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
            else:
                _pos = get_normal_pos(pos, def_type=posx)
        else:
            if(pos!=None):
                _pos = get_normal_pos(pos, def_type=posx)
            else:
                pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                _pos = get_normal_pos(pos, def_type=posx)
    
        # check axis
        if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
            _ref = DR_TOOL
    
        if __ROS__:
            srv = self._ros_check_position_condition(axis, min, max, _ref, mod, _pos)  
            ret = srv.success   #True or False 
        else:
            # C function call
            ret = PythonMgr.py_check_position_condition(axis, min, max, _ref, mod, _pos)
            #print_ext_result("{0} = PythonMgr.py_check_position_condition(axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6})" \
            #                 .format(ret, axis, dr_form(min), dr_form(max), _ref, mod, _pos))
    
        return ret
    
    def check_force_condition(self, axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    
        if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
            axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))
    
        # min
        if type(min) != int and type(min) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
    
        # max
        if type(max) != int and type(max) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")
    
        if min == DR_COND_NONE and max == DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))
    
        # min check : min 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
        if min != DR_COND_NONE:
            if min < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min (Ranges: min({0}) >= 0)".format(min))
    
        # max check : max 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
        if max != DR_COND_NONE:
            if max < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: max (Ranges: max({0}) >= 0)".format(max))
    
        # min < max check
        if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
            if min > max:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))
    
        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
        # check axis
        if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
            _ref = DR_TOOL
    
        if __ROS__:
            srv = self._ros_check_force_condition(axis, min, max, _ref)  
            ret = srv.success   #True or False 
        else:
            # C function call
            ret = PythonMgr.py_check_force_condition(axis, min, max, _ref)
            #print_ext_result("{0} = PythonMgr.py_check_force_condition(axis:{1}, min:{2}, max:{3}, ref:{4})" \
            #                 .format(ret, axis, dr_form(min), dr_form(max), _ref))
    
        return ret
    
    def check_orientation_condition(self, axis, min=None, max=None, ref=None, mod = None, pos=None):
        _cmd_type = 0
    
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    
        if (axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))
    
        # min, max check type
        if(min != None):
            if type(min) == posx:
                _cmd_type = 0
            elif type(min) == posj:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : min is not posj")
            elif type(min) == list and len(min) == POINT_COUNT:
                _cmd_type = 0
            elif type(min) == int or type(min) == float:
                _cmd_type = 1
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
        if(max != None):
            if type(max) == posx:
                _cmd_type = 0
            elif type(max) == posj:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : max is not posj")
            elif type(max) == list and len(max) == POINT_COUNT:
                _cmd_type = 0
            elif type(max) == int or type(max) == float:
                _cmd_type = 1
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")
    
        if _cmd_type == 0:
        
            # _pos_min
            if(min != None):
                _pos_min = get_normal_pos(min, def_type=posx)
            else:
                _pos_min = [DR_COND_NONE]*POINT_COUNT
    
            # _pos_max
            if(max != None):
                _pos_max = get_normal_pos(max, def_type=posx)
            else:
                _pos_max = [DR_COND_NONE]*POINT_COUNT
    
            # _ref
            if ref == None:
                _ref = self._g_coord
            else:
                _ref = ref
    
            # check ref
            if type(_ref) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
            if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
            # mod
            if mod == None:
                mod = DR_MV_MOD_ABS
            else:
                if type(mod) != int:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")
    
                if mod != DR_MV_MOD_ABS:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_ABS".format(mod))
    
            print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}".format(_cmd_type, axis, _pos_min, _pos_max, _ref, mod) )
    
        elif _cmd_type == 1:
            # min
            if min == None:
                min = DR_COND_NONE
            else:
                if type(min) != int and type(min) != float:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
            # max
            if max == None:
                max = DR_COND_NONE
            else:
                if type(max) != int and type(max) != float:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")
    
            if min == DR_COND_NONE and max == DR_COND_NONE:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))
    
            # min < max check
            if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
                if min > max:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))
    
            # _ref
            if ref == None:
                _ref = self._g_coord
            else:
                _ref = ref
        
            # check ref
            if type(_ref) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    
            if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    
            # mod
            if mod == None:
                mod = DR_MV_MOD_REL
            else:
                if type(mod) != int:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")
    
                if mod != DR_MV_MOD_REL:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_REL".format(mod))
    
            # _pos
            if(pos==None):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
            else:
                _pos = get_normal_pos(pos, def_type=posx)
    
            #print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6}".format(_cmd_type, axis, min, max, _ref, mod, _pos) )
    
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")
    
    
        # C function call
        if _cmd_type == 0:
            if __ROS__:
                srv = self._ros_check_orientation_condition1(axis, _pos_min, _pos_max, _ref, mod)  
                ret = srv.success   #True or False 
            else:
                ret = PythonMgr.py_check_orientation_condition(axis, _pos_min, _pos_max, _ref, mod)
                #print_ext_result("{0} = PythonMgr.py_check_orientation_condition(axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5})" \
                #                 .format(ret, axis, _pos_min, _pos_max, _ref, mod))
        else:
            if __ROS__:
                srv = self._ros_check_orientation_condition2(axis, min, max, _ref, mod, _pos)  
                ret = srv.success   #True or False 
            else:
                ret = PythonMgr.py_check_orientation_condition2(axis, min, max, _ref, mod, _pos)
                #print_ext_result("{0} = PythonMgr.py_check_orientation_condition2(axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6})" \
                #                .format(ret, axis, dr_form(min), dr_form(max), _ref, mod, _pos))
    
        return ret
    
    def coord_transform(self, pose_in, ref_in=DR_BASE, ref_out=None):
    
        # _pos
        _pos = get_normal_pos(pose_in, def_type=posx)
    
        # ref_in
        if type(ref_in) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_in")
    
        # ref_out
        if type(ref_out) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")
    
        if __ROS__:
            srv = self._ros_coord_transform(_pos, ref_in, ref_out)  
            pos = list(srv.conv_posx)  # Convert tuple to list 
        else:
            # C function call
            ret = PythonMgr.py_coord_transformed_posx(_pos, ref_in, ref_out)
            #print_ext_result("{0} = PythonMgr.py_coord_transformed_posx(pose_in:{1},ref_in:{2},ref_out:{3})".format(ret, _pos, ref_in, ref_out))
            # check return
            _check_ext_result(ret)
            # wait for job to be finished
            proc_id = ret
            _wait_result(proc_id)
            # get result
            pos = PythonMgr.py_get_result(proc_id)
    
        trans_posx = posx(pos)
    
        #print_result("{0} = coord_transform(pose_in:{1},ref_in:{2},ref_out:{3})".format(trans_posx, _pos, ref_in, ref_out))
        return trans_posx
    
    
    
    
    ##### I/O #########################################################################################################################
    
    def get_digital_input(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")
    
        if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_digital_input(index)
            value = srv.value
        else:
            value = PythonMgr.py_get_digital_input(index)
            print_ext_result("{0} = PythonMgr.py_get_digital_input(index:{1})".format(value, index))
        return value
    
    def get_analog_input(self, ch):
        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")
    
        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_analog_input(ch)
            value = srv.value
        else:
            value = PythonMgr.py_get_analog_input(ch)
            print_ext_result("{0} = PythonMgr.py_get_analog_input(index:{1})".format(value, ch))
        return value
    
    def get_tool_digital_input(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")
    
        if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_tool_digital_input(index)
            value = srv.value
        else:
            value = PythonMgr.py_get_tool_digital_input(index)
            print_ext_result("{0} = PythonMgr.py_get_tool_digital_input(index:{1})".format(value, index))
        return value
    
    def set_digital_output(self, index, val=None):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")
    
        if val != None:
            if type(val) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")
        
            if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
        
            if val != ON and val != OFF:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: # val 인자가 생략된 simple style
            if (index < (-DR_DIO_MAX_INDEX)) or (index > DR_DIO_MAX_INDEX) or (index==0): # -16~+16
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
            else:
                if index < 0:
                    index = index*(-1) 
                    val = OFF
                else:
                    index = index
                    val = ON
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_digital_output(index, val) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_digital_output(index, val)
            print_ext_result("{0} = PythonMgr.py_set_digital_output(index:{1}, val:{2})".format(ret, index, val))
        return ret

    def get_digital_output(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")
    
        if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_digital_output(index)
            value = srv.value
        else:
            value = PythonMgr.py_get_digital_output(index)
            print_ext_result("{0} = PythonMgr.py_get_digital_output(index:{1})".format(value, index))
        return value

    
    def set_analog_output(self, ch, val):
        #for ros global _g_analog_output_mode_ch1
        #for ros global _g_analog_output_mode_ch2
    
        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")
    
        if type(val) != int and type(val) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")
    
        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")
    
        #if val < 0 or val > 20.0:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        if ch == 1:
            if self._g_analog_output_mode_ch1 == DR_ANALOG_CURRENT:
                if val < 4 or val > 20.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            elif self._g_analog_output_mode_ch1 == DR_ANALOG_VOLTAGE:
                if val < 0 or val > 10.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            else: 
                raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch1) is not set")
        if ch == 2:
            if self._g_analog_output_mode_ch2 == DR_ANALOG_CURRENT:
                if val < 4 or val > 20.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            elif self._g_analog_output_mode_ch2 == DR_ANALOG_VOLTAGE:
                if val < 0 or val > 10.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            else: 
                raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch2) is not set")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_analog_output(ch, val) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_analog_output(ch, val)
            print_ext_result("{0} = PythonMgr.py_set_analog_output(ch:{1}, val:{2})".format(ret, ch, val))   
        return ret
    
    def set_mode_analog_output(self, ch, mod):
        #for ros global _g_analog_output_mode_ch1
        #for ros global _g_analog_output_mode_ch2
    
        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")
    
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")
    
        if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        if ch == 1: 
            self._g_analog_output_mode_ch1 = mod
        if ch == 2: 
            self._g_analog_output_mode_ch2 = mod
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_mode_analog_output(ch, mod)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_mode_analog_output(ch, mod)
            print_ext_result("{0} = PythonMgr.py_set_mode_analog_output(ch:{1}, mod:{2})".format(ret, ch, mod))
        return ret
    
    def set_mode_analog_input(self, ch, mod):
        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")
    
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")
    
        if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_mode_analog_input(ch, mod)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_mode_analog_input(ch, mod)
            print_ext_result("{0} = PythonMgr.py_set_mode_analog_input(ch:{1}, mod:{2})".format(ret, ch, mod))
        return ret
    
    def set_tool_digital_output(self, index, val=None):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")
    
        if val != None:
            if type(val) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")
    
            if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
    
            if val != ON and val != OFF:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: # val ���ڰ� ������ simple style
            if (index < (-DR_TDIO_MAX_INDEX)) or (index > DR_TDIO_MAX_INDEX) or (index==0): # -6~+6
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
            else:
                if index < 0:
                    index = index*(-1) 
                    val = OFF
                else:
                    index = index
                    val = ON
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_tool_digital_output(index, val)   
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_tool_digital_output(index, val)
            print_ext_result("{0} = PythonMgr.py_set_tool_digital_output(index:{1}, val:{2})".format(ret, index, val))
        return ret
     
    def get_tool_digital_output(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")
    
        if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_tool_digital_output(index)
            value = srv.value
        else:
            value = PythonMgr.py_get_tool_digital_output(index)
            print_ext_result("{0} = PythonMgr.py_get_tool_digital_output(index:{1})".format(value, index))
        return value


    ##### Modbus #########################################################################################################################
    
    def add_modbus_signal(self, ip, port, name, reg_type, index, value=0, slaveid=255):
        # ip
        if type(ip) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ip")
    
        #try:
        #    ipaddress.ip_address(ip)
        #except:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : ip")
    
        # port
        if type(port) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : port")
    
        if port <= 0 or port > 65535:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : port (Ranges: 1 ~ 65535)")
    
        # name
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # reg_type
        if type(reg_type) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : reg_type")
    
        if reg_type != DR_MODBUS_DIG_INPUT and reg_type != DR_MODBUS_DIG_OUTPUT and reg_type != DR_MODBUS_REG_INPUT and reg_type != DR_MODBUS_REG_OUTPUT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : reg_type (Ranges : 0 ~ 3")
    
        # index
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")
    
        if index < 0 or index > 65535:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index (Ranges: 0 ~ 65535)")
    
        # value
        if type(value) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : value")
    
        if value < 0 or value > 65535:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : value (Ranges: 0 ~ 65535)")
    
        # check value
        if reg_type == DR_MODBUS_DIG_OUTPUT or reg_type == DR_MODBUS_REG_OUTPUT:
            _value = value
        else:
            _value = 0
    
        # slaveid
        if type(slaveid) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : slaveid")
        if slaveid < 0 or slaveid > 255:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")
        elif slaveid > 247 and slaveid < 255:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_add_modbus_signal(name, ip, port, reg_type, index, _value, slaveid)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_add_modbus_signal(ip, port, name, reg_type, index, _value, slaveid)
            print_ext_result("{0} = PythonMgr.py_add_modbus_signal(ip:{1}, port:{2}, name:{3}, type:{4}, index:{5}, value:{6}, slaveid:{7})" \
                             .format(ret, ip, port, name, reg_type, index, _value, slaveid))    
        return ret
    
    def del_modbus_signal(self, name):
        # name
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_del_modbus_signal(name) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_del_modbus_signal(name)
            print_ext_result("{0} = PythonMgr.py_del_modbus_signal(name:{1})".format(ret, name))
        return ret
    
    def set_modbus_output(self, iobus, val):
        if type(iobus) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")
    
        if type(val) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")
    
        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_modbus_output(iobus, val) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_modbus_output(iobus, val)
            print_ext_result("{0} = PythonMgr.py_set_modbus_output(iobus:{1}, val:{2})".format(ret, iobus, val))
        return ret
    
    def get_modbus_input(self, iobus):
        if type(iobus) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_modbus_input(iobus)
            value = srv.value
        else:    
            value = PythonMgr.py_get_modbus_input(iobus)
            print_ext_result("{0} = PythonMgr.py_get_modbus_input(iobus:{1})".format(value, iobus))
        return value
    
    #################################################################################################################################################
    def set_tcp(self, name):
    
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_current_tcp(name) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_tcp(name)
            print_ext_result("{0} = PythonMgr.py_set_tcp(name:{1})".format(ret, name))
        return ret
    
    def get_tcp(self):
        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_get_current_tcp() 
        return srv.info
    
    def set_tool(self, name):
    
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_set_current_tool(name)    
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            ret = PythonMgr.py_set_tool(name)
            print_ext_result("{0} = PythonMgr.py_set_tool(name:{1})".format(ret, name))
        return ret
    
    def get_tool(self):
        # ROS service call
        if __ROS__:
            srv = self._ros_get_current_tool()
        return srv.info
    
    def set_tool_shape(self, name):
    
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        if __ROS__:
            srv = self._ros_set_tool_shape(name)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        else:
            # C function call
            ret = PythonMgr.py_set_tool_shape(name)
    
        return ret
    
    
    def add_tcp(self, name, pos):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_config_create_tcp(name, pos)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def del_tcp(self, name):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_config_delete_tcp(name)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def add_tool(self, name, weight, cog, inertia):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_config_create_tool(name, weight, cog, inertia) 
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def del_tool(self, name):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_config_delete_tool(name)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    ###########################################################################################################
    def drl_script_run(self, robotSystem, code):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")
    
        # ROS service call
        if __ROS__:
            srv = self._ros_drl_start(robotSystem, code)
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def drl_script_stop(self, stop_mode):
    
        # ROS service call
        if __ROS__:
            srv = self._ros_drl_stop(stop_mode)  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def drl_script_pause(self):
    
        # ROS service call
        if __ROS__:
            srv = self._ros_drl_pause()  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def drl_script_resume(self):
    
        # ROS service call
        if __ROS__:
            srv = self._ros_drl_resume()  
            #ret = srv.success
            ret = 0 if (srv.success == True) else -1
        return ret
    
    def get_drl_state():
    # ROS service call
        if __ROS__:
            srv = self._ros_get_drl_state()  
            ret = srv.drl_state
        return ret
