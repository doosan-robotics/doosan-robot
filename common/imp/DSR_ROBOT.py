#-*- coding: utf-8 -*-

# ##
# @mainpage
# @file     DSR_ROBOT.py
# @brief    Doosan Robotics ROS service I/F module
# @author   kabdol2<kabkyoum.kim@doosan.com>   
# @version  0.20
# @Last update date     2018-12-17
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

#  GPIO Operations
_ros_set_digital_output         = rospy.ServiceProxy(_srv_name_prefix +"/io/set_digital_output", SetCtrlBoxDigitalOutput)
_ros_get_digital_input          = rospy.ServiceProxy(_srv_name_prefix +"/io/get_digital_input", GetCtrlBoxDigitalInput)
_ros_set_tool_digital_output    = rospy.ServiceProxy(_srv_name_prefix +"/io/set_tool_digital_output", SetToolDigitalOutput)
_ros_get_tool_digital_input     = rospy.ServiceProxy(_srv_name_prefix +"/io/get_tool_digital_input", GetToolDigitalInput)
_ros_set_analog_output          = rospy.ServiceProxy(_srv_name_prefix +"/io/set_analog_output", SetCtrlBoxAnalogOutput)
_ros_get_analog_input           = rospy.ServiceProxy(_srv_name_prefix +"/io/get_analog_input", GetCtrlBoxAnalogInput)
_ros_set_mode_analog_output     = rospy.ServiceProxy(_srv_name_prefix +"/io/set_analog_output_type", SetCtrlBoxAnalogOutputType)
_ros_set_mode_analog_input      = rospy.ServiceProxy(_srv_name_prefix +"/io/set_analog_input_type", SetCtrlBoxAnalogInputType)

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

# DRL Operations
_ros_drl_pause                  = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_pause", DrlPause)                   #new
_ros_drl_resume                 = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_resume", DrlResume)                 #new
_ros_drl_start                  = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_start", DrlStart)                   #new
_ros_drl_stop                   = rospy.ServiceProxy(_srv_name_prefix +"/drl/drl_stop", DrlStop)                     #new

########################################################################################################################################


# point count
POINT_COUNT = 6

# solution space
DR_SOL_MIN = 0
DR_SOL_MAX = 7

# posb seg_type
DR_LINE = 0
DR_CIRCLE = 1

# move reference
DR_BASE = 0
DR_TOOL = 1
DR_WORLD = 2
DR_TC_USER_MIN = 101
DR_TC_USER_MAX = 200

# move mod
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# move reaction
DR_MV_RA_NONE = 0
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE = 1

# move command type
DR_MV_COMMAND_NORM = 0

# movesx velocity
DR_MVS_VEL_NONE = 0
DR_MVS_VEL_CONST = 1

# motion state
DR_STATE_IDLE = 0
DR_STATE_INIT = 1
DR_STATE_BUSY = 2
DR_STATE_BLEND = 3
DR_STATE_ACC = 4
DR_STATE_CRZ = 5
DR_STATE_DEC = 6

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
DR_QSTOP = 1
DR_SSTOP = 2
DR_HOLD  = 3

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
DR_MODBUS_DIG_INPUT = 0
DR_MODBUS_DIG_OUTPUT = 1
DR_MODBUS_REG_INPUT = 2
DR_MODBUS_REG_OUTPUT = 3

DR_MODBUS_ACCESS_MAX = 32
DR_MAX_MODBUS_NAME_SIZE = 32

# tp_popup pm_type
DR_PM_MESSAGE = 0
DR_PM_WARNING = 1
DR_PM_ALARM = 2

# tp_get_user_input type
DR_VAR_INT = 0
DR_VAR_FLOAT = 1
DR_VAR_STR = 2

# len
DR_VELJ_DT_LEN = 6
DR_ACCJ_DT_LEN = 6

DR_VELX_DT_LEN = 2
DR_ACCX_DT_LEN = 2

DR_ANGLE_DT_LEN = 2
DR_COG_DT_LEN =3
DR_WEIGHT_DT_LEN = 3
DR_VECTOR_DT_LEN = 3
DR_ST_DT_LEN = 6
DR_FD_DT_LEN = 6
DR_DIR_DT_LEN = 6
DR_INERTIA_DT_LEN = 6
DR_VECTOR_U1_LEN = 3
DR_VECTOR_V1_LEN = 3

# set_singular_handling mode 
DR_AVOID     = 0
DR_TASK_STOP = 1

# =============================================================================================
# global variable

DR_CONFIG_PRT_EXT_RESULT = False
DR_CONFIG_PRT_RESULT = False

_g_blend_state = False
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

# make multi positions : list -> Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res

##### MOTION ##############################################################################################################################
def movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, async=0)
    return ret
def amovej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, async=1)
    return ret
def _movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, async=0):
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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__: 
        ret = _ros_movej(_pos, _vel[0], _acc[0], _time, mod, _radius, ra, _async)
    else:   
        ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
    return ret

def movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, async=0)
    return ret 
def amovejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, async=1)
    return ret 
def _movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, async=0):
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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__: 
        ret = _ros_movejx(_pos, sol, _vel[0], _acc[0], _time, mod, _ref, _radius, ra, _async)   
    else:    
        ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
        print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
    return ret

def movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, async=0)
    return ret
def amovel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, async=1)
    return ret
def _movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, async=0):
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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__: 
        ret = _ros_movel(_pos, _vel, _acc, _time, mod, _ref, ra, qcommand, _async) 
    else:    
        ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
    return ret

def movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, async=0)
    return ret
def amovec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, async=1)
    return ret
def _movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, async=0):

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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        # make multi pos
        _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
        #print(_circle_pos)
        ret = _ros_movec(_circle_pos, _vel, _acc, _time, mod, _ref, _angle[0], _angle[1], _radius, ra, _async) 
    else:   
        ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                         .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
    return ret

def movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, async=0)
    return ret
def amovesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, async=1)
    return ret
def _movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, async=0):
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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        # make multi pos
        _spline_posj = _ros_listToFloat64MultiArray(pos_list)
        ret = _ros_movesj(_spline_posj, len(_spline_posj), vel, acc, _time, mod, _async)

    else:    
        ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
        print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                         .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
    return ret

def movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, async=0)
    return ret
def amovesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, async=1)
    return ret
def _movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, async=0):
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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        _spline_posx = _ros_listToFloat64MultiArray(pos_list)
        ret = _ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
    else:
        ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
        print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                        .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
    return ret

def moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, async=0)
    return ret
def amoveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, async=1)
    return ret
def _moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, async=0):

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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS Service call
    if __ROS__:
        seg = _ros_listToFloat64MultiArray(_ros_seg_list)
        ret = _ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, mod, _ref, _async)    
    else:
        ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
        print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                         .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
    return ret

def move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, async=0)
    return ret
def amove_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, async=1)
    return ret
def _move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, async=0):
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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        ret = _ros_move_spiral(axis, rev, rmax, lmax, _vel, _acc, _time, ref, _async)
    else:    
        ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
        print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                         .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
    return ret

def move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, async=0)
    return ret
def amove_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, async=1)
    return ret
def _move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL, async=0):
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
    _async = async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS__:
        ret = _ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
    else:
        ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
        print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                         .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
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
        ret = _ros_move_wait()  #ROS 에서는 time 인자를 사용하지 않음. 
    else:    
        ret = PythonMgr.py_mwait(time)
        print_ext_result("{0} = PythonMgr.py_mwait(time:{1})".format(ret, dr_form(time)))
    return ret

##############################################################################################################################

def add_modbus_signal(ip, port, name, reg_type, index, value=0):
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

    # ROS service call
    if __ROS__:
        ret = _ros_add_modbus_signal(name, ip, port, reg_type, index, _value)
    else:
        ret = PythonMgr.py_add_modbus_signal(ip, port, name, reg_type, index, _value)
        print_ext_result("{0} = PythonMgr.py_add_modbus_signal(ip:{1}, port:{2}, name:{3}, type:{4}, index:{5}, value:{6})" \
                         .format(ret, ip, port, name, reg_type, index, _value))    
    return ret

def del_modbus_signal(name):
    # name
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_del_modbus_signal(name) 
    else:
        ret = PythonMgr.py_del_modbus_signal(name)
        print_ext_result("{0} = PythonMgr.py_del_modbus_signal(name:{1})".format(ret, name))
    return ret

def get_digital_input(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    # ROS service call
    if __ROS__:
        value = _ros_get_digital_input(index)
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
        value = _ros_get_analog_input(ch)
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
        value = _ros_get_tool_digital_input(index)
    else:
        value = PythonMgr.py_get_tool_digital_input(index)
        print_ext_result("{0} = PythonMgr.py_get_tool_digital_input(index:{1})".format(value, index))
    return value

def get_modbus_input(iobus):
    if type(iobus) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

    # ROS service call
    if __ROS__:
        value = _ros_get_modbus_input(iobus)
    else:    
        value = PythonMgr.py_get_modbus_input(iobus)
        print_ext_result("{0} = PythonMgr.py_get_modbus_input(iobus:{1})".format(value, iobus))
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
        ret = _ros_set_digital_output(index, val) 
    else:
        ret = PythonMgr.py_set_digital_output(index, val)
        print_ext_result("{0} = PythonMgr.py_set_digital_output(index:{1}, val:{2})".format(ret, index, val))
    return ret

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
        ret = _ros_set_analog_output(ch, val) 
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
        ret = _ros_set_mode_analog_output(ch, mod)
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
        ret = _ros_set_mode_analog_input(ch, mod)
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
        ret = _ros_set_tool_digital_output(iobus, val)   
    else:
        ret = PythonMgr.py_set_tool_digital_output(index, val)
        print_ext_result("{0} = PythonMgr.py_set_tool_digital_output(index:{1}, val:{2})".format(ret, index, val))
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
        ret = _ros_set_modbus_output(iobus, val) 
    else:
        ret = PythonMgr.py_set_modbus_output(iobus, val)
        print_ext_result("{0} = PythonMgr.py_set_modbus_output(iobus:{1}, val:{2})".format(ret, iobus, val))
    return ret


def set_tcp(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

    # ROS service call
    if __ROS__:
        ret = _ros_get_current_tcp(name) 
    else:
        ret = PythonMgr.py_set_tcp(name)
        print_ext_result("{0} = PythonMgr.py_set_tcp(name:{1})".format(ret, name))
    return ret

def set_tool(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

    # ROS service call
    if __ROS__:
        ret = _ros_get_current_tool(name)    
    else:
        ret = PythonMgr.py_set_tool(name)
        print_ext_result("{0} = PythonMgr.py_set_tool(name:{1})".format(ret, name))
    return ret


###########################################################################################################

def add_tcp(name, pos):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_config_create_tcp(name, pos)  
    return ret

def del_tcp(name):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_config_delete_tcp(name)  
    return ret

def add_tool(name, weight, cog, inertia):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_config_create_tool(name, weight, cog, inertia) 
    return ret

def del_tool(name):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_config_delete_tool(name)  
    return ret

def drl_script_run(robotSystem, code):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_drl_start(robotSystem, code)
    return ret

def drl_script_stop(stop_mode):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_drl_stop(stop_mode)  
    return ret

def drl_script_pause():
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_drl_pause()  
    return ret

def drl_script_resume():
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS__:
        ret = _ros_drl_resume()  
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

        #  GPIO Operations
        self._ros_set_digital_output         = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_digital_output", SetCtrlBoxDigitalOutput)
        self._ros_get_digital_input          = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_digital_input", GetCtrlBoxDigitalInput)
        self._ros_set_tool_digital_output    = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_tool_digital_output", SetToolDigitalOutput)
        self._ros_get_tool_digital_input     = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_tool_digital_input", GetToolDigitalInput)
        self._ros_set_analog_output          = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_analog_output", SetCtrlBoxAnalogOutput)
        self._ros_get_analog_input           = rospy.ServiceProxy(self._srv_name_prefix +"/io/get_analog_input", GetCtrlBoxAnalogInput)
        self._ros_set_mode_analog_output     = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_analog_output_type", SetCtrlBoxAnalogOutputType)
        self._ros_set_mode_analog_input      = rospy.ServiceProxy(self._srv_name_prefix +"/io/set_analog_input_type", SetCtrlBoxAnalogInputType)

        #  Modbus Operations
        self._ros_set_modbus_output          = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/set_modbus_output", SetModbusOutput)
        self._ros_get_modbus_input           = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/get_modbus_input", GetModbusInput)
        self._ros_add_modbus_signal          = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/config_create_modbus", ConfigCreateModbus)
        self._ros_del_modbus_signal          = rospy.ServiceProxy(self._srv_name_prefix +"/modbus/config_delete_modbus", ConfigDeleteModbus)

        # TCP Operations
        self._ros_set_current_tcp            = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/set_current_tcp", SetCurrentTcp)        #new
        self._ros_get_current_tcp            = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/get_current_tcp", GetCurrentTcp)        
        self._ros_config_create_tcp          = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/config_create_tcp", ConfigCreateTcp)    #new
        self._ros_config_delete_tcp          = rospy.ServiceProxy(self._srv_name_prefix +"/tcp/config_delete_tcp", ConfigDeleteTcp)    #new

        # Tool Operations
        self._ros_set_current_tool           = rospy.ServiceProxy(self._srv_name_prefix +"/tool/set_current_tool", SetCurrentTool)     #new
        self._ros_get_current_tool           = rospy.ServiceProxy(self._srv_name_prefix +"/tool/get_current_tool", GetCurrentTool)      
        self._ros_config_create_tool         = rospy.ServiceProxy(self._srv_name_prefix +"/tool/config_create_tool", ConfigCreateTool) #new
        self._ros_config_delete_tool         = rospy.ServiceProxy(self._srv_name_prefix +"/tool/config_delete_tool", ConfigDeleteTool) #new

        # DRL Operations
        self._ros_drl_pause                  = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_pause", DrlPause)                   #new
        self._ros_drl_resume                 = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_resume", DrlResume)                 #new
        self._ros_drl_start                  = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_start", DrlStart)                   #new
        self._ros_drl_stop                   = rospy.ServiceProxy(self._srv_name_prefix +"/drl/drl_stop", DrlStop)                     #new
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

    ##### MOTION ##############################################################################################################################
    def movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, async=0)
        return ret
    def amovej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, async=1)
        return ret
    def _movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, async=0):
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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__: 
            ret = self._ros_movej(_pos, _vel[0], _acc[0], _time, mod, _radius, ra, _async)
        else:   
            ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
        return ret

    def movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = self._movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, async=0)
        return ret 
    def amovejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = self._movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, async=1)
        return ret 
    def _movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, async=0):
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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__: 
            ret = self._ros_movejx(_pos, sol, _vel[0], _acc[0], _time, mod, _ref, _radius, ra, _async)   
        else:    
            ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
            print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
        return ret

    def movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, async=0)
        return ret
    def amovel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, async=1)
        return ret
    def _movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, async=0):
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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__: 
            ret = self._ros_movel(_pos, _vel, _acc, _time, mod, _ref, ra, qcommand, _async) 
        else:    
            ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
        return ret

    def movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = self._movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, async=0)
        return ret
    def amovec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = self._movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, async=1)
        return ret
    def _movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, async=0):

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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__:
            # make multi pos
            _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
            #print(_circle_pos)
            ret = self._ros_movec(_circle_pos, _vel, _acc, _time, mod, _ref, _angle[0], _angle[1], _radius, ra, _async) 
        else:   
            ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                             .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
        return ret

    def movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._movesj(pos_list, vel, acc, time, mod, v, a, t, async=0)
        return ret
    def amovesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._movesj(pos_list, vel, acc, time, mod, v, a, t, async=1)
        return ret
    def _movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, async=0):
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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__:
            # make multi pos
            _spline_posj = _ros_listToFloat64MultiArray(pos_list)
            ret = self._ros_movesj(_spline_posj, len(_spline_posj), vel, acc, _time, mod, _async)
        else:    
            ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
            print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                             .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
        return ret

    def movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = self._movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, async=0)
        return ret
    def amovesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = self._movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, async=1)
        return ret
    def _movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, async=0):
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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__:
            _spline_posx = _ros_listToFloat64MultiArray(pos_list)
            ret = self._ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
        else:
            ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
            print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                            .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
        return ret

    def moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._moveb(seg_list, vel, acc, ref, time, mod, v, a, t, async=0)
        return ret
    def amoveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = self._moveb(seg_list, vel, acc, ref, time, mod, v, a, t, async=1)
        return ret
    def _moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, async=0):

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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS Service call
        if __ROS__:
            seg = _ros_listToFloat64MultiArray(_ros_seg_list)
            ret = self._ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, mod, _ref, _async)    
        else:
            ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
            print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                             .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
        return ret

    def move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = self._move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, async=0)
        return ret
    def amove_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = self._move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, async=1)
        return ret
    def _move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, async=0):
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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__:
            ret = self._ros_move_spiral(axis, rev, rmax, lmax, _vel, _acc, _time, ref, _async)
        else:    
            ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
            print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                             .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
        return ret

    def move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = self._move_periodic(amp, period, atime, repeat, ref, async=0)
        return ret
    def amove_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = self._move_periodic(amp, period, atime, repeat, ref, async=1)
        return ret
    def _move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL, async=0):
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
        _async = async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS__:
            ret = self._ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
        else:
            ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
            print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                             .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
        return ret

    def mwait(self, time=0):
        ret = self._move_wait(time)
        return ret
    def move_wait(self, time=0):
        ret = self._move_wait(time)
        return ret
    def _move_wait(self, time):
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

        if time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time")

        # ROS service call
        if __ROS__:
            ret = self._ros_move_wait()  #ROS 에서는 time 인자를 사용하지 않음. 
        else:    
            ret = PythonMgr.py_mwait(time)
            print_ext_result("{0} = PythonMgr.py_mwait(time:{1})".format(ret, dr_form(time)))
        return ret

    ##############################################################################################################################

    def add_modbus_signal(self, ip, port, name, reg_type, index, value=0):
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

        # ROS service call
        if __ROS__:
            ret = self._ros_add_modbus_signal(name, ip, port, reg_type, index, _value)
        else:
            ret = PythonMgr.py_add_modbus_signal(ip, port, name, reg_type, index, _value)
            print_ext_result("{0} = PythonMgr.py_add_modbus_signal(ip:{1}, port:{2}, name:{3}, type:{4}, index:{5}, value:{6})" \
                             .format(ret, ip, port, name, reg_type, index, _value))    
        return ret

    def del_modbus_signal(self, name):
        # name
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_del_modbus_signal(name) 
        else:
            ret = PythonMgr.py_del_modbus_signal(name)
            print_ext_result("{0} = PythonMgr.py_del_modbus_signal(name:{1})".format(ret, name))
        return ret

    def get_digital_input(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

        # ROS service call
        if __ROS__:
            value = self._ros_get_digital_input(index)
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
            value = self._ros_get_analog_input(ch)
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
            value = self._ros_get_tool_digital_input(index)
        else:
            value = PythonMgr.py_get_tool_digital_input(index)
            print_ext_result("{0} = PythonMgr.py_get_tool_digital_input(index:{1})".format(value, index))
        return value

    def get_modbus_input(self, iobus):
        if type(iobus) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

        # ROS service call
        if __ROS__:
            value = self._ros_get_modbus_input(iobus)
        else:    
            value = PythonMgr.py_get_modbus_input(iobus)
            print_ext_result("{0} = PythonMgr.py_get_modbus_input(iobus:{1})".format(value, iobus))
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
            ret = self._ros_set_digital_output(index, val) 
        else:
            ret = PythonMgr.py_set_digital_output(index, val)
            print_ext_result("{0} = PythonMgr.py_set_digital_output(index:{1}, val:{2})".format(ret, index, val))
        return ret

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
            ret = self._ros_set_analog_output(ch, val) 
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
            ret = self._ros_set_mode_analog_output(ch, mod)
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
            ret = self._ros_set_mode_analog_input(ch, mod)
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
            ret = self._ros_set_tool_digital_output(iobus, val)   
        else:
            ret = PythonMgr.py_set_tool_digital_output(index, val)
            print_ext_result("{0} = PythonMgr.py_set_tool_digital_output(index:{1}, val:{2})".format(ret, index, val))
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
            ret = self._ros_set_modbus_output(iobus, val) 
        else:
            ret = PythonMgr.py_set_modbus_output(iobus, val)
            print_ext_result("{0} = PythonMgr.py_set_modbus_output(iobus:{1}, val:{2})".format(ret, iobus, val))
        return ret


    def set_tcp(self, name):

        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

        # ROS service call
        if __ROS__:
            ret = self._ros_get_current_tcp(name) 
        else:
            ret = PythonMgr.py_set_tcp(name)
            print_ext_result("{0} = PythonMgr.py_set_tcp(name:{1})".format(ret, name))
        return ret

    def set_tool(self, name):

        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

        # ROS service call
        if __ROS__:
            ret = self._ros_get_current_tool(name)    
        else:
            ret = PythonMgr.py_set_tool(name)
            print_ext_result("{0} = PythonMgr.py_set_tool(name:{1})".format(ret, name))
        return ret


    ###########################################################################################################

    def add_tcp(self, name, pos):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_config_create_tcp(name, pos)  
        return ret

    def del_tcp(self, name):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_config_delete_tcp(name)  
        return ret

    def add_tool(self, name, weight, cog, inertia):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_config_create_tool(name, weight, cog, inertia) 
        return ret

    def del_tool(self, name):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_config_delete_tool(name)  
        return ret

    def drl_script_run(self, robotSystem, code):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_drl_start(robotSystem, code)
        return ret

    def drl_script_stop(self, stop_mode):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_drl_stop(stop_mode)  
        return ret

    def drl_script_pause(self):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_drl_pause()  
        return ret

    def drl_script_resume(self):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS__:
            ret = self._ros_drl_resume()  
        return ret