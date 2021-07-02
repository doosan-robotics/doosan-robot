#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
from sensor_msgs.msg import Joy
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 


# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

m_joyAnalogFlag = False
m_xyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = 0
m_joyJogVel = 0.0

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        #print("  current_posx      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))

        #print("  io_control_box    : %d" % (msg.io_control_box))
        ##print("  io_modbus         : %d" % (msg.io_modbus))
        ##print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    

r = CDsrRobot(ROBOT_ID, ROBOT_MODEL)

def joy_cb(msg):
    global m_joyAnalogFlag
    global m_xyCompareFlag
    global m_joyButtonFlag
    global m_joyJogFlag
    global m_joyJogVel

    targetPos = [0, 0, 90, 0, 90, 0]
    hommingPos = [0, 0, 0, 0, 0, 0]
##
    if msg.buttons[7] == 1 and msg.buttons[6] == 1:
        r.movej(targetPos, 50, 50)
    elif msg.buttons[8] == 1:
        r.movej(hommingPos, 50, 50)
    
    if msg.axes[4] != 0 or msg.axes[0] != 0 or msg.axes[1] != 0:
        m_joyAnalogFlag = True
    else:
        m_joyAnalogFlag = False

    if msg.axes[1] != 0 or msg.axes[0] or 0:
        if abs(msg.axes[1]) > abs(msg.axes[0]):
            m_xyCompareFlag = False
        else:
            m_xyCompareFlag = True
    
    if msg.axes[6] != 0 or msg.axes[7] != 0:
        m_joyButtonFlag = True
    else:
        m_joyButtonFlag = False

    if m_joyJogFlag == -1 and not m_joyAnalogFlag and m_joyButtonFlag:
        if msg.axes[6] == 1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel  = -60
        if msg.axes[6] == -1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel  = 60
            
        if msg.axes[7] == 1:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel  = 60
            
        if msg.axes[7] == -1:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel  = -60
            
        r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, m_joyJogVel)

    elif m_joyAnalogFlag and m_joyJogFlag == -1 and not m_joyButtonFlag:
        if msg.axes[4] > 0:
            m_joyJogFlag = JOG_AXIS_TASK_Z
            m_joyJogVel = -60
        if msg.axes[4] < 0:
            m_joyJogFlag = JOG_AXIS_TASK_Z
            m_joyJogVel = 60
        if msg.axes[1] > 0 and m_xyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel = 60
        if msg.axes[1] < 0 and m_xyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel = -60
        if msg.axes[0] > 0 and m_xyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = -60
        if msg.axes[0] < 0 and m_xyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = 60
        r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, m_joyJogVel)

    else:
        if not m_joyAnalogFlag and not m_joyButtonFlag:
            rospy.loginfo("jog stop")
            r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, 0)
            m_joyJogFlag = -1


if __name__ == "__main__":
    rospy.init_node('joy_xbox360_py')
    rospy.on_shutdown(shutdown)

    #t1 = threading.Thread(target=thread_subscriber)
    #t1.daemon = True 
    #t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)
    while not rospy.is_shutdown():
        pass

    print('good bye!')
