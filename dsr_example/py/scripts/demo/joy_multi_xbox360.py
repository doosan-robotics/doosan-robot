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

from DR_tcp_client import *

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *
r = CDsrRobot(ROBOT_ID, ROBOT_MODEL)


m_stop_watch_time = 30 #sec 
m_joyAnalogFlag = False
m_xyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = 0
m_joyJogVel = 0.0

g_sock = client_socket_open("192.168.137.2", 10004)
print("stop_watch server connect O.K!")

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
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    


def thread_stop_watch():
    print("thread_stop_watch running...")
    while 1:     
        res, rx_data = client_socket_read(g_sock) #server로부터 수신 대기
        print("XXXXXXXXXXXXXXXXXXXXX")
        print("XXXXXXXXXXXXXXXXXXXXX")
        print("XXXXXXXXXXXXXXXXXXXXX")

        print("res={0}, rx_data ={1}".format(res, rx_data)) 
        rev_str = str(rx_data).encode("utf-8")
        if rev_str == "#TIMEOUT":
            print("Time of game is over!!!!")
        elif rev_str == "#STOP":
            print("The game is stopped!!!!")
	else: 
            print("unknown data!!!")

def joy_cb(msg):
    global m_joyAnalogFlag
    global m_xyCompareFlag
    global m_joyButtonFlag
    global m_joyJogFlag
    global m_joyJogVel

    m_joyJogVel  = 60
    targetPos = [0, 0, 90, 0, 90, 0]
    hommingPos = [0, 0, 0, 0, 0, 0]
    jog_target = [0, 0, 0, 0, 0, 0]

    for i in range(0,8):
        #print("msg.buttons[{}] = {}".format(i,msg.buttons[i]) )
        print("msg.axes[{}] = {}".format(i,msg.axes[i]) )
    print("\n")    

####
    # go home 
    if msg.buttons[7] == 1 and msg.buttons[6] == 1:
        r.movej(targetPos, 50, 50)
        #----- START stop_watch --------------------------------- 
        client_socket_write(g_sock, b'#START')    
        #--------------------------------------------------------
    elif msg.buttons[8] == 1:
        #----- STOP stop_watch ----------------------------------
        client_socket_write(g_sock, b'#STOP')    
        #--------------------------------------------------------
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
        print("1111111")        
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
            
        #r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, m_joyJogVel)
        r.jog_multi([1,1,0,0,0,0], MOVE_REFERENCE_BASE, m_joyJogVel)

    #elif m_joyAnalogFlag and m_joyJogFlag == -1 and not m_joyButtonFlag:
    elif m_joyAnalogFlag and not m_joyButtonFlag:
        print("22222222")        

        if msg.axes[4] > 0:
            #m_joyJogFlag = JOG_AXIS_TASK_Z
            jog_target[2] = 1
        if msg.axes[4] < 0:
            #m_joyJogFlag = JOG_AXIS_TASK_Z
            jog_target[2] = -1

        m_xyCompareFlag = 0
        if msg.axes[1] > 0 and m_xyCompareFlag == 0:
            #m_joyJogFlag = JOG_AXIS_TASK_X
            jog_target[0] = -1*msg.axes[1] #-1
        if msg.axes[1] < 0 and m_xyCompareFlag == 0:
            #m_joyJogFlag = JOG_AXIS_TASK_X
            jog_target[0] = -1*msg.axes[1] #1

        m_xyCompareFlag = 1
        if msg.axes[0] > 0 and m_xyCompareFlag == 1:
            #m_joyJogFlag = JOG_AXIS_TASK_Y
            jog_target[1] = -1*msg.axes[0] #-1
        if msg.axes[0] < 0 and m_xyCompareFlag == 1:
            #m_joyJogFlag = JOG_AXIS_TASK_Y
            jog_target[1] = -1*msg.axes[0] #1
        print(">>>>>>>>>>>>>> jog_target = {}".format(jog_target))
        #r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, m_joyJogVel)
        r.jog_multi(jog_target, MOVE_REFERENCE_BASE, m_joyJogVel)       
    else:
        print("33333333")        
        if not m_joyAnalogFlag and not m_joyButtonFlag:
            rospy.loginfo("jog stop")
            #r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, 0)
            r.jog_multi([0,0,0,0,0,0], MOVE_REFERENCE_BASE, 0)
            m_joyJogFlag = -1


if __name__ == "__main__":
    rospy.init_node('joy_xbox360_py')
    rospy.on_shutdown(shutdown)

    t1 = threading.Thread(target=thread_stop_watch)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)
    while not rospy.is_shutdown():
        pass

    client_socket_close(g_sock)

    print('good bye!')
