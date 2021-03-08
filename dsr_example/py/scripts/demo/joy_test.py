#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] joystick example
# @author   Jinhyuk Gong (jinhyuk.gong@doosan.com)   

import rospy
import os
import threading, time
import sys
from sensor_msgs.msg import Joy    # Joy package import 
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"         # Robot Model & Robot ID identify namespace 
ROBOT_MODEL  = "m1013"      
import DR_init
m_joyAnalogFlag = False
m_xyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = 0
m_joyJogVel = 0.0

jog_target = [0, 0, 0, 0, 0, 0]
JOG_VELOCITY = 100

FLAG_HOMMING = 0
FLAG_READY = 1
FLAG_MULTI_JOG_START = 2
FLAG_JOG_STOP = 3

FLAG_CONTROL = -1

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

jog_multi = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/jog_multi', JogMultiAxis, queue_size=1) # Publish multi-jog topic

def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

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

def thread_publish_jog(): # Publish multi-jog 
    while not rospy.is_shutdown():
        if FLAG_CONTROL == FLAG_MULTI_JOG_START:
           # if (jog_target) != pre_jog_target:
            jog_multi.publish(jog_target, MOVE_REFERENCE_BASE, JOG_VELOCITY)
        rospy.sleep(0.01)
        


def joy_cb(msg):  # Joystick Callback Func; This function can handle the key mapping
    global m_joyAnalogFlag
    global m_joyButtonFlag
    global m_joyJogFlag
    global JOG_VELOCITY
    global FLAG_CONTROL
    global jog_target
    global pre_jog_target 
    rospy.loginfo(m_joyAnalogFlag)
    if msg.axes[4] != 0 or msg.axes[0] != 0 or msg.axes[1] != 0: # Analog stick is pushed
        m_joyAnalogFlag = True
    else:
        m_joyAnalogFlag = False

    if msg.axes[6] != 0 or msg.axes[7] != 0: # Button is pushed
        m_joyButtonFlag = True
    else:
        m_joyButtonFlag = False
    #rospy.loginfo(str(sum(msg.buttons)))
    if sum(msg.buttons) > 0:
        if msg.buttons[4] == 1 and msg.buttons[5] == 1: # R1 and L1 buttons are pressed simultaneously
            FLAG_CONTROL = FLAG_READY
        elif msg.buttons[10] == 1: # PS button is pressed
            FLAG_CONTROL = FLAG_HOMMING

    if m_joyAnalogFlag: # Analog stick is touched      	
        jog_target[0] = round(msg.axes[1], 1) # : X                                         
        jog_target[1] = round(msg.axes[0], 1) # : Y                                     
        jog_target[2] = round(msg.axes[4], 1) # : Z   
        FLAG_CONTROL = FLAG_MULTI_JOG_START
    elif not m_joyAnalogFlag and FLAG_CONTROL == FLAG_MULTI_JOG_START:
        FLAG_CONTROL = FLAG_JOG_STOP

############# Main Function
if __name__ == "__main__":
    rospy.init_node('joy_test_py')
    rospy.on_shutdown(shutdown)

    t1 = threading.Thread(target=thread_publish_jog)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)
    while not rospy.is_shutdown():
        if FLAG_CONTROL == FLAG_HOMMING:
            movej([0,0,0,0,0,0], 60, 30)
            FLAG_CONTROL = -1
        elif FLAG_CONTROL == FLAG_READY:
            #rospy.loginfo("READY")
            movej([0,0,90,0,90,0], 60, 30)      
            FLAG_CONTROL = -1      
        elif FLAG_CONTROL == FLAG_MULTI_JOG_START:
            pass
        elif FLAG_CONTROL == FLAG_JOG_STOP:
            jog_multi.publish([0,0,0,0,0,0], MOVE_REFERENCE_BASE, 0)        
            FLAG_CONTROL = -1

    print 'good bye!'
