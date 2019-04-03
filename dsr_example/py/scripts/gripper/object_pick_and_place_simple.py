#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_py 

# for single robot 

ROBOT_SYSTEM_VIRTUAL = 1
ROBOT_SYSTEM_REAL = 0
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

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
        print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    
  
if __name__ == "__main__":
    rospy.init_node('dsr_simple_test_py')
    rospy.on_shutdown(shutdown)

    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    p0=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    p1=[0.0, 0.0, 90.0, 0.0, 90.0 , 0.0]                           
    p2=[180.0, 0.0, 90, 0.0, 90.0, 0.0] 
    
    x1=[0, 0, -200, 0, 0, 0]
    x2=[0, 0, 200, 0, 0, 0]
    velx=[50, 50]
    accx=[100, 100]

    
    while not rospy.is_shutdown():
        movej(p0, 60, 30)
        movej(p1, 60, 30)
        
        movel(x1, velx, accx, 2, 0.0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)
        gripper_move(0.4)
        rospy.sleep(1)
        movel(x2, velx, accx, 2, 0.0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)

        movej(p2, 60, 30, 3)
        movel(x1, velx, accx, 2, 0.0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)
        gripper_move(0.0)
        rospy.sleep(1)
        movel(x2, velx, accx, 2, 0.0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)

    print 'good bye!'
