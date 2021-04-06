#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] Jog simple test for doosan robot
# @author   Jin Hyuk Gong (jinhyuk.gong@doosan.com)   
# @format   rosrun dsr_example_py jog_simple.py <JOG_AXIS> <MOVE_REFERENCE> <SPEED>

import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

JOG_AXIS = 0
REF      = 0
SPEED      = 10

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")
    VEL = 0
    jog(JOG_AXIS, REF, SPEED)
    pub_stop.publish(stop_mode=1) # STOP_TYPE_QUICK
    return 0

if __name__ == "__main__":
    rospy.init_node('jog_simple_py')
    rospy.on_shutdown(shutdown)

    set_robot_mode(ROBOT_MODE_MANUAL)  ### jog is only available in manual mode.
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    
    if len(sys.argv) < 4:
        print("usage : jog_basic.py jog_axis reference, velocity")
        print("default setting")
        print("jog_axis : 0, move_reference : base, speed : 10%")
        JOG_AXIS = 0
        REF      = 0
        SPEED    = 10

    else:
        print("jog_simple.py %d %d %5.3f" % (int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])))
        JOG_AXIS = int(sys.argv[1])
        REF      = int(sys.argv[2])
        SPEED    = float(sys.argv[3])

    jog(JOG_AXIS, REF, SPEED)

    while not rospy.is_shutdown():
        pass

    print('good bye!')
