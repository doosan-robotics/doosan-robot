#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example basic] Jog basic test for doosan robot
# @author   Jin Hyuk Gong (jinhyuk.gong@doosan.com)
# @format   rosrun dsr_example_py jog_basic.py <JOG_AXIS> <MOVE_REFERENCE> <SPEED>   

import rospy
import os
import threading, time
from std_msgs.msg import String,Int32,Int32MultiArray,Float64,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import sys
sys.dont_write_bytecode = True

from dsr_msgs.msg import *
from dsr_msgs.srv import *

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"

JOG_AXIS = 0
REF      = 0
SPEED      = 10

def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"
    SPEED = 0
    jog(JOG_AXIS, REF, SPEED)
    pub_stop.publish(stop_mode=1) # STOP_TYPE_QUICK
    return 0

if __name__ == "__main__":
    rospy.init_node('jog_basic_py')
    rospy.on_shutdown(shutdown)

    jog            = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/motion/jog', Jog)
    set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/system/set_robot_mode', SetRobotMode)
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    set_robot_mode(ROBOT_MODE_MANUAL)  ### jog is only available in manual mode.

    if len(sys.argv) < 4:
        print("usage : jog_basic.py jog_axis reference, speed")
        print("default setting")
        print("jog_axis : 0, move_reference : base, speed : 10%")
        JOG_AXIS = 0
        REF      = 0
        SPEED      = 10

    else:
        print("jog_basic.py %d %d %5.3f" % (int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])))
        JOG_AXIS = int(sys.argv[1])
        REF      = int(sys.argv[2])
        SPEED      = float(sys.argv[3])

    jog(JOG_AXIS, REF, SPEED)

    while not rospy.is_shutdown():
        pass

    print 'good bye!'
