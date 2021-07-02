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
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

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
  
if __name__ == "__main__":
    rospy.init_node('dsr_simple_test_py')
    rospy.on_shutdown(shutdown)

    #t1 = threading.Thread(target=thread_subscriber)
    #t1.daemon = True 
    #t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

    set_velx(30,20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60,40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    JReady = [0, -20, 110, 0, 60, 0]

    TCP_POS = [0, 0, 0, 0, 0, 0]
    J00 = [-180, 0, -145, 0, -35, 0]

    J01r = [-180.0, 71.4, -145.0, 0.0, -9.7, 0.0]
    J02r = [-180.0, 67.7, -144.0, 0.0, 76.3, 0.0]
    J03r = [-180.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    J04r = [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    J04r1 = [-90.0, 30.0, -60.0, 0.0, 30.0, -0.0]
    J04r2 = [-90.0, -45.0, 90.0, 0.0, -45.0, -0.0]
    J04r3 = [-90.0, 60.0, -120.0, 0.0, 60.0, -0.0]
    J04r4 = [-90.0, 0.0, -0.0, 0.0, 0.0, -0.0]

    J05r = [-144.0, -4.0, -84.8, -90.9, 54.0, -1.1]
    J07r = [-152.4, 12.4, -78.6, 18.7, -68.3, -37.7]
    J08r = [-90.0, 30.0, -120.0, -90.0, -90.0, 0.0]

    JEnd = [0.0, -12.6, 101.1, 0.0, 91.5, -0.0]

    dREL1 = [0, 0, 350, 0, 0, 0]
    dREL2 = [0, 0, -350, 0, 0, 0]

    velx = [0, 0]
    accx = [0, 0]

    vel_spi = [400, 400]
    acc_spi = [150, 150]

    J1 = [81.2, 20.8, 127.8, 162.5, 56.1, -37.1]
    X0 = [-88.7, 799.0, 182.3, 95.7, 93.7, 133.9]
    X1 = [304.2, 871.8, 141.5, 99.5, 84.9, 133.4]
    X2 = [437.1, 876.9, 362.1, 99.6, 84.0, 132.1]
    X3 = [-57.9, 782.4, 478.4, 99.6, 84.0, 132.1]

    amp = [0, 0, 0, 30, 30, 0]
    period = [0, 0, 0, 3, 6, 0]

    x01 = [423.6, 334.5, 651.2, 84.7, -180.0, 84.7]
    x02 = [423.6, 34.5, 951.2, 68.2, -180.0, 68.2]
    x03 = [423.6, -265.5, 651.2, 76.1, -180.0, 76.1]
    x04 = [423.6, 34.5, 351.2, 81.3, -180.0, 81.3]
    x0204c = [x02, x04]

    while not rospy.is_shutdown():
        movej(JReady, v=20, a=20)

        movej(J1, v=0, a=0, t=3)
        movel(X3, velx, accx, t=2.5)
        
        for i in range(0, 1):
            movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(X0, velx, accx, t=2.5) 
            movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        

        movej(J00, v=60, a=60, t=6)

        movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)
        movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 
        movej(J03r, v=0, a=0, t=2)

        movej(J04r, v=0, a=0, t=1.5)
        movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)
        movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)
        movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)
        movej(J04r4, v=0, a=0, t=2)

        movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 
        movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 
        movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

        movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 
        movej(J08r, v=60, a=60, t=2)

        movej(JEnd, v=60, a=60, t=4)

        move_periodic(amp, period, 0, 1, ref=DR_TOOL)
        move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

        movel(x01, velx, accx, t=2)
        movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(x01, velx, accx, t=2)  

        movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)

    print('good bye!')
