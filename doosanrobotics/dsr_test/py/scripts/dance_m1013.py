#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../common/imp")) ) # get import pass : DSR_ROBOT.py 

# for single robot 
import DR_init
DR_init.__dsr__id = "dsr01"
DR_init.__dsr__model = "m1013"
from DSR_ROBOT import *

# for mulit robot 
###from DSR_ROBOT_MULTI import *

def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

    pub_stop_r1.publish(stop_mode=STOP_TYPE_QUICK)
    pub_stop_r2.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        
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
    #print "00000000000000000"
    rospy.Subscriber('/doosan_robot/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    
    return 0

def thread_robot1():
    r1 = CDsrRobot("dsr01","m1013")

    p1 = posj(0, 0, 90, 0, 90, 0)
    p2 = posj(0, 0,  0, 0,  0, 0)

    while not rospy.is_shutdown():    
        print("r1.movej(p1, v=100, a= 100)")
        r1.movej(p1, v=100, a= 100)
        time.sleep(1)

        print("r1.movej(p2, v=100, a= 100)")
        r1.movej(p2, v=100, a= 100)
        time.sleep(1)
    return 0

def thread_robot2():
    r2 = CDsrRobot("dsr02","m0609")

    p1 = posj(0, 0, 90, 0, 90, 0)
    p2 = posj(0, 0,  0, 0,  0, 0)

    while not rospy.is_shutdown():    
        print("r2.movej(p1, v=100, a= 100)")
        r2.movej(p1, v=100, a= 100)
        time.sleep(1)

        print("r2.movej(p2, v=100, a= 100)")
        r2.movej(p2, v=100, a= 100)
        time.sleep(1)
    return 0

if __name__ == "__main__":
    rospy.init_node('dance_py')
    rospy.on_shutdown(shutdown)

    _robot_id1 = "dsr01"; _robot_model1 = "m1013"
    _robot_id2 = "dsr02"; _robot_model2 = "m0609"
    _sev_prefix_robot1 ='/' + _robot_id1 + _robot_model1
    _sev_prefix_robot2 ='/' + _robot_id2 + _robot_model2

    r1 = CDsrRobot("dsr01","m1013")
    r2 = CDsrRobot("dsr02","m0609")

    pub_stop_r1  = rospy.Publisher(_sev_prefix_robot1 +'/stop', RobotStop, queue_size=10)           
    pub_stop_r2 = rospy.Publisher(_sev_prefix_robot2 +'/stop', RobotStop, queue_size=10)           

    '''
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    t2 = threading.Thread(target=thread_robot1)
    t2.daemon = True 
    t2.start()

    t2 = threading.Thread(target=thread_robot2)
    t2.daemon = True 
    t2.start()
    '''

    p1 = posj(0, 0, 90, 0, 90, 0)
    p2 = posj(0, 0,  0, 0,  0, 0)

    while 1:
        movej(p1, v=100, a=100)
        movej(p2, v=100, a=100)

        r1.movej(p1, v=100, a=100)
        r1.movej(p2, v=100, a=100)

        r2.movej(p1, v=100, a=100)
        r2.movej(p2, v=100, a=100)


    while not rospy.is_shutdown():    
        time.sleep(1)

    #----------------------------------------------------------------------
    '''
    JReady = posj(0, -20, 110, 0, 60, 0)

    J00 = posj(-180, 0, -145, 0, -35, 0)


    J01r = posj(-180.0, 71.4, -145.0, 0.0, -9.7, 0.0)
    J02r = posj(-180.0, 67.7, -144.0, 0.0, 76.3, 0.0)
    J03r = posj(-180.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    J04r = posj(-90.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    J04r1 = posj(-90.0, 30.0, -60.0, 0.0, 30.0, -0.0)
    J04r2 = posj(-90.0, -45.0, 90.0, 0.0, -45.0, -0.0)
    J04r3 = posj(-90.0, 60.0, -120.0, 0.0, 60.0, -0.0)
    J04r4 = posj(-90.0, 0.0, -0.0, 0.0, 0.0, -0.0)

    J05r = posj(-144.0, -4.0, -84.8, -90.9, 54.0, -1.1)

    J07r = posj(-152.4, 12.4, -78.6, 18.7, -68.3, -37.7)
    J08r = posj(-90.0, 30.0, -120.0, -90.0, -90.0, 0.0)

    JEnd = posj(0.0, -12.6, 101.1, 0.0, 91.5, -0.0)

    dREL1 = posx(0, 0, 350, 0, 0, 0)
    dREL2 = posx(0, 0, -350, 0, 0, 0)

    velx = [0, 0]
    accx = [0, 0]

    vel_spi = [400, 400]
    acc_spi = [150, 150]

    J1 = posj(81.2, 20.8, 127.8, 162.5, 56.1, -37.1)
    X0 = posx(-88.7, 799.0, 182.3, 95.7, 93.7, 133.9)
    X1 = posx(304.2, 871.8, 141.5, 99.5, 84.9, 133.4)
    X2 = posx(437.1, 876.9, 362.1, 99.6, 84.0, 132.1)
    X3 = posx(-57.9, 782.4, 478.4, 99.6, 84.0, 132.1)

    amp = [0, 0, 0, 30, 30, 0]
    period = [0, 0, 0, 3, 6, 0]

    x01 = [423.6, 334.5, 651.2, 84.7, -180.0, 84.7]
    x02 = [423.6, 34.5, 951.2, 68.2, -180.0, 68.2]
    x03 = [423.6, -265.5, 651.2, 76.1, -180.0, 76.1]
    x04 = [423.6, 34.5, 351.2, 81.3, -180.0, 81.3]

    while not rospy.is_shutdown():    
        r1.movej(JReady, 20, 20)
        r1.movej(J1, time = 3)

        r1.movel(X3, time = 2.5)

        for i in range(1, 3):
            r1.movel(X2, time = 2.5, r = 50)
            r1.movel(X1, time = 1.5, r = 50)           
            r1.movel(X0, time = 2.5)            
            r1.movel(X1, time = 2.5, r = 50)
            r1.movel(X2, time = 1.5, r = 50)
            r1.movel(X3, time = 2.5, r = 50)

        r1.movej(J00, time = 6)
        r1.movej(J01r, time = 2, r = 100)
        r1.movej(J02r, time = 2, r = 50)
        r1.movej(J03r, time = 2)
        r1.movej(J04r, time = 1.5)
        r1.movej(J04r1, time = 2, r = 50)
        r1.movej(J04r2, time = 4, r = 50)
        r1.movej(J04r3, time = 4, r = 50)
        r1.movej(J04r4, time = 2)
        r1.movej(J05r, time = 2.5, r = 100)
        r1.movel(dREL1, time = 1, ref = DR_TOOL, r = 50)
        r1.movel(dREL2, time = 1.5, ref = DR_TOOL, r = 100)
        r1.movej(J07r, time = 1.5, r = 100)
        r1.movej(J08r, time = 2)
        r1.movej(JEnd, time = 4)
        r1.move_periodic([0,0,0,30,30,0],[0,0,0,3,6,0], atime=0,repeat=1, ref=DR_TOOL)
        r1.move_spiral (rev=3, rmax=200, lmax=100, vel=400, acc=150, axis=DR_AXIS_X, ref=DR_TOOL)
        r1.movel(x01, time = 2)
        r1.movel(x04, time = 2, r = 100)
        r1.movel(x03, time = 2, r = 100)
        r1.movel(x02, time = 2, r = 100)
        r1.movel(x01, time = 2)
        r1.movec(x02, x04, time = 4, angle = 360)
    #----------------------------------------------------------------------
    '''
    print 'good bye!'
