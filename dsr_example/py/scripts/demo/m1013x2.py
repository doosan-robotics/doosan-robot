#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py demo]  muliti robot sync test
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
#import DR_init
#DR_init.__dsr__id = "dsr01"
#DR_init.__dsr__model = "m1013"
#from DSR_ROBOT import *

# for mulit robot 
########from DSR_ROBOT_MULTI import *
from DSR_ROBOT import *

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop_r1.publish(stop_mode=STOP_TYPE_QUICK)
    pub_stop_r2.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb_r1(msg):
    msgRobotState_cb_r1.count += 1

    if (0==(msgRobotState_cb_r1.count % 100)): 
        rospy.loginfo("________ ROBOT[1] STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        
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
msgRobotState_cb_r1.count = 0

def msgRobotState_cb_r2(msg):
    msgRobotState_cb_r2.count += 1

    if (0==(msgRobotState_cb_r2.count % 100)): 
        rospy.loginfo("________ ROBOT[2] STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        
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
msgRobotState_cb_r2.count = 0

def thread_subscriber_r1(robot_id, robot_model):
    rospy.Subscriber('/'+ robot_id + robot_model +'/state', RobotState, msgRobotState_cb_r1)
    rospy.spin()
    #rospy.spinner(2)    

def thread_subscriber_r2(robot_id, robot_model):
    rospy.Subscriber('/'+ robot_id + robot_model +'/state', RobotState, msgRobotState_cb_r2)
    rospy.spin()
    #rospy.spinner(2)    

if __name__ == "__main__":
    rospy.init_node('m1013x2_amove_py')
    rospy.on_shutdown(shutdown)

    robot_id1 = "dsr01"; robot_model1 = "m1013"
    robot_id2 = "dsr02"; robot_model2 = "m1013"

    r1 = CDsrRobot(robot_id1,robot_model1)
    r2 = CDsrRobot(robot_id2,robot_model2)

    pub_stop_r1 = rospy.Publisher('/'+ robot_id1 + robot_model1 +'/stop', RobotStop, queue_size=10)           
    pub_stop_r2 = rospy.Publisher('/'+ robot_id2 + robot_model2 +'/stop', RobotStop, queue_size=10)           

    #t1 = threading.Thread(target=thread_subscriber_r1, args=(robot_id1, robot_model1))
    #t1.daemon = True 
    #t1.start()

    #t2 = threading.Thread(target=thread_subscriber_r2, args=(robot_id2, robot_model2))
    #t2.daemon = True 
    #t2.start()

    #----------------------------------------------------------------------
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
        r1.amovej(JReady, 20, 20)
        r2.amovej(JReady, 20, 20)
        r1.mwait(); r2.mwait() 

        r1.amovej(J1, time = 3)
        r2.amovej(J1, time = 3)
        r1.mwait(); r2.mwait() 

        r1.amovel(X3, time = 2.5)
        r2.amovel(X3, time = 2.5)
        r1.mwait(); r2.mwait() 

        for i in range(1, 3):
            r1.amovel(X2, time = 2.5, r = 50)
            r2.amovel(X2, time = 2.5, r = 50)
            r1.mwait(); r2.mwait() 

            r1.amovel(X1, time = 1.5, r = 50)
            r2.amovel(X1, time = 1.5, r = 50)
            r1.move_wait(); r2.move_wait() 
            
            r1.amovel(X0, time = 2.5)
            r2.amovel(X0, time = 2.5)
            r1.move_wait(); r2.move_wait() 
            
            r1.amovel(X1, time = 2.5, r = 50)
            r2.amovel(X1, time = 2.5, r = 50)
            r1.move_wait(); r2.move_wait() 
            
            r1.amovel(X2, time = 1.5, r = 50)
            r2.amovel(X2, time = 1.5, r = 50)
            r1.move_wait(); r2.move_wait() 
            
            r1.amovel(X3, time = 2.5, r = 50)
            r2.amovel(X3, time = 2.5, r = 50)
            r1.move_wait(); r2.move_wait() 

        r1.amovej(J00, time = 6)
        r2.amovej(J00, time = 6)
        r1.move_wait(); r2.move_wait() 

        r1.amovej(J01r, time = 2, r = 100)
        r2.amovej(J01r, time = 2, r = 100)
        r1.move_wait(); r2.move_wait() 
       
        r1.amovej(J02r, time = 2, r = 50)
        r2.amovej(J02r, time = 2, r = 50)
        r1.move_wait(); r2.move_wait() 
        
        r1.amovej(J03r, time = 2)
        r2.amovej(J03r, time = 2)
        r1.move_wait(); r2.move_wait() 
        
        r1.amovej(J04r, time = 1.5)
        r2.amovej(J04r, time = 1.5)
        r1.move_wait(); r2.move_wait() 
        
        r1.amovej(J04r1, time = 2, r = 50)
        r2.amovej(J04r1, time = 2, r = 50)
        r1.move_wait(); r2.move_wait() 
        
        r1.amovej(J04r2, time = 4, r = 50)
        r2.amovej(J04r2, time = 4, r = 50)
        r1.move_wait(); r2.move_wait() 
        
        r1.amovej(J04r3, time = 4, r = 50)
        r2.amovej(J04r3, time = 4, r = 50)
        r1.move_wait(); r2.move_wait() 
        
        r1.amovej(J04r4, time = 2)
        r2.amovej(J04r4, time = 2)
        r1.move_wait(); r2.move_wait() 

        r1.amovej(J05r, time = 2.5, r = 100)
        r2.amovej(J05r, time = 2.5, r = 100)
        r1.move_wait(); r2.move_wait() 

        r1.amovel(dREL1, time = 1, ref = DR_TOOL, r = 50)
        r2.amovel(dREL1, time = 1, ref = DR_TOOL, r = 50)
        r1.move_wait(); r2.move_wait() 

        r1.amovel(dREL2, time = 1.5, ref = DR_TOOL, r = 100)
        r2.amovel(dREL2, time = 1.5, ref = DR_TOOL, r = 100)
        r1.move_wait(); r2.move_wait() 

        r1.amovej(J07r, time = 1.5, r = 100)
        r2.amovej(J07r, time = 1.5, r = 100)
        r1.move_wait(); r2.move_wait() 

        r1.amovej(J08r, time = 2)
        r2.amovej(J08r, time = 2)
        r1.move_wait(); r2.move_wait() 

        r1.amovej(JEnd, time = 4)
        r2.amovej(JEnd, time = 4)
        r1.move_wait(); r2.move_wait() 

        r1.amove_periodic([0,0,0,30,30,0],[0,0,0,3,6,0], atime=0,repeat=1, ref=DR_TOOL)
        r2.amove_periodic([0,0,0,30,30,0],[0,0,0,3,6,0], atime=0,repeat=1, ref=DR_TOOL)
        r1.move_wait(); r2.move_wait() 

        r1.amove_spiral (rev=3, rmax=200, lmax=100, vel=400, acc=150, axis=DR_AXIS_X, ref=DR_TOOL)
        r2.amove_spiral (rev=3, rmax=200, lmax=100, vel=400, acc=150, axis=DR_AXIS_X, ref=DR_TOOL)
        r1.move_wait(); r2.move_wait() 

        r1.amovel(x01, time = 2)
        r2.amovel(x01, time = 2)
        r1.move_wait(); r2.move_wait() 

        r1.amovel(x04, time = 2, r = 100)
        r2.amovel(x04, time = 2, r = 100)
        r1.move_wait(); r2.move_wait() 

        r1.amovel(x03, time = 2, r = 100)
        r2.amovel(x03, time = 2, r = 100)
        r1.move_wait(); r2.move_wait() 

        r1.amovel(x02, time = 2, r = 100)
        r2.amovel(x02, time = 2, r = 100)
        r1.move_wait(); r2.move_wait() 

        r1.amovel(x01, time = 2)
        r2.amovel(x01, time = 2)
        r1.move_wait(); r2.move_wait() 

        r1.amovec(x02, x04, time = 4, angle = 360)
        r2.amovec(x02, x04, time = 4, angle = 360)
        r1.move_wait(); r2.move_wait() 
    #----------------------------------------------------------------------

    print('good bye!')
