#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py demo]  car demo [robotx6 sync test]
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 


# for single robot 
#import DR_init
#DR_init.__dsr__id = "dsr01"
#DR_init.__dsr__model = "m1013"
#from DSR_ROBOT import *

# for mulit robot 
########from DSR_ROBOT_MULTI import *
from DSR_ROBOT import *

NUM_ROBOT = 2

############################################################################################
class CRobotSync:
    def __init__(self, r):
        self.description = "Sync for Multiple Robos"
        self.author = "Doosan Robotics"
        self.nRobot = r
        self.nIsRun = True

        self.nWaitBit  = 0
        self.nCurBit   = 0

        self.bIsWait = list()        
        self.lock    = list() 

        for i in range(r):
            self.lock.append( threading.Lock() )
            self.bIsWait.append(False)
            self.nWaitBit |= 0x1<<i         

    def CleanUp(self):
        if True == self.nIsRun:
            self.nIsRun = False
        print("~CleanUp()")

    def Wait(self, r):
        self.bIsWait[r] = True    
        self.lock[r].acquire()   
        self.bIsWait[r] = False      
        return 0

    def WakeUp(self, r):
        while self.nIsRun: 
            if(True == self.bIsWait[r]):        
                self.lock[r].release()   
                break;
            else:
                time.sleep(0.01)
        return 0

    def WakeUpAll(self):
        self.nCurBit = 0
        while self.nIsRun: 
            for i in range(self.nRobot):
                if(True == self.bIsWait[i]):        
                    self.nCurBit |= 0x1<<i;        
            if(self.nWaitBit == self.nCurBit):
                break;
        for i in range(self.nRobot):
            self.lock[i].release()   
        return 0

RobotSync = CRobotSync(NUM_ROBOT)

#######################################################################################


#----------------------------------------------------------------------
J0 = posj(0, 0, 0,  0,  0, 0)
J1 = posj(0, 0, 0, 30, 30, 0)

def thread_robot1(robot_id, robot_model):
    try:
        nRobotID = 0
        r = CDsrRobot(robot_id, robot_model)
        while not rospy.is_shutdown():    
            RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)

            RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)

    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot2(robot_id, robot_model):
    try:
        nRobotID = 1
        r = CDsrRobot(robot_id, robot_model)
        while not rospy.is_shutdown():    
            RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)

            RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)

    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot3(robot_id, robot_model):
    try:
        nRobotID = 2
        r = CDsrRobot(robot_id, robot_model)
        while not rospy.is_shutdown():    
            time.sleep(1)
            print("running...thread_robot 3")
    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot4(robot_id, robot_model):
    try:
        nRobotID = 3
        r = CDsrRobot(robot_id, robot_model)
        while not rospy.is_shutdown():    
            time.sleep(1)
            print("running...thread_robot 4")
    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot5(robot_id, robot_model):
    try:
        nRobotID = 4
        r = CDsrRobot(robot_id, robot_model)
        while not rospy.is_shutdown():    
            time.sleep(1)
            print("running...thread_robot 5")
    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot6(robot_id, robot_model):
    try:
        nRobotID = 5
        r = CDsrRobot(robot_id, robot_model)
        while not rospy.is_shutdown():    
            time.sleep(1)
            print("running...thread_robot 6")
    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def shutdown():
 
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

    pub_stop_r1.publish(stop_mode=STOP_TYPE_QUICK)
    pub_stop_r2.publish(stop_mode=STOP_TYPE_QUICK)
    pub_stop_r3.publish(stop_mode=STOP_TYPE_QUICK)
    pub_stop_r4.publish(stop_mode=STOP_TYPE_QUICK)
    pub_stop_r5.publish(stop_mode=STOP_TYPE_QUICK)
    pub_stop_r6.publish(stop_mode=STOP_TYPE_QUICK)

    return 0

if __name__ == "__main__":
    rospy.init_node('car_py')
    rospy.on_shutdown(shutdown)

    robot_id1 = "dsr01"; robot_model1 = "m0617"
    robot_id2 = "dsr02"; robot_model2 = "m1013"
    robot_id3 = "dsr03"; robot_model3 = "m1509"
    robot_id4 = "dsr04"; robot_model4 = "m1013"
    robot_id5 = "dsr05"; robot_model5 = "m1013"
    robot_id6 = "dsr06"; robot_model6 = "m1013"

    pub_stop_r1 = rospy.Publisher('/'+ robot_id1 + robot_model1 +'/stop', RobotStop, queue_size=10)           
    pub_stop_r2 = rospy.Publisher('/'+ robot_id2 + robot_model2 +'/stop', RobotStop, queue_size=10)           
    pub_stop_r3 = rospy.Publisher('/'+ robot_id3 + robot_model1 +'/stop', RobotStop, queue_size=10)           
    pub_stop_r4 = rospy.Publisher('/'+ robot_id4 + robot_model2 +'/stop', RobotStop, queue_size=10)           
    pub_stop_r5 = rospy.Publisher('/'+ robot_id5 + robot_model1 +'/stop', RobotStop, queue_size=10)           
    pub_stop_r6 = rospy.Publisher('/'+ robot_id6 + robot_model2 +'/stop', RobotStop, queue_size=10)           

    #RobotSync = CRobotSync(NUM_ROBOT)

    t1 = threading.Thread(target=thread_robot1, args=(robot_id1, robot_model1))
    t1.daemon = True 
    t1.start()

    t2 = threading.Thread(target=thread_robot2, args=(robot_id2, robot_model2))
    t2.daemon = True 
    t2.start()

    t3 = threading.Thread(target=thread_robot3, args=(robot_id3, robot_model3))
    t3.daemon = True 
    t3.start()

    t4 = threading.Thread(target=thread_robot4, args=(robot_id4, robot_model4))
    t4.daemon = True 
    t4.start()

    t5 = threading.Thread(target=thread_robot5, args=(robot_id5, robot_model5))
    t5.daemon = True 
    t5.start()

    t6 = threading.Thread(target=thread_robot6, args=(robot_id6, robot_model6))
    t6.daemon = True 
    t6.start()

    time.sleep(1)

    while not rospy.is_shutdown():    
        time.sleep(0.01)
        RobotSync.WakeUpAll()

    #----------------------------------------------------------------------

    print 'good bye!'
