#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
#sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../common/imp")) ) # get import pass : DSR_ROBOT.py 


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
        self.nRobot = r
        self.nIsRun = True

        self.nWaitBit  = 0
        self.nCurBit   = 0

        self.bIsWait = list()        
        self.lock    = list() 

        for i in range(self.nRobot):
            self.nWaitBit |= (0x1<<i)         
            self.bIsWait.append(False)
            self.lock.append( threading.Lock() )

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
                    self.nCurBit |= (0x1<<i)                   
            if(self.nWaitBit == self.nCurBit):
                break
            time.sleep(0.01)
        for i in range(self.nRobot):
            self.lock[i].release()   
        return 0

RobotSync = CRobotSync(NUM_ROBOT)

#######################################################################################

def shutdown():
 
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"
    RobotSync.CleanUp()
    return 0

start_time1 = [0]
start_time2 = [0]

def thread_robot1(robot_id, robot_model):
    RobotID = 0
    while not rospy.is_shutdown():    
        RobotSync.Wait(RobotID)
        start_time1[0] = time.time()
        time.sleep(0.001)
    return 0

def thread_robot2(robot_id, robot_model):
    RobotID = 1
    while not rospy.is_shutdown():    
        RobotSync.Wait(RobotID)
        start_time2[0] = time.time()
        time.sleep(0.001)
    return 0

if __name__ == "__main__":

    rospy.init_node('robot_sync_test_py')
    rospy.on_shutdown(shutdown)

    robot_id1 = "dsr01"; robot_model1 = "m1013"
    robot_id2 = "dsr02"; robot_model2 = "m1013"

    t1 = threading.Thread(target=thread_robot1, args=(robot_id1, robot_model1))
    t1.daemon = True 
    t1.start()

    t2 = threading.Thread(target=thread_robot2, args=(robot_id2, robot_model2))
    t2.daemon = True 
    t2.start()

    time.sleep(1)

    max_time = 0.0
    cur_time = 0.0

    while not rospy.is_shutdown():    
        time.sleep(0.001)
        RobotSync.WakeUpAll()
        cur_time = abs(start_time1[0] - start_time2[0])*1000
        if cur_time > max_time:
            max_time = cur_time        
            print("Max Sync Time = %.3f ms" % max_time )

    #----------------------------------------------------------------------

    print 'good bye!'
