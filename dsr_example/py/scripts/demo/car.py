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
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 


# for single robot 
#import DR_init
#DR_init.__dsr__id = "dsr01"
#DR_init.__dsr__model = "m1013"
#from DSR_ROBOT import *

# for mulit robot 
########from DSR_ROBOT_MULTI import *
from DSR_ROBOT import *

NUM_ROBOT = 4

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

def thread_robot1(robot_id, robot_model):
    try:
        #nRobotID = 0
        r = CDsrRobot(robot_id, robot_model)
        
        while not rospy.is_shutdown():    
            #RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)
        
            #RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)

        '''
        while not rospy.is_shutdown():
            RobotSync.Wait(nRobotID)
            movej(JReady, v=20, a=20)

            RobotSync.Wait(nRobotID)
            movej(J1, v=0, a=0, t=3)

            RobotSync.Wait(nRobotID)
            movel(X3, velx, accx, t=2.5)

           
            for i in range(0, 1):
                RobotSync.Wait(nRobotID)
                movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X0, velx, accx, t=2.5) 
                RobotSync.Wait(nRobotID)
                movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
           
            RobotSync.Wait(nRobotID)
            movej(J00, v=60, a=60, t=6)

            RobotSync.Wait(nRobotID)
            movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movej(J03r, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            movej(J04r, v=0, a=0, t=1.5)

            RobotSync.Wait(nRobotID)
            movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J04r4, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movej(J08r, v=60, a=60, t=2)

            RobotSync.Wait(nRobotID)
            movej(JEnd, v=60, a=60, t=4)

            RobotSync.Wait(nRobotID)
            move_periodic(amp, period, 0, 1, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            movel(x01, velx, accx, t=2)

            RobotSync.Wait(nRobotID)
            movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movel(x01, velx, accx, t=2)     

            RobotSync.Wait(nRobotID)
            movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)  
        '''

    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot2(robot_id, robot_model):
    try:
        nRobotID = 0
        r = CDsrRobot(robot_id, robot_model)

        '''
        while not rospy.is_shutdown():    
            RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)
            RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)
        '''
        
        while not rospy.is_shutdown():
            RobotSync.Wait(nRobotID)
            r.movej(JReady, v=20, a=20)

            RobotSync.Wait(nRobotID)
            r.movej(J1, v=0, a=0, t=3)

            RobotSync.Wait(nRobotID)
            r.movel(X3, velx, accx, t=2.5)
           
            for i in range(0, 1):
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X0, velx, accx, t=2.5) 
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
           
            RobotSync.Wait(nRobotID)
            r.movej(J00, v=60, a=60, t=6)

            RobotSync.Wait(nRobotID)
            r.movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J03r, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J04r, v=0, a=0, t=1.5)

            RobotSync.Wait(nRobotID)
            r.movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r4, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J08r, v=60, a=60, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(JEnd, v=60, a=60, t=4)

            RobotSync.Wait(nRobotID)
            r.move_periodic(amp, period, 0, 1, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)

            RobotSync.Wait(nRobotID)
            r.movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)     

            RobotSync.Wait(nRobotID)
            r.movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)
 
    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot3(robot_id, robot_model):
    try:
        #nRobotID = 2
        r = CDsrRobot(robot_id, robot_model)
        
        
        while not rospy.is_shutdown():    
        #    RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)
        #    RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)
        
        '''
        while not rospy.is_shutdown():
            RobotSync.Wait(nRobotID)
            movej(JReady, v=20, a=20)

            RobotSync.Wait(nRobotID)
            movej(J1, v=0, a=0, t=3)

            RobotSync.Wait(nRobotID)
            movel(X3, velx, accx, t=2.5)

            for i in range(0, 1):
                RobotSync.Wait(nRobotID)
                movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X0, velx, accx, t=2.5) 
                RobotSync.Wait(nRobotID)
                movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
           
            RobotSync.Wait(nRobotID)
            movej(J00, v=60, a=60, t=6)

            RobotSync.Wait(nRobotID)
            movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movej(J03r, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            movej(J04r, v=0, a=0, t=1.5)

            RobotSync.Wait(nRobotID)
            movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movej(J04r4, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            movej(J08r, v=60, a=60, t=2)

            RobotSync.Wait(nRobotID)
            movej(JEnd, v=60, a=60, t=4)

            RobotSync.Wait(nRobotID)
            move_periodic(amp, period, 0, 1, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            movel(x01, velx, accx, t=2)

            RobotSync.Wait(nRobotID)
            movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            movel(x01, velx, accx, t=2)     

            RobotSync.Wait(nRobotID)
            movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)
        '''

    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot4(robot_id, robot_model):
    try:
        nRobotID = 1
        r = CDsrRobot(robot_id, robot_model)

        '''
        while not rospy.is_shutdown():    
            RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)
            RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)
        '''

        while not rospy.is_shutdown():
            RobotSync.Wait(nRobotID)
            r.movej(JReady, v=20, a=20)

            RobotSync.Wait(nRobotID)
            r.movej(J1, v=0, a=0, t=3)

            RobotSync.Wait(nRobotID)
            r.movel(X3, velx, accx, t=2.5)
           
            for i in range(0, 1):
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X0, velx, accx, t=2.5) 
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
           
            RobotSync.Wait(nRobotID)
            r.movej(J00, v=60, a=60, t=6)

            RobotSync.Wait(nRobotID)
            r.movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J03r, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J04r, v=0, a=0, t=1.5)

            RobotSync.Wait(nRobotID)
            r.movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r4, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J08r, v=60, a=60, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(JEnd, v=60, a=60, t=4)

            RobotSync.Wait(nRobotID)
            r.move_periodic(amp, period, 0, 1, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)

            RobotSync.Wait(nRobotID)
            r.movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)     

            RobotSync.Wait(nRobotID)
            r.movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)

    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot5(robot_id, robot_model):
    try:
        nRobotID = 2
        r = CDsrRobot(robot_id, robot_model)

        '''
        while not rospy.is_shutdown():    
            RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)
            RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)
        '''

        while not rospy.is_shutdown():
            RobotSync.Wait(nRobotID)
            r.movej(JReady, v=20, a=20)

            RobotSync.Wait(nRobotID)
            r.movej(J1, v=0, a=0, t=3)

            RobotSync.Wait(nRobotID)
            r.movel(X3, velx, accx, t=2.5)
           
            for i in range(0, 1):
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X0, velx, accx, t=2.5) 
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
           
            RobotSync.Wait(nRobotID)
            r.movej(J00, v=60, a=60, t=6)

            RobotSync.Wait(nRobotID)
            r.movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J03r, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J04r, v=0, a=0, t=1.5)

            RobotSync.Wait(nRobotID)
            r.movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r4, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J08r, v=60, a=60, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(JEnd, v=60, a=60, t=4)

            RobotSync.Wait(nRobotID)
            r.move_periodic(amp, period, 0, 1, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)

            RobotSync.Wait(nRobotID)
            r.movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)     

            RobotSync.Wait(nRobotID)
            r.movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)

    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def thread_robot6(robot_id, robot_model):
    try:
        nRobotID = 3
        r = CDsrRobot(robot_id, robot_model)

        '''
        while not rospy.is_shutdown():    
            RobotSync.Wait(nRobotID)
            r.movej(J0, v=20, a=20)
            RobotSync.Wait(nRobotID)
            r.movej(J1, v=20, a=20)
        '''

        while not rospy.is_shutdown():
            RobotSync.Wait(nRobotID)
            r.movej(JReady, v=20, a=20)

            RobotSync.Wait(nRobotID)
            r.movej(J1, v=0, a=0, t=3)

            RobotSync.Wait(nRobotID)
            r.movel(X3, velx, accx, t=2.5)
           
            for i in range(0, 1):
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X0, velx, accx, t=2.5) 
                RobotSync.Wait(nRobotID)
                r.movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                RobotSync.Wait(nRobotID)
                r.movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
           
            RobotSync.Wait(nRobotID)
            r.movej(J00, v=60, a=60, t=6)

            RobotSync.Wait(nRobotID)
            r.movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J03r, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J04r, v=0, a=0, t=1.5)

            RobotSync.Wait(nRobotID)
            r.movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movej(J04r4, v=0, a=0, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 

            RobotSync.Wait(nRobotID)
            r.movej(J08r, v=60, a=60, t=2)

            RobotSync.Wait(nRobotID)
            r.movej(JEnd, v=60, a=60, t=4)

            RobotSync.Wait(nRobotID)
            r.move_periodic(amp, period, 0, 1, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)

            RobotSync.Wait(nRobotID)
            r.movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            RobotSync.Wait(nRobotID)
            r.movel(x01, velx, accx, t=2)     

            RobotSync.Wait(nRobotID)
            r.movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)

    except Exception as err:
        RobotSync.CleanUp()
        rospy.loginfo("Runtime Exception : %s" % err)
    return 0

def shutdown():
 
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

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

    time.sleep(5)

    while not rospy.is_shutdown():    
        time.sleep(0.01)
        RobotSync.WakeUpAll()

    #----------------------------------------------------------------------

    print('good bye!')
