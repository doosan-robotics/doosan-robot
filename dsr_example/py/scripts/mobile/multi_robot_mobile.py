#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example gripper] gripper test for doosan robot
# @author   Jin Hyuk Gong (jinhyuk.gong@doosan.com)   

import rospy
import os
import threading, time
import sys
import random
from geometry_msgs.msg import Twist

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID1    = "dsr01"
ROBOT_ID2    = "dsr02"
ROBOT_MODEL1 = "m1013"
ROBOT_MODEL2 = "m1013"

RAND_MAX = 2147483647

#import DR_init
#DR_init.__dsr__id = ROBOT_ID1
#DR_init.__dsr__model = ROBOT_MODEL1
from DSR_ROBOT import *

NUM_ROBOT = 2

def thread_mobile1():         
    msg = Twist()
    while not rospy.is_shutdown():
        msg.linear.x = (random.random())/RAND_MAX + 1
        msg.angular.z = 2*(random.random())/RAND_MAX - 1
        pubMobile1.publish(msg)

def thread_mobile2():
    msg = Twist()
    while not rospy.is_shutdown():
        msg.linear.x = (random.random())/RAND_MAX + 1
        msg.angular.z = 2*(random.random())/RAND_MAX - 1
        pubMobile2.publish(msg)

def SET_ROBOT1(id, model):
    ROBOT_ID1 = id; ROBOT_MODEL1= model   


def SET_ROBOT2(id, model):
    ROBOT_ID2 = id; ROBOT_MODEL2= model   

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop1.publish(stop_mode=1) #STOP_TYPE_QUICK)
    pub_stop2.publish(stop_mode=1)
    return 0

# convert list to Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res
 
if __name__ == "__main__":
    #----- set target robot --------------- 
    my_robot_id1    = "dsr01"
    my_robot_id2    = "dsr02"
    my_robot_model1 = "m1013"
    my_robot_model2 = "m1013"
    SET_ROBOT1(my_robot_id1, my_robot_model1)
    SET_ROBOT2(my_robot_id2, my_robot_model2)

    rospy.init_node('single_robot_mobile_py')
    rospy.on_shutdown(shutdown)

    pub_stop1  = rospy.Publisher('/' + ROBOT_ID1 + ROBOT_MODEL1 + '/stop', RobotStop, queue_size=10)         
    pub_stop2  = rospy.Publisher('/' + ROBOT_ID2 + ROBOT_MODEL2 + '/stop', RobotStop, queue_size=10)
    pubMobile1 = rospy.Publisher('/' + ROBOT_ID1 + '/twist_marker_server/cmd_vel', Twist, queue_size=10)
    pubMobile2 = rospy.Publisher('/' + ROBOT_ID2 + '/twist_marker_server/cmd_vel', Twist, queue_size=10)  
    
    r1 = CDsrRobot(my_robot_id1, my_robot_model1)
    r2 = CDsrRobot(my_robot_id2, my_robot_model2)

    mThread1 = threading.Thread(target = thread_mobile1)
    mThread1.daemon = True
    mThread1.start()

    mThread2 = threading.Thread(target = thread_mobile2)
    mThread2.daemon = True
    mThread2.start()



    set_velx(30,20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60,40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    velx=[50, 50]
    accx=[100, 100]

    p1= posj(0,0,0,0,0,0)                    #joint
    p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint

    x1= posx(400, 500, 800.0, 0.0, 180.0, 0.0) #task
    x2= posx(400, 500, 500.0, 0.0, 180.0, 0.0) #task

    c1 = posx(559,434.5,651.5,0,180,0)
    c2 = posx(559,434.5,251.5,0,180,0)


    q0 = posj(0,0,0,0,0,0)
    q1 = posj(10, -10, 20, -30, 10, 20)
    q2 = posj(25, 0, 10, -50, 20, 40) 
    q3 = posj(50, 50, 50, 50, 50, 50) 
    q4 = posj(30, 10, 30, -20, 10, 60)
    q5 = posj(20, 20, 40, 20, 0, 90)
    qlist = [q0, q1, q2, q3, q4, q5]

    x1 = posx(600, 600, 600, 0, 175, 0)
    x2 = posx(600, 750, 600, 0, 175, 0)
    x3 = posx(150, 600, 450, 0, 175, 0)
    x4 = posx(-300, 300, 300, 0, 175, 0)
    x5 = posx(-200, 700, 500, 0, 175, 0)
    x6 = posx(600, 600, 400, 0, 175, 0)
    xlist = [x1, x2, x3, x4, x5, x6]


    X1 =  posx(370, 670, 650, 0, 180, 0)
    X1a = posx(370, 670, 400, 0, 180, 0)
    X1a2= posx(370, 545, 400, 0, 180, 0)
    X1b = posx(370, 595, 400, 0, 180, 0)
    X1b2= posx(370, 670, 400, 0, 180, 0)
    X1c = posx(370, 420, 150, 0, 180, 0)
    X1c2= posx(370, 545, 150, 0, 180, 0)
    X1d = posx(370, 670, 275, 0, 180, 0)
    X1d2= posx(370, 795, 150, 0, 180, 0)


    seg11 = posb(DR_LINE, X1, radius=20)
    seg12 = posb(DR_CIRCLE, X1a, X1a2, radius=21)
    seg14 = posb(DR_LINE, X1b2, radius=20)
    seg15 = posb(DR_CIRCLE, X1c, X1c2, radius=22)
    seg16 = posb(DR_CIRCLE, X1d, X1d2, radius=23)
    b_list1 = [seg11, seg12, seg14, seg15, seg16]



    while not rospy.is_shutdown():
        r1.movej(p2, vel=100, acc=100)
        r2.movej(p2, vel=100, acc=100)
    
        r1.movejx(x1, vel=30, acc=60)
        r2.movejx(x1, vel=30, acc=60)

        r1.movel(x2, velx, accx)
        r2.movel(x2, velx, accx)
        
        r1.movec(c1, c2, velx, accx)
        r2.movec(c1, c2, velx, accx)

        r1.movesj(qlist, vel=100, acc=100)
        r2.movesj(qlist, vel=100, acc=100)

        r1.movesx(xlist, vel=100, acc=100)
        r2.movesx(xlist, vel=100, acc=100)
        
        r1.move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
        r2.move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
        
        r1.move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
        r2.move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
        
        r1.moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        r2.moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    print('good bye!')
