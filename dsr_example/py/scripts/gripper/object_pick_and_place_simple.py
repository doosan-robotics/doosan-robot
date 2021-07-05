#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example gripper] gripper test for doosan robot
# @author   Jin Hyuk Gong (jinhyuk.gong@doosan.com)   

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

def robotiq_2f_open():
    srv_robotiq_2f_open()

def robotiq_2f_move(width):
    srv_robotiq_2f_move(width)

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
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
    my_robot_id    = "dsr01"
    my_robot_model = "m1013"
    SET_ROBOT(my_robot_id, my_robot_model)

    rospy.init_node('pick_and_place_simple_py')
    rospy.on_shutdown(shutdown)


    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)    
    srv_robotiq_2f_open = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_open', Robotiq2FOpen)       
    srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove)

    p0 = posj(0, 0, 0, 0, 0, 0)
    p1 = posj(0, 0, 90, 0, 90, 0)
    p2 = posj(180, 0, 90, 0, 90, 0)

    x1 = posx(0, 0, -200, 0, 0, 0)
    x2 = posx(0, 0, 200, 0, 0, 0)
    velx = [50, 50]
    accx = [100, 100]

    while not rospy.is_shutdown():
        movej(p0, vel=60, acc=30)

        movej(p1, vel=60, acc=30)

        movel(x1, velx, accx, 2, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)
        robotiq_2f_move(0.4)
        rospy.sleep(1)
        movel(x2, velx, accx, 2, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)

        movej(p2, vel=60, acc=30)

        movel(x1, velx, accx, 2, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)
        robotiq_2f_open()
        rospy.sleep(1)
        movel(x2, velx, accx, 2, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)

        

    print('good bye!')
