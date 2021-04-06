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
    
    #print 'wait services'
    #rospy.wait_for_service('/'+ROBOT_ID +ROBOT_MODEL+'/drl/drl_start')

    srv_robotiq_2f_open = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/serial_send_data', SerialSendData)
   
    p0 = posj(0, 0, 0, 0, 0, 0)
    p1 = posj(0, 0, 90, 0, 90, 0)
    p2 = posj(180, 0, 90, 0, 90, 0)

    x1 = posx(0, 0, -200, 0, 0, 0)
    x2 = posx(0, 0, 200, 0, 0, 0)
    velx = [50, 50]
    accx = [100, 100]

    init_data = bytes(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x00\x73\x30')
    #init_data = bytes(b'abcde')
    activation_data = bytes(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00\x72\xE1')
    open_data = bytes(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19')
    close_data = bytes(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29')
    
    time_cnt = 0

    while not rospy.is_shutdown():

        if time_cnt == 0:
            srv_robotiq_2f_open(init_data)
            #gripper_send_data(init_data)
            rospy.sleep(4)
            srv_robotiq_2f_open(activation_data)
            #gripper_send_data(activation_data)
            rospy.sleep(4)

        srv_robotiq_2f_open(open_data)
        #gripper_send_data(open_data)
        rospy.sleep(4)
        srv_robotiq_2f_open(close_data)
        #gripper_send_data(close_data)
        rospy.sleep(4)    
        time_cnt = time_cnt + 1

        

    print('good bye!')
