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
        print("  robot_state           : %d" % (msg.robot_state))
        print("  robot_state_str       : %s" % (msg.robot_state_str))
        print("  actual_mode           : %d" % (msg.actual_mode))
        print("  actual_space          : %d" % (msg.actual_space))
        print("  current_posj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        print("  current_velj          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_velj[0],msg.current_velj[1],msg.current_velj[2],msg.current_velj[3],msg.current_velj[4],msg.current_velj[5]))
        print("  joint_abs             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.joint_abs[0],msg.joint_abs[1],msg.joint_abs[2],msg.joint_abs[3],msg.joint_abs[4],msg.joint_abs[5]))
        print("  joint_err             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.joint_err[0],msg.joint_err[1],msg.joint_err[2],msg.joint_err[3],msg.joint_err[4],msg.joint_err[5]))
        print("  target_posj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.target_posj[0],msg.target_posj[1],msg.target_posj[2],msg.target_posj[3],msg.target_posj[4],msg.target_posj[5]))
        print("  target_velj           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.target_velj[0],msg.target_velj[1],msg.target_velj[2],msg.target_velj[3],msg.target_velj[4],msg.target_velj[5]))    
        print("  current_posx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))
        print("  current_velx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_velx[0],msg.current_velx[1],msg.current_velx[2],msg.current_velx[3],msg.current_velx[4],msg.current_velx[5]))
        print("  task_err              : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.task_err[0],msg.task_err[1],msg.task_err[2],msg.task_err[3],msg.task_err[4],msg.task_err[5]))
        print("  solution_space        : %d" % (msg.solution_space))
        sys.stdout.write("  rotation_matrix       : ")
        for i in range(0 , 3):
            sys.stdout.write(  "dim : [%d]"% i)
            sys.stdout.write("  [ ")
            for j in range(0 , 3):
                sys.stdout.write("%d " % msg.rotation_matrix[i].data[j])
            sys.stdout.write("] ")
        print ##end line
        print("  dynamic_tor           : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.dynamic_tor[0],msg.dynamic_tor[1],msg.dynamic_tor[2],msg.dynamic_tor[3],msg.dynamic_tor[4],msg.dynamic_tor[5]))
        print("  actual_jts            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_jts[0],msg.actual_jts[1],msg.actual_jts[2],msg.actual_jts[3],msg.actual_jts[4],msg.actual_jts[5]))
        print("  actual_ejt            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_ejt[0],msg.actual_ejt[1],msg.actual_ejt[2],msg.actual_ejt[3],msg.actual_ejt[4],msg.actual_ejt[5]))
        print("  actual_ett            : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_ett[0],msg.actual_ett[1],msg.actual_ett[2],msg.actual_ett[3],msg.actual_ett[4],msg.actual_ett[5]))
        print("  sync_time             : %7.3f" % (msg.sync_time))
        print("  actual_bk             : %d %d %d %d %d %d" % (msg.actual_bk[0],msg.actual_bk[1],msg.actual_bk[2],msg.actual_bk[3],msg.actual_bk[4],msg.actual_bk[5]))
        print("  actual_bt             : %d %d %d %d %d " % (msg.actual_bt[0],msg.actual_bt[1],msg.actual_bt[2],msg.actual_bt[3],msg.actual_bt[4]))
        print("  actual_mc             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_mc[0],msg.actual_mc[1],msg.actual_mc[2],msg.actual_mc[3],msg.actual_mc[4],msg.actual_mc[5]))
        print("  actual_mt             : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_mt[0],msg.actual_mt[1],msg.actual_mt[2],msg.actual_mt[3],msg.actual_mt[4],msg.actual_mt[5]))

        #print digital i/o
        sys.stdout.write("  ctrlbox_digital_input : ")
        for i in range(0 , 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_input[i])
        print ##end line
        sys.stdout.write("  ctrlbox_digital_output: ")
        for i in range(0 , 16):
            sys.stdout.write("%d " % msg.ctrlbox_digital_output[i])
        print
        sys.stdout.write("  flange_digital_input  : ")
        for i in range(0 , 6):
            sys.stdout.write("%d " % msg.flange_digital_input[i])
        print
        sys.stdout.write("  flange_digital_output : ")
        for i in range(0 , 6):
            sys.stdout.write("%d " % msg.flange_digital_output[i])
        print
        #print modbus i/o
        sys.stdout.write("  modbus_state          : " )
        if len(msg.modbus_state) > 0:
            for i in range(0 , len(msg.modbus_state)):
                sys.stdout.write("[" + msg.modbus_state[i].modbus_symbol)
                sys.stdout.write(", %d] " % msg.modbus_state[i].modbus_value)
        print

        print("  access_control        : %d" % (msg.access_control))
        print("  homming_completed     : %d" % (msg.homming_completed))
        print("  tp_initialized        : %d" % (msg.tp_initialized))
        print("  mastering_need        : %d" % (msg.mastering_need))
        print("  drl_stopped           : %d" % (msg.drl_stopped))
        print("  disconnected          : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    
  
if __name__ == "__main__":
    rospy.init_node('dsr_simple_test_py')
    rospy.on_shutdown(shutdown)

    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

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
        movej(p2, vel=100, acc=100)
        movejx(x1, vel=30, acc=60, sol=0)
        movel(x2, velx, accx)
        movec(c1, c2, velx, accx)
        movesj(qlist, vel=100, acc=100)
        movesx(xlist, vel=100, acc=100)
        move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
        move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
        moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    print('good bye!')