#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   
# TEST 2019-12-09
import rospy
import os
import threading, time
import sys
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0509"
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
    rospy.init_node('single_robot_simple_py')
    rospy.on_shutdown(shutdown)
    set_robot_mode  = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_mode', SetRobotMode)
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    set_velx(30,20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60,40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    velx=[50, 50]
    accx=[100, 100]

    p1= posj(0,0,0,0,0,0)                    #joint
    p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint

    x1= posx(367, 37.6, 540.5, 45, 180, 45) #task
    x2= posx(367, 10, 540.5, 62.0, 180, 62.0) #task

    c1 = posx(367, 40, 540.5, 12, 180, 12)
    c2 = posx(367, 10, 490, 12, 180, 12)


    q0 = posj(0,0,90,0,90,0)
    q1 = posj(10,0,90,10,90,0)
    q2 = posj(0,10,90,10,90,0)
    q3 = posj(0,10,80,0,90,0)
    q4 = posj(0,0,100,0,90,0)
    q5 = posj(20,0,90,20,90,0)
    qlist = [q0, q1, q2, q3, q4, q5]

    x1 = posx(330, 185, 497, 110, 160, 90)
    x2 = posx(360, 185, 497, 110, 160, 90)
    x3 = posx(310, 185, 497, 110, 160, 90)
    x4 = posx(310, 185, 397, 110, 160, 90)
    x5 = posx(310, 185, 397, 110, 160, 90)
    xlist = [x1, x2, x3, x4, x5, x5]


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
        movej(p2, vel=60, acc=30)
        movejx(x1, vel=60, acc=60, sol=2)
        movel(x2, velx, accx)
        movec(c1, c2, velx, accx)
        movesj(qlist, vel=100, acc=100)
        movesx(xlist, vel=100, acc=100)
        movej(p2, vel=60, acc=30)
        move_spiral(rev=1.00, rmax=20.00, lmax=20.00, time=5.00, axis=DR_AXIS_Z, ref=DR_TOOL)
        move_periodic(amp=[10.00, 0.00, 20.00, 0.00, 0.50, 0.00], period=[1.00, 0.00, 1.50, 0.00, 0.00, 0.00], atime=0.50, repeat=3, ref=DR_BASE)

    print('good bye!')
