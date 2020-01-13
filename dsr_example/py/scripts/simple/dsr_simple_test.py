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
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0509"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

velx=[50, 50]
accx=[100, 100]

p1= posj(0,0,0,0,0,0)                    #joint
p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint

x1= posx(400, 500, 800.0, 0.0, 180.0, 0.0) #task
x2= posx(400, 500, 500.0, 0.0, 180.0, 0.0) #task

c1 = posx(559,434.5,651.5,0,180,0)
c2 = posx(559,434.5,251.5,0,180,0)


q0 = posj(0,0,90,0,90,0)
q1 = posj(10,0,90,10,90,0)
q2 = posj(0,10,90,10,90,0)
q3 = posj(0,10,80,0,90,0)
q4 = posj(0,0,100,0,90,0)
q5 = posj(20,0,90,20,90,0)
qlist = [q0, q1, q2, q3, q4, q5]

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

drl_script = "set_velj(60)\nset_accj(30)\nmovej([0,0,90,0,90,0])\nmovej([0,0,0,0,0,0])\n"
dev_ip = "192.168.137.50"

def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

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

def motion_test():
    movej(p2, vel=60, acc=30)
    print("movej")

    temp, sol1 = get_current_posx()
    x1 = list(temp)
    x1[2] = x1[2] + 100
    movejx(x1, vel=60, acc=60, sol=2)
    print("movejx")

    temp, sol1 = get_current_posx()
    x2 = list(temp)
    x2[1] = x2[1] + 10
    movel(x2, velx, accx)
    print("movel")

    temp, xol1 = get_current_posx()
    c1 = list(temp)
    c1[1] = c1[1] + 30
    c2 = list(temp)
    c2[2] = c2[2] - 50
    movec(c1, c2, velx, accx)
    print("movec")

    movesj(qlist, vel=100, acc=100)
    print("movesj")

    temp, sol1 = get_current_posx()
    temp = list(temp)
    xlist = []
    temp[2] = temp[2] + 50
    temp = posx(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5])
    xlist.append(temp)
    temp[1] = temp[1] + 20
    temp = posx(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5])
    xlist.append(temp)
    temp[0] = temp[0] + 30
    temp = posx(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5])
    xlist.append(temp)
    temp[0] = temp[0] - 50
    temp = posx(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5])
    xlist.append(temp)
    temp[2] = temp[2] - 100
    temp = posx(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5])
    xlist.append(temp)
    movesx(xlist, vel=100, acc=100)
    print("movesx")

    movej(p2, vel=60, acc=30)
    ret = move_spiral(rev=1.00, rmax=20.00, lmax=20.00, time=5.00, axis=DR_AXIS_Z, ref=DR_TOOL)
    print("movespiral")

    move_periodic(amp=[10.00, 0.00, 20.00, 0.00, 0.50, 0.00], period=[1.00, 0.00, 1.50, 0.00, 0.00, 0.00], atime=0.50, repeat=3, ref=DR_BASE)
    print("moveperiodic")

    #moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)

def io_test():
    set_digital_output(1, ON)
    wait(1)
    if get_digital_input(1) == ON:
        rospy.loginfo("ctrl port 1 is ON")
        set_digital_output(1, OFF)
    else:
        rospy.loginfo("ctrl port 1 is OFF")
    
    set_tool_digital_output(1, ON)
    wait(1)
    if get_tool_digital_input(1) == ON:
        rospy.loginfo("tool port 1 is ON")
        set_tool_digital_output(1, OFF)
    else:
        rospy.loginfo("tool port 1 is OFF")

def drl_test(robotsystem):
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    if get_robot_mode() == ROBOT_MODE_AUTONOMOUS:
        drl_script_run(robotsystem, drl_script)
    if get_drl_state() == DRL_PROGRAM_STATE_STOP:
        set_robot_mode(ROBOT_MODE_MANUAL)
    #set_robot_mode(ROBOT_MODE_MANUAL)
    

def modbus_test():
    add_modbus_signal(dev_ip, 502, "ACT", DR_MODBUS_REG_OUTPUT, 0, 0)
    add_modbus_signal(dev_ip, 502, "WID", DR_MODBUS_REG_OUTPUT, 1, 0)
    add_modbus_signal(dev_ip, 502, "FOR", DR_MODBUS_REG_OUTPUT, 2, 0)
    add_modbus_signal(dev_ip, 502, "ACT_FEED", DR_MODBUS_REG_INPUT, 0, 0)
    add_modbus_signal(dev_ip, 502, "WID_FEED", DR_MODBUS_REG_INPUT, 1, 0)

    wait(0.5)

    set_modbus_output("ACT" , 2304)
    rospy.loginfo("ACT : " + str(get_modbus_input("ACT")))
    set_modbus_output("WID", 255)
    rospy.loginfo("WID : " + str(get_modbus_input("WID")))
    set_modbus_output("FOR", 65535)
    rospy.loginfo("FOR : " + str(get_modbus_input("FOR")))

    wait(0.5)

    del_modbus_signal("ACT")
    del_modbus_signal("WID")
    del_modbus_signal("FOR")

def tcp_test():
    add_tcp  = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tcp/config_create_tcp', ConfigCreateTcp)
    del_tcp  = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tcp/config_delete_tcp', ConfigDeleteTcp)

    add_tcp("tcp1", [0, 0, 0, 0, 0, 0])
    set_tcp("tcp1")
    rospy.loginfo("current tcp : " + str(get_tcp()))
    wait(0.5)
    del_tcp("tcp1")

def tool_test():
    fCog = [10.0, 10.0, 10.0]
    finertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    add_tool  = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tool/config_create_tool', ConfigCreateTool)
    del_tool  = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tool/config_delete_tool', ConfigDeleteTool)

    add_tool("tool1", 5.3, fCog, finertia)
    set_tool("tool1")
    rospy.loginfo("current tool : " + str(get_tool()))
    wait(0.5)
    del_tool("tool1")

def system_test():
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    rospy.loginfo("robot_mode : " + str(get_robot_mode()))
    set_robot_mode(ROBOT_MODE_MANUAL)
    rospy.loginfo("robot_mode : " + str(get_robot_mode()))

    temp = get_robot_system()
    set_robot_system(ROBOT_SYSTEM_REAL)
    rospy.loginfo("robot_system : " + str(get_robot_system()))
    set_robot_system(ROBOT_SYSTEM_VIRTUAL)
    rospy.loginfo("robot_system : " + str(get_robot_system()))
    set_robot_system(temp)

    rospy.loginfo("robot_state : " + str(get_robot_state()))

    temp = get_robot_speed_mode()
    set_robot_speed_mode(SPEED_NORMAL_MODE)
    rospy.loginfo("robot_speed_mode : " + str(get_robot_speed_mode()))
    set_robot_speed_mode(SPEED_REDUCED_MODE)
    rospy.loginfo("robot_speed_mode : " + str(get_robot_speed_mode()))
    set_robot_speed_mode(temp)

    set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT)

    rospy.loginfo("current joint pos : " + str(get_current_pose(ROBOT_SPACE_JOINT)))
    rospy.loginfo("current task pos : " + str(get_current_pose(ROBOT_SPACE_TASK)))
    rospy.loginfo("current solution space : " + str(get_current_solution_space()))

    rospy.loginfo("get_current_posj : " + str(get_current_posj()))
    rospy.loginfo("get_current_posx : " + str(get_current_posx()))
    rospy.loginfo("get_external_torque : " + str(get_external_torque()))
    rospy.loginfo("get_joint_torque : " + str(get_joint_torque()))
    rospy.loginfo("get_tool_force : " + str(get_tool_force()))
    rospy.loginfo("get_last_alarm : " + str(get_last_alarm()))



if __name__ == "__main__":
    rospy.init_node('single_robot_simple_py')
    rospy.on_shutdown(shutdown)
    set_robot_mode  = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_mode', SetRobotMode)
    #t1 = threading.Thread(target=thread_subscriber)
    #t1.daemon = True 
    #t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    set_velx(30,20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60,40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    while not rospy.is_shutdown():
        motion_test()
        #io_test()
        #drl_test(0)
        #modbus_test()
        #tcp_test()
        #tool_test()
        #system_test()
        break
    print 'good bye!'
