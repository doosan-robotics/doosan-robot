#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
from sensor_msgs.msg import Joy
import random
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 

from DR_tcp_client import *

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


G_GRIPPER_FORCE = 150
G_GRIPPER_SPEED = 150
G_JOG_VELOCITY = 100

FLAG_HOMMING = 0
FLAG_READY = 1
FLAG_MULTI_JOG_START = 2
FLAG_JOG_STOP = 3
FLAG_GRIPPER_OPEN = 4
FLAG_GRIPPER_CLOSE = 5
FLAG_END = 7
FLAG_DANCE = 8
FLAG_CONTROL = -1

FLAG_X_LIMIT = 0
FLAG_Y_LIMIT = 0
FLAG_Z_LIMIT = 0 #0: no error  1: upper limit  2: lower limit

G_GRIP_MOVE = 3 

CURRENT_STATE = -1
PRE_STATE = -1
STATE_JOG = 0
STATE_GRIPPING = 1
STATE_MOVING = 2
STATE_READY = 3
STATE_STAY = 4
STATE_DANCE = 5

FLAG_READY_COMPLETE = False
FLAG_GAME_STOP = True
STOP_WATCH_TIME = 60# 총 게임시간
G_POWER_SLEEP_TIME = 10#int(STOP_WATCH_TIME/12)#random.randrange(int(STOP_WATCH_TIME/12), int(STOP_WATCH_TIME/6)) # 대기시간
G_DIO2 = False
G_GRIPPER_STATUS = False # 0 CLOSE 1 OPEN
G_GRIPPER_PRE_STATUS = 0
GRIPPER_IS_OPEN = True
OBJECT_DETECT = False

#G_POWER_OFF_LIST = [14, 34, 54]#[random.randrange(1, int(STOP_WATCH_TIME/6)), random.randrange(int(STOP_WATCH_TIME/3), int(STOP_WATCH_TIME/2)), random.randrange(int(2*STOP_WATCH_TIME/3), int(5*STOP_WATCH_TIME/6))]
#G_POWER_ON_LIST = [20, 40]

G_SENSOR_FLAG = False
G_POWER_OFF_TIME = 0
G_POWER_ON_TIME = 0

g_dance_stop = True

HOMMING_POS = [0, 0, 0, 0, 0, 0]
#READY_POS = [0, 21, -113, 0, -88, 0] ## Ready 자세
READY_POS = [0, 0, 90, 0, 90, 0]
#READY_POS_TASK = [-537.7, 70.1, 482.4, 175.0, 180.0, 175.4] 
#Z_LIMIT = [490, 710]
#X_LIMIT = [-755, -240]
#Y_LIMIT = [-550, 650]
Z_LIMIT = [0, 1000]
X_LIMIT = [-1000, 1000]
Y_LIMIT = [-1000, 1000]
FLAG_POWER = False
GAME_END = False

SKIP_JOG = False
G_MOVE_READY_VEL = 60
G_MOVE_READY_ACC = 30
m_joyAnalogFlag = False
m_joyButtonFlag = False
m_joyJogFlag = -1

jog_target = [0, 0, 0]
pre_jog_target = [0,0,0]

m_timer = STOP_WATCH_TIME

jog_multi = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/jog_multi', JogMultiAxis, queue_size=1)
#g_sock = client_socket_open("192.168.1.51", 10004)
print("stop_watch server connect O.K!")

def shutdown():
    #global fd
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"
    #client_socket_write(g_sock, b'#STOP')
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1
    global FLAG_X_LIMIT
    global FLAG_Y_LIMIT
    global FLAG_Z_LIMIT
    global OBJECT_DETECT
    global G_JOG_VELOCITY
    global G_DIO2

    x_cor = msg.current_posx[0]
    y_cor = msg.current_posx[1]
    z_cor = msg.current_posx[2] + (0.25 * msg.current_velx[2])
    G_JOG_VELOCITY  = 100
    #G_DIO2 = msg.ctrlbox_digital_input[1]
    if msg.ctrlbox_digital_input[1] == True:
        #rospy.loginfo("msg callback G_DIO2 : is True")
        G_DIO2 = True
    if z_cor > Z_LIMIT[0] and z_cor < Z_LIMIT[0] + 20:
        G_JOG_VELOCITY = 60
        rospy.loginfo("current Velocity : " + str(G_JOG_VELOCITY))

    if z_cor <= Z_LIMIT[0] :
        FLAG_Z_LIMIT = 2
        #print("LOW_Z_LIMIT")
    elif z_cor > Z_LIMIT[1]:
        FLAG_Z_LIMIT = 1
        #print("UPPER_Z_LIMIT")
    else:
        FLAG_Z_LIMIT = 0
    
    if y_cor < Y_LIMIT[0]:
        FLAG_Y_LIMIT = 2
        #print("LOW_Y_LIMIT")
    elif y_cor > Y_LIMIT[1]:
        FLAG_Y_LIMIT = 1
        #print("UPPER_Y_LIMIT")
    else:
        FLAG_Y_LIMIT = 0

    if x_cor < X_LIMIT[0]:
        FLAG_X_LIMIT = 2
        #print("LOW_X_LIMIT")
    elif x_cor > X_LIMIT[1]:
        FLAG_X_LIMIT = 1
        #print("UPPER_X_LIMIT")
    else:
        FLAG_X_LIMIT = 0
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
'''
def thread_stop_watch():
    global FLAG_READY_COMPLETE
    global GAME_END
    global m_timer
    print("thread_stop_watch running...")
    
    while not rospy.is_shutdown():     
        res, rx_data = client_socket_read(g_sock) #server로부터 수신 대기

        print("res={0}, rx_data ={1}".format(res, rx_data)) 

        rev_str = str(rx_data).encode("utf-8")

        if rev_str == "#STOP":
            print("GAME_STOP")
	else: 
            print("unknown data!!!")
'''
def thread_gripper():
    global GRIPPER_IS_OPEN
    global G_GRIPPER_STATUS
    global SKIP_JOG
    global G_GRIP_MOVE
    moveup_flag = False
    print("thread-gripper is started")
    ACT_FEED = 0b00
    while not rospy.is_shutdown():
        if SKIP_JOG == True:
            pass
        else:
            if G_GRIPPER_STATUS and GRIPPER_IS_OPEN:
                set_modbus_output("WID", 255)
                GRIPPER_IS_OPEN = False
                G_GRIPPER_STATUS= False
            elif G_GRIPPER_STATUS and not GRIPPER_IS_OPEN:
                set_modbus_output("WID", 122)
                GRIPPER_IS_OPEN = True
                G_GRIPPER_STATUS= False
        rospy.sleep(0.1)
        #if get_modbus_input("ACT_FEED") == 47360:
        #    if moveup_flag == False:
         #       SKIP_JOG = True
           #     movel([0, 0, G_GRIP_MOVE, 0, 0, 0], vel = 60, acc = 30, mod=DR_MV_MOD_REL)
          #      moveup_flag = True
          #      SKIP_JOG = False
        #else:
         #   moveup_flag = False

def thread_timer():
    global FLAG_READY_COMPLETE
    global FLAG_CONTROL
    #global G_POWER_OFF_LIST
    global G_POWER_SLEEP_TIME
    #global POWER_ON_LIST
    global m_timer
    global GAME_END
    global G_SENSOR_FLAG
    global G_POWER_ON_TIME
    global G_POWER_OFF_TIME
    global FLAG_POWER
    global G_DIO2
    nAt = 0
    while not rospy.is_shutdown():
        if m_timer != 0 and FLAG_READY_COMPLETE:
            print(str(m_timer))
            if G_DIO2 == True and not G_SENSOR_FLAG:
                G_SENSOR_FLAG = True
                G_POWER_OFF_TIME = m_timer - random.randrange(5, 8)
                rospy.loginfo("G_POWER_OFF_TIME : " + str(G_POWER_OFF_TIME)) 
                G_POWER_ON_TIME = G_POWER_OFF_TIME - 10
                rospy.loginfo("G_POWER_ON_TIME : " + str(G_POWER_ON_TIME)) 
            
            elif m_timer == G_POWER_OFF_TIME and FLAG_POWER:
                rospy.loginfo("power off")
                set_digital_output(5, OFF)
                FLAG_POWER = False

            elif m_timer == G_POWER_ON_TIME and not FLAG_POWER:
                rospy.loginfo("power on")
                set_digital_output(5, ON)
                FLAG_POWER = True   

            elif m_timer == G_POWER_ON_TIME - 2 and FLAG_POWER:
                G_SENSOR_FLAG = False       
                G_DIO2 = False         
                
            #if m_timer == STOP_WATCH_TIME:
                #rospy.loginfo("next sleep sec :  " + str(STOP_WATCH_TIME - G_POWER_OFF_LIST[nAt]))
                #print("power on1")
                #set_digital_output(5, ON)
                #FLAG_POWER = True
            #elif (m_timer in G_POWER_OFF_LIST) and FLAG_POWER == True:#(m_timer == STOP_WATCH_TIME - G_POWER_OFF_LIST[nAt])  and FLAG_POWER == True:
                #print("power off")
                #rospy.loginfo("G_POWER_SLEEP_TIME : " + str(G_POWER_SLEEP_TIME))
                #rospy.loginfo("next powerontime sec :  " + str(STOP_WATCH_TIME-(G_POWER_OFF_LIST[nAt] + G_POWER_SLEEP_TIME)))
                #set_digital_output(5, OFF)
                #FLAG_POWER = False
            #elif (m_timer in G_POWER_ON_LIST )and G_POWER_ON_LIST and FLAG_POWER == False:#m_timer == STOP_WATCH_TIME-(G_POWER_OFF_LIST[nAt] + G_POWER_SLEEP_TIME) and FLAG_POWER == False :
                #rospy.loginfo("next sleep sec :  " + str(STOP_WATCH_TIME - (G_POWER_OFF_LIST[nAt])))
                #print("power on2")
                #nAt = nAt+1
                #if nAt > 2:
                #    nAt = 0
                #set_digital_output(5, ON)
                #FLAG_POWER = True
            m_timer = m_timer - 1

        elif m_timer == 0:
            GAME_END = True
            
            
        rospy.sleep(1)
    rospy.loginfo("thread timer is killed")
def joy_cb(msg):
    global m_joyAnalogFlag
    global m_joyButtonFlag
    global m_joyJogFlag
    global G_JOG_VELOCITY
    global FLAG_CONTROL
    global FLAG_Z_LIMIT
    global FLAG_READY_COMPLETE
    global GRIPPER_IS_OPEN
    global jog_target
    global pre_jog_target 
    global CURRENT_STATE
    global G_GRIPPER_STATUS
    global G_GRIPPER_PRE_STATUS
    global G_DIO2
    global FLAG_POWER
    global G_SENSOR_FLAG

    button_target = [0,0,0]

    if msg.axes[4] != 0 or msg.axes[0] != 0 or msg.axes[1] != 0:
        m_joyAnalogFlag = True
    else:
        m_joyAnalogFlag = False

    if msg.axes[6] != 0 or msg.axes[7] != 0:
        m_joyButtonFlag = True
    else:
        m_joyButtonFlag = False
    
    
    if G_GRIPPER_PRE_STATUS != msg.buttons[1]:
        G_GRIPPER_PRE_STATUS = msg.buttons[1]
        if msg.buttons[1] == 1:
            G_GRIPPER_STATUS = True
    #if msg.buttons[1] == 1:
        #rospy.loginfo("G_GRIPPER_PRE_STATUS: " + str(G_GRIPPER_PRE_STATUS))
       # G_GRIPPER_STATUS = True
    elif msg.buttons[8] == 1:
        set_digital_output(5, OFF)
        rospy.loginfo("set_digital_output ON")
        FLAG_POWER = False
    elif msg.buttons[9] == 1:
        set_digital_output(5, ON) 
        G_SENSOR_FLAG = False       
        G_DIO2 = False
        FLAG_POWER = True
        rospy.loginfo("set_digital_output OFF")
    elif msg.buttons[6] == 1 and msg.buttons[7] == 1 and msg.buttons[10] == 1: # GAME END
        rospy.loginfo("FLAG_END!!!!")
        FLAG_CONTROL = FLAG_END
    elif msg.buttons[11] == 1 and msg.buttons[12] == 1:
        FLAG_CONTROL = FLAG_DANCE
    if sum(msg.buttons) > 0:    # 버튼이 눌렸을 경우
        #rospy.loginfo("button is pushed")
        if CURRENT_STATE == STATE_STAY or CURRENT_STATE == STATE_READY:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:
                FLAG_CONTROL = FLAG_READY
           # elif msg.buttons[10] == 1:
              #  FLAG_CONTROL = FLAG_HOMMING
    elif FLAG_READY_COMPLETE:
        
        if m_joyAnalogFlag and not m_joyButtonFlag:          
            button_target[0] = round(msg.axes[1], 1) # : X                                         
            button_target[1] = round(msg.axes[0], 1) # : Y                                     
            button_target[2] = round(msg.axes[4], 1) # : Z   
            
            if FLAG_Z_LIMIT == 2 and button_target[2] <= 0:
                button_target[2] = 0    
            elif FLAG_Z_LIMIT == 1 and button_target[2] > 0:
                button_target[2] = 0

            if FLAG_Y_LIMIT == 2 and button_target[1] <= 0:
                button_target[1] = 0    
            elif FLAG_Y_LIMIT == 1 and button_target[1] > 0:
                button_target[1] = 0

            if FLAG_X_LIMIT == 2 and button_target[0] <= 0:
                button_target[0] = 0    
            elif FLAG_X_LIMIT == 1 and button_target[0] > 0:
                button_target[0] = 0
            for i in range(0,3):
                jog_target[i] = button_target[i]
            FLAG_CONTROL = FLAG_MULTI_JOG_START
        elif not m_joyAnalogFlag and not m_joyButtonFlag:
            FLAG_CONTROL = FLAG_JOG_STOP
            

def gripper_activation():
    add_modbus_signal("192.168.1.11", 502, "ACT", DR_MODBUS_REG_OUTPUT, 0, 0)
    add_modbus_signal("192.168.1.11", 502, "WID", DR_MODBUS_REG_OUTPUT, 1, 0)
    add_modbus_signal("192.168.1.11", 502, "FOR", DR_MODBUS_REG_OUTPUT, 2, 0)
    add_modbus_signal("192.168.1.11", 502, "ACT_FEED", DR_MODBUS_REG_INPUT, 0, 0)
    add_modbus_signal("192.168.1.11", 502, "WID_FEED", DR_MODBUS_REG_INPUT, 1, 0)

    set_modbus_output("ACT", 2304)
    set_modbus_output("WID", 122)
    set_modbus_output("FOR", G_GRIPPER_FORCE * G_GRIPPER_SPEED)

def display_change_ready():
   # client_socket_write(g_sock, b'#BUTTON')  
    pass

def move_ready():
    global CURRENT_STATE
    global G_MOVE_READY_ACC
    global G_MOVE_READY_VEL
    CURRENT_STATE = STATE_MOVING
    rospy.loginfo("move_ready")
    #current_z = get_current_pose(ROBOT_SPACE_TASK)[2]
    #task_diff = READY_POS_TASK[2] - current_z
    #if task_diff > 30:
     #   movej([0, 0, task_diff, 0, 0, 0], vel = 60, acc = 30, mod=DR_MV_MOD_REL)
    movej(READY_POS, vel = G_MOVE_READY_VEL, acc = G_MOVE_READY_ACC)
    CURRENT_STATE = STATE_STAY

def dance_hold():
    global g_dance_stop
    while g_dance_stop:
        wait(0.1)
    return 

def dance():
    global g_dance_stop
    rospy.loginfo("dancing........")
    rospy.loginfo("running start........")
    m1 = posj(-14.25, 28.55, -101.94, 18.59, -42, -38.91) # motion 1
    m2 = posj(-44.64, 19.49, -115.75, 109.52, -67.71, -22.15) # motion 2
    m3 = posj(26.69, 22.05, -117.58, 71.65, 63.1, -22.32) # motion 5
    m4 = posj(-6.19, 31.84, -100.24, 169.54, 52.37, -46.12) # motion 6
    m5 = posj(-3.75, 28.18, -119.87, 9.03, -35.13, -46.06)

    t1 = posj(-4.83, -1.15, -101.08, -0.24, -77.53, -4.76)
    t2 = posj(-4.82, -3.14, -107.23, -0.29, -69.27, -4.73)
    t3 = posj(-4.83, -1.15, -101.08, -0.24, -77.53, -4.76)
    t4 = posj(-4.82, -3.14, -107.23, -0.29, -69.27, -4.73) 
    t5 = posj(-37.63, 8.5, -108.25, -0.08, -79.4, -37.53)
    t6 = posj(-37.34, 5.3, -116.99, -0.1, -67.25, -37.28)
    t7 = posj(-28.65, 12.24, -109.55, -0.23, -81.27, -28.76)
    t8 = posj(-3.39, 15.8, -112.74, -0.79, -81.73, -3.42)
    t9 = posj(-3.4, 14.88, -117.4, -0.83, -76.07, -3.42)

    #j1 = posj(-15.72, -2.86, -95.95, 7.64, -63.58, -12.67) # motion3
    #j2 = posj(-18.07, 2.44, -82.34, 7.51, -82.63, -12.42) #motion 4

    c1 =posj(-82.95, 9.59, -119.53, -0.07, -69.45, -95.17)
    c2 =posj(-45.83, -4.11, -105.04, -0.34, -71.21, -51.52)
    c3 =posj(-16.55, -4.75, -105.12, -0.29, -68.53, -16.14)
    c4 =posj(11.87, -5.93, -104.64, -0.01, -71.48, 11.44)
    c5 =posj(39.67, -7.81, -104.62, -0.34, -69.24, 40.92)
    c6 =posj(58.23, 0.97, -113.37, -0.87, -65.62, 59.54)

    while True:
        ready = posj(0, 21, -113, 0, -88, 0)
        
        dance_hold()
        set_modbus_output("WID",0)
        dance_hold()
        movej(ready,t=2)
        dance_hold()
        set_modbus_output("WID",255)
        dance_hold()
        movej(m1,t=2)
        dance_hold()
        set_modbus_output("WID",0)
        dance_hold()
        movej(m2,t=2)
        dance_hold()
        set_modbus_output("WID",255)
        dance_hold()
        movej(m3,t=2)
        dance_hold()
        set_modbus_output("WID",0)
        dance_hold()
        movej(m4,t=3)
        dance_hold()
        set_modbus_output("WID",255)
        dance_hold()
        movej(m5,t=4)
        dance_hold()
        movej(t1,t=2)
        dance_hold()
        movej(t2,t=1)
        dance_hold()
        movej(t3,t=1)
        dance_hold()
        movej(t5,t=2)
        dance_hold()
        movej(t6,t=2)
        dance_hold()
        basket = [t7,t8,t9]
        movesj(basket,t=3)
        #movej(t7,t=2)
        #movej(t8,t=2)
        #movej(t9,t=2)
        rospy.loginfo("running end........")

        
        curve = [c1,c2,c3,c4,c5,c6]
        dance_hold()
        set_modbus_output("WID",255)
        dance_hold()
        movesj(curve,t=3)
        dance_hold()
        movej(ready,t=2)
        dance_hold()
        set_modbus_output("WID",0)
    ###대기자세

def thread_publish_jog():
    global jog_multi
    global jog_target
    global pre_jog_target
    global FLAG_CONTROL
    global CURRENT_STATE
    global SKIP_JOG
    while not rospy.is_shutdown():
        if not SKIP_JOG: 
            if (jog_target) != pre_jog_target and (FLAG_CONTROL == FLAG_MULTI_JOG_START or FLAG_CONTROL == FLAG_JOG_STOP) and (CURRENT_STATE != STATE_MOVING):
                if FLAG_Z_LIMIT == 2 and jog_target[2] <= 0:
                    jog_target[2] = 0    
                elif FLAG_Z_LIMIT == 1 and jog_target[2] > 0:
                    jog_target[2] = 0

                if FLAG_Y_LIMIT == 2 and jog_target[1] <= 0:
                    jog_target[1] = 0    
                elif FLAG_Y_LIMIT == 1 and jog_target[1] > 0:
                    jog_target[1] = 0

                if FLAG_X_LIMIT == 2 and jog_target[0] <= 0:
                    jog_target[0] = 0    
                elif FLAG_X_LIMIT == 1 and jog_target[0] > 0:
                    jog_target[0] = 0
                l_jog_target = [jog_target[0], jog_target[1], jog_target[2], 0, 0, 0]
                if FLAG_Z_LIMIT != 0:
                    print("l_jog_target {0}, G_JOG_VELOCITY {1}".format(jog_target, G_JOG_VELOCITY))
                jog_multi.publish(l_jog_target, MOVE_REFERENCE_BASE, G_JOG_VELOCITY)
        else:
            rospy.loginfo("SKIP_JOG")
        pre_jog_target[0] = jog_target[0]
        pre_jog_target[1] = jog_target[1]
        pre_jog_target[2] = jog_target[2]
        rospy.sleep(0.1)
    jog_multi.publish([0,0,0,0,0,0], MOVE_REFERENCE_BASE, 0)

def get_state_string(nState):
    if nState == -1:
        return "STATE_START"
    elif nState == 0:
        return "STATE_JOG"
    elif nState == 1:    
        return "STATE_GRIPPING"    
    elif nState == 2:  
        return "STATE_MOVING"
    elif nState == 3:    
        return "STATE_READY"   
    elif nState == 4:  
        return "STATE_STAY"
    elif nState == 5:
        return "STATE_DANCE"
    else:
        return "STATE_UNKNOWN"
if __name__ == "__main__":
    rospy.init_node('joy_xbox360_py')
    rospy.on_shutdown(shutdown)
    CURRENT_STATE = STATE_READY
    
    set_robot_mode(ROBOT_MODE_MANUAL)

    t2 = threading.Thread(target=thread_subscriber)
    t2.daemon = True 
    t2.start()

    t3 = threading.Thread(target=thread_publish_jog)
    t3.daemon = True 
    t3.start()

    #t4 = threading.Thread(target=thread_timer)
    #t4.daemon = True 
    #t4.start()

    #t5 = threading.Thread(target=thread_gripper)
    #t5.daemon = True 
    #t5.start()
    
    #t6 = threading.Thread(target=dance)
    #t6.daemon = True
    #t6.start()
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    #gripper_activation()
    #display_change_ready()
    sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)
    jog_stop_cnt = 0
    ACT = 0
    while not rospy.is_shutdown():
        if FLAG_READY_COMPLETE:
            #rospy.loginfo("get_digital_input(2) : " + str(G_DIO2))
            if FLAG_Z_LIMIT == 2 and jog_target[2] <= 0:
                jog_target[2] = 0    
            elif FLAG_Z_LIMIT == 1 and jog_target[2] > 0:
                jog_target[2] = 0

            if FLAG_Y_LIMIT == 2 and jog_target[1] <= 0:
                jog_target[1] = 0    
            elif FLAG_Y_LIMIT == 1 and jog_target[1] > 0:
                jog_target[1] = 0

            if FLAG_X_LIMIT == 2 and jog_target[0] <= 0:
                jog_target[0] = 0    
            elif FLAG_X_LIMIT == 1 and jog_target[0] > 0:
                jog_target[0] = 0
            
            if FLAG_CONTROL == FLAG_JOG_STOP:
                jog_multi.publish([0,0,0,0,0,0], MOVE_REFERENCE_BASE, 0)
                G_JOG_VELOCITY = 0
                FLAG_CONTROL = -1
            elif (FLAG_CONTROL == FLAG_MULTI_JOG_START) and (CURRENT_STATE == STATE_STAY or CURRENT_STATE == STATE_JOG) and not GAME_END :
                CURRENT_STATE = STATE_JOG   
                #G_JOG_VELOCITY = 100

            if FLAG_CONTROL == FLAG_END or GAME_END:
                print("end!!!!!!!!1")
                jog_multi.publish([0,0,0,0,0,0], MOVE_REFERENCE_BASE, 0)
                
                FLAG_READY_COMPLETE = False
                #client_socket_write(g_sock,b'#STOP')
                display_change_ready()  
                set_digital_output(5, OFF)
                SKIP_JOG = True
                G_SENSOR_FLAG = False
                FLAG_POWER = False
                G_POWER_ON_TIME = 0
                G_POWER_OFF_TIME = 0
                G_DIO2 = False
                wait(2)
                move_ready()
                m_timer = STOP_WATCH_TIME
                SKIP_JOG = False
                set_modbus_output("WID", 122)
                FLAG_CONTROL = -1
                CURRENT_STATE = STATE_READY
                GAME_END = False
        else:    
            
            if FLAG_CONTROL == FLAG_READY and (CURRENT_STATE == STATE_READY or CURRENT_STATE == STATE_STAY or STATE_DANCE):
                move_ready()
                print("start!!!!!!!!!!!!!!!!")
                #client_socket_write(g_sock, b'#START')
                FLAG_READY_COMPLETE = True
                set_digital_output(5, ON)
                FLAG_POWER = True
                FLAG_CONTROL = -1            
            #elif FLAG_CONTROL == FLAG_HOMMING:
             #   movej(HOMMING_POS, 10, 10)
              #  display_change_ready() 
              #  FLAG_CONTROL = -1
            elif FLAG_CONTROL == FLAG_DANCE and (CURRENT_STATE == STATE_READY or CURRENT_STATE == STATE_STAY):
                if CURRENT_STATE != STATE_DANCE:
                    CURRENT_STATE = STATE_DANCE
                    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                    g_dance_stop = False
                    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!dance start")
                FLAG_CONTROL = -1
            elif FLAG_CONTROL == FLAG_END and (CURRENT_STATE == STATE_DANCE):
                g_dance_stop = True
                pub_stop.publish(stop_mode=STOP_TYPE_SLOW)
                #wait(2)
                rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!dance stop")
                wait(1)
                set_robot_mode(ROBOT_MODE_MANUAL)
                wait(1)
                move_ready()
                wait(0.5)
                set_modbus_output("WID", 122)
                CURERNT_STATE = STATE_READY
                FLAG_CONTROL = -1
            else: 
                pass
            if PRE_STATE != CURRENT_STATE:
                print("PRE_STATE {0} -> {1}".format(get_state_string(PRE_STATE), get_state_string(CURRENT_STATE)))
            PRE_STATE = CURRENT_STATE

    #client_socket_write(g_sock, b'#STOP')
    wait(0.4)
    #client_socket_close(g_sock)
    print 'good bye!'
