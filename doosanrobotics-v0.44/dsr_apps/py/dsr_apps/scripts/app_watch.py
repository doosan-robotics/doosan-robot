#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import sys

from dsr_msgs.msg import *

DSR_CTL_PUB_RATE  = 100  #[hz] , 10ms     
DEFAULT_RATE = 0.1  #[sec]  
MINIMUM_RATE = 0.1 #[sec]  

g_dRate = DEFAULT_RATE
g_nCnt  = 0

class CMonitor(object):
    def __init__(self, robot_name, drate):
        # Params
        self.robot_name = robot_name
        self.dRate = drate
        self.nCnt  = 0

        # Node cycle rate (in Hz).
        #elf.loop_rate = rospy.Rate(1)

        # Subscribers
        rospy.Subscriber(self.robot_name +'/state', RobotState, self.msgRobotStateCallback)
        rospy.Subscriber(self.robot_name +'/error', RobotError, self.msgRobotErrorCallback)

    def switch_level(self,x):
        return {1:"INFO", 2:"WARN", 3:"ERROR"}.get(x,"NONE")    
    def switch_group(self,x):
        return {1:"SYSTEM", 2:"MOTION", 3:"INVERTER", 4:"SAFETY_CONTROLLER"}.get(x,"NONE")    

    def msgRobotStateCallback(self, msg):
        # This function is called every 10 msec
        self.nCnt += 1
        if ( 0==(self.nCnt % (self.dRate * DSR_CTL_PUB_RATE)) ): 
            self.nCnt = 0        
            os.system('clear')
            rospy.loginfo("________ ROBOT(%s) STATUS ________" % self.robot_name)
            #rospy.loginfo("  robot_state       : %d" % (msg.robot_state))
            rospy.loginfo("  robot_state       : %s" % msg.robot_state_str)
            rospy.loginfo("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
            #rospy.loginfo("  current_posx      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))

            rospy.loginfo("  io_control_box    : %d" % msg.io_control_box)
            #rospy.loginfo("  io_modbus         : %d" % msg.io_modbus)
            #rospy.loginfo("  error             : %d" % msg.error)
            rospy.loginfo("  access_control    : %d" % msg.access_control)
            rospy.loginfo("  homming_completed : %d" % msg.homming_completed)
            rospy.loginfo("  tp_initialized    : %d" % msg.tp_initialized)
            rospy.loginfo("  speed             : %d" % msg.speed)
            rospy.loginfo("  mastering_need    : %d" % msg.mastering_need)
            rospy.loginfo("  drl_stopped       : %d" % msg.drl_stopped)
            rospy.loginfo("  disconnected      : %d" % msg.disconnected)

    def msgRobotErrorCallback(self, msg):
        # This function is called when an error occurs.     
        rospy.loginfo("________ ROBOT ERROR ________")
        rospy.loginfo("  level       : %s" % self.switch_level(1) )
        rospy.loginfo("  group       : %s" % self.switch_group(2) )
        rospy.loginfo("  code        : %d" % msg.code)
        if msg.msg1 != '':
            rospy.loginfo("  msg1        : %s" % msg.msg1)
        if msg.msg2 != '':
            rospy.loginfo("  msg2        : %s" % msg.msg2)
        if msg.msg3 != '':
            rospy.loginfo("  msg3        : %s" % msg.msg3)

        rospy.signal_shutdown("stopped by error of robot")
 

if __name__ == '__main__':

    MsgNamePrefix = "/dsr01m1013"
    dRate = DEFAULT_RATE #sec   

    if len(sys.argv)==3:
        MsgNamePrefix = sys.argv[1];

        dRate = float(sys.argv[2])
        if(dRate <= MINIMUM_RATE):
            dRate = MINIMUM_RATE  
    else:  
        print("[ERROR] invalid argments: <robot_id> <duraion>")
        print("<ex> rosrun dsr_app_py app_watch.py /dsr01m1013 1.0")
        sys.exit(-1)


    rospy.init_node('app_watch_py', anonymous=True)

    rospy.loginfo("________ Monitor every [%.3f]sec ________" % dRate);
    rospy.sleep(0.5) #[sec]

    monitor_node = CMonitor(MsgNamePrefix, dRate)

    rospy.spin()

    rospy.loginfo("app_watch_py finished !!!")


