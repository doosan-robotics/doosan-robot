#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
from std_msgs.msg import String,Int32,Int32MultiArray,Float64,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import sys
sys.dont_write_bytecode = True
#sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 
#from DSR_ROBOT import *

from dsr_msgs.msg import *
from dsr_msgs.srv import *

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   

def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

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

    rospy.init_node('dsr_basic_test_py')
    rospy.on_shutdown(shutdown)


    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    
    #print 'wait services'
    rospy.wait_for_service('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_joint')

    move_joint  = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_joint', MoveJoint)
    move_jointx = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_jointx', MoveJointx)

    move_line   = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_line', MoveLine)
    move_circle = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_circle', MoveCircle)
    move_spline_joint = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_spline_joint', MoveSplineJoint)   
    move_spline_task = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_spline_task', MoveSplineTask)

    move_blending = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_blending', MoveBlending)
    move_spiral = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_spiral', MoveSpiral)
    move_periodic = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_periodic', MovePeriodic)

    time= 0.0; mode=0; ref=0; radius=0.0; blendType=0; syncType=0
    vel=30; acc=30; sol=0
    angle1=0.0; angle2=0.0
    opt = 0

    p1=[0,0,0,0,0,0]                    #joint
    p2=[0.0, 0.0, 90.0, 0.0, 90.0, 0.0] #joint
    x1=[400, 500, 800.0, 0.0, 180.0, 0.0] #task
    x2=[400, 500, 500.0, 0.0, 180.0, 0.0] #task
    velx=[50, 50]
    accx=[100, 100]

    c1 = [559,434.5,651.5,0,180,0]
    c2 = [559,434.5,251.5,0,180,0]
    CirclePos = _ros_listToFloat64MultiArray([c1, c2])

    q0 =[0,0,0,0,0,0]
    q1 =[10, -10, 20, -30, 10, 20]
    q2 =[25, 0, 10, -50, 20, 40] 
    q3 =[50, 50, 50, 50, 50, 50] 
    q4 =[30, 10, 30, -20, 10, 60]
    q5 =[20, 20, 40, 20, 0, 90]
    SplinePosj = _ros_listToFloat64MultiArray([q0,q1,q2,q3,q4,q5])

    x1 = [600, 600, 600, 0, 175, 0] 
    x2 = [600, 750, 600, 0, 175, 0]
    x3 = [150, 600, 450, 0, 175, 0]
    x4 = [-300, 300, 300, 0, 175, 0]
    x5 = [-200, 700, 500, 0, 175, 0]
    x6 = [600, 600, 400, 0, 175, 0]
    SplinePosx = _ros_listToFloat64MultiArray([x1, x2, x3, x4, x5, x6])


    taskAxis = 0            
    revolution = 9      
    maxRadius = 20.0     
    maxLength =50.0      

    amp =[10,0,0,0,30,0]
    period=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    atime=0.2
    repeat=5


    bx11 = [370.1, 670.1, 650.1, 0.1, 180.1, 0.1]; bx12 = [370.1, 670.1, 650.1, 0.1, 180.1, 0.1]; bx_type1= [0]; bx_radius1=[21.0]
    bx21 = [370.2, 670.2, 400.2, 0.2, 180.2, 0.2]; bx22 = [370.2, 545.2, 400.2, 0.2, 180.2, 0.2]; bx_type2= [1]; bx_radius2=[22.0]
    bx31 = [370.3, 595.3, 400.3, 0.3, 180.3, 0.3]; bx32 = [370.3, 595.3, 400.3, 0.3, 180.3, 0.3]; bx_type3= [0]; bx_radius3=[23.0]
    
    seg1 = bx11 + bx12 + bx_type1 + bx_radius1
    seg2 = bx21 + bx22 + bx_type2 + bx_radius2
    seg3 = bx31 + bx32 + bx_type3 + bx_radius3
    
    mb_seg = _ros_listToFloat64MultiArray([seg1, seg2, seg3])
    posCnt = len(mb_seg)
    
    while not rospy.is_shutdown():
        move_joint(p2, vel, acc, time, mode, radius, blendType, syncType) 
        move_jointx(x1, sol, vel, acc, time, mode, ref, radius, blendType, syncType)
        move_line(x2, velx, accx, time, mode, ref, radius, blendType, syncType) 
        move_circle(CirclePos, velx, accx, time, mode, ref, angle1, angle2, radius, blendType, syncType)
        move_spline_joint(SplinePosj, len(SplinePosj), vel, acc, time, mode, syncType)
        move_spline_task(SplinePosx, len(SplinePosx), velx, accx, time, mode, ref, opt, syncType)
        move_spiral(taskAxis, revolution, maxRadius, maxLength, velx, accx, time, ref, syncType)
        move_periodic(amp, period, atime, repeat, ref, syncType)
        move_blending(mb_seg, posCnt, velx, accx, time, mode, ref, syncType)

    print 'good bye!'
    print 'good bye!'
    print 'good bye!' 


