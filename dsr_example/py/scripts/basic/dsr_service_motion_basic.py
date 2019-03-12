#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example basic] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
from std_msgs.msg import String,Int32,Int32MultiArray,Float64,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import sys
sys.dont_write_bytecode = True
#sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 
#from DSR_ROBOT import *

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DRFC.py 
from DRFC import *

from dsr_msgs.msg import *
from dsr_msgs.srv import *


#--------------------------


#--------------------------

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   



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

def  movej(fTargetPos, fTargetVel, fTargetAcc, fTargetTime = 0.0, fBlendingRadius = 0.0, 
          nMoveMode = MOVE_MODE_ABSOLUTE, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSyncType = 0):
    return srv_move_joint(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveMode, nBlendingType, 0)
def amovej(fTargetPos, fTargetVel, fTargetAcc, fTargetTime = 0.0, fBlendingRadius = 0.0,
          nMoveMode = MOVE_MODE_ABSOLUTE, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSyncType = 1):
    return srv_move_joint(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveMode, nBlendingType, 1)


def  movel(fTargetPos, fTargetVel, fTargetAcc, fTargetTime = 0.0, fBlendingRadius = 0.0,
           nMoveReference = MOVE_REFERENCE_BASE, nMoveMode = MOVE_MODE_ABSOLUTE, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSyncType = 0):
    return srv_move_line(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, 0)
def amovel(fTargetPos, fTargetVel, fTargetAcc, fTargetTime = 0.0, fBlendingRadius = 0.0,
           nMoveReference = MOVE_REFERENCE_BASE, nMoveMode = MOVE_MODE_ABSOLUTE, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSyncType = 1):
    return srv_move_line(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, 1)


def  movejx(fTargetPos, fTargetVel, fTargetAcc, fTargetTime=0.0,
           fBlendingRadius = 0.0, nMoveReference = MOVE_REFERENCE_BASE, nMoveMode = MOVE_MODE_ABSOLUTE, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSolSpace = 0, nSyncType = 0):
    return srv_move_jointx(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, nSolSpace, 0)
def amovejx(fTargetPos, fTargetVel, fTargetAcc, fTargetTime=0.0,
           fBlendingRadius = 0.0, nMoveReference = MOVE_REFERENCE_BASE, nMoveMode = MOVE_MODE_ABSOLUTE, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSolSpace = 0, nSyncType = 0):
    return srv_move_jointx(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, nSolSpace, 1)


def  movec(fTargetPos, fTargetVel, fTargetAcc, fTargetTime = 0.0, fBlendingRadius = 0.0, 
           nMoveReference = MOVE_REFERENCE_BASE, nMoveMode = MOVE_MODE_ABSOLUTE, fAngle1 = 0.0, fAngle2 = 0.0, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSyncType = 0):

    print("XXXXXXXXXXXXXX fTargetPos[0] = {0}".format(fTargetPos[0]))
    print("XXXXXXXXXXXXXX fTargetPos[1] = {0}".format(fTargetPos[1]))

    #_circle_pos = _ros_listToFloat64MultiArray([fTargetPos[0], fTargetPos[1]])
    _circle_pos = _ros_listToFloat64MultiArray(fTargetPos)

    print("XXXXXXXXXXXXXX _circle_pos = {0}".format(_circle_pos))

    return srv_move_circle(_circle_pos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, fAngle1, fAngle2, nBlendingType, 0)
def amovec(fTargetPos, fTargetVel, fTargetAcc, fTargetTime = 0.0, fBlendingRadius = 0.0,
           nMoveReference = MOVE_REFERENCE_BASE, nMoveMode = MOVE_MODE_ABSOLUTE, fAngle1 = 0.0, fAngle2 = 0.0, nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, nSyncType = 0):
    #_circle_pos = _ros_listToFloat64MultiArray([fTargetPos[0], fTargetPos[1]])
    _circle_pos = _ros_listToFloat64MultiArray(fTargetPos)
    return srv_move_circle(_circle_pos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, fAngle1, fAngle2, nBlendingType, 1)


def  movesj(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime = 0.0,
            nMoveMode = MOVE_MODE_ABSOLUTE, nSyncType = 0):

    _spline_joint_pos = _ros_listToFloat64MultiArray(fTargetPos)

    print("XXXXXXXXXXXXXX _spline_joint_pos = {0}".format(_spline_joint_pos) )
    print("XXXXXXXXXXXXXX len(fTargetPos) = {0}".format(len(fTargetPos)) )
    print("XXXXXXXXXXXXXX len(_spline_joint_pos) = {0}".format(len(_spline_joint_pos)) )

    return srv_move_spline_joint(_spline_joint_pos, len(_spline_joint_pos), fTargetVel, fTargetAcc, fTargetTime, nMoveMode, 0)
def amovesj(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime = 0.0, 
            nMoveMode = MOVE_MODE_ABSOLUTE, nSyncType = 0):
    _spline_joint_pos = _ros_listToFloat64MultiArray(fTargetPos)
    return srv_move_spline_joint(_spline_joint_pos, len(_spline_joint_pos), fTargetVel, fTargetAcc, fTargetTime, nMoveMode, 1)


def  movesx(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime = 0.0,
            nMoveMode = MOVE_MODE_ABSOLUTE, nMoveReference = MOVE_REFERENCE_BASE, nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT, nSyncType = 0):
    _spline_task_pos = _ros_listToFloat64MultiArray(fTargetPos)
    return srv_move_spline_task(_spline_task_pos, len(_spline_task_pos), fTargetVel, fTargetAcc, fTargetTime, nMoveMode, nMoveReference, nVelOpt, 0)
def amovesx(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime = 0.0,
            nMoveMode = MOVE_MODE_ABSOLUTE, nMoveReference = MOVE_REFERENCE_BASE, nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT, nSyncType = 0):
    _spline_task_pos = _ros_listToFloat64MultiArray(fTargetPos)
    return srv_move_spline_task(_spline_task_pos, len(_spline_task_pos), fTargetVel, fTargetAcc, fTargetTime, nMoveMode, nMoveReference, nVelOpt, 1)


def  moveb(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime = 0.0,
           nMoveMode = MOVE_MODE_ABSOLUTE, nMoveReference = MOVE_REFERENCE_BASE, nSyncType = 0):

    _moveb_pos = _ros_listToFloat64MultiArray(fTargetPos)

    print("XXXXXXXXXXXXXX _moveb_pos = {0}".format(_moveb_pos) )
    print("XXXXXXXXXXXXXX len(fTargetPos) = {0}".format(len(_moveb_pos)) )
    return srv_move_blending(_moveb_pos, len(_moveb_pos), fTargetVel, fTargetAcc, fTargetTime, nMoveMode, nMoveReference, 0)
def amoveb(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime = 0.0,
           nMoveMode = MOVE_MODE_ABSOLUTE, nMoveReference = MOVE_REFERENCE_BASE, nSyncType = 0):
    _moveb_pos = _ros_listToFloat64MultiArray(fTargetPos)
    return srv_move_blending(_moveb_pos, len(_moveb_pos), fTargetVel, fTargetAcc, fTargetTime, nMoveMode, nMoveReference, 1)


def  move_spiral(nTaskAxis, fRevolution, fMaxRadius, fMaxLength, fTargetVel, fTargetAcc, fTargetTime = 0.0,
                 nMoveReference = MOVE_REFERENCE_TOOL, nSyncType = 0):
    return srv_move_spiral(nTaskAxis, fRevolution, fMaxRadius, fMaxLength, fTargetVel, fTargetAcc, fTargetTime, nMoveReference, 0)
def amove_spiral(nTaskAxis, fRevolution, fMaxRadius, fMaxLength, fTargetVel, fTargetAcc, fTargetTime = 0.0,
                 nMoveReference = MOVE_REFERENCE_TOOL, nSyncType = 0):
    return srv_move_spiral(nTaskAxis, fRevolution, fMaxRadius, fMaxLength, fTargetVel, fTargetAcc, fTargetTime, nMoveReference, 1)


def  move_periodic(fAmplitude, fPeriodic, fAccelTime = 0.0,
                   nRepeat = 1, nMoveReference = MOVE_REFERENCE_TOOL, nSyncType = 0):
    return srv_move_periodic(fAmplitude, fPeriodic, fAccelTime, nRepeat, nMoveReference, 0)
def amove_periodic(fAmplitude, fPeriodic, fAccelTime = 0.0,
                   nRepeat = 1, nMoveReference = MOVE_REFERENCE_TOOL, nSyncType = 0):
    return srv_move_periodic(fAmplitude, fPeriodic, fAccelTime, nRepeat, nMoveReference, 1)


def move_wait():
    return srv_move_wait()
####################################################################################################################################################

def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

    pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
    return 0
##################################################################################################    
if __name__ == "__main__":
    #----- set target robot --------------- 
    my_robot_id    = "dsr01"
    my_robot_model = "m1013"
    SET_ROBOT(my_robot_id, my_robot_model)

    rospy.init_node('dsr_service_motion_basic_py')
    rospy.on_shutdown(shutdown)


    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    
    #print 'wait services'
    rospy.wait_for_service('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_joint')

    srv_move_joint  = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_joint', MoveJoint)
    srv_move_jointx = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_jointx', MoveJointx)

    srv_move_line   = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_line', MoveLine)
    srv_move_circle = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_circle', MoveCircle)
    srv_move_spline_joint = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_spline_joint', MoveSplineJoint)   
    srv_move_spline_task = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_spline_task', MoveSplineTask)

    srv_move_blending = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_blending', MoveBlending)
    srv_move_spiral = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_spiral', MoveSpiral)
    srv_move_periodic = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_periodic', MovePeriodic)
    srv_move_wait = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/motion/move_wait', MoveWait)


    velx =[250.0, 80.625]   # 태스크 속도를 250(mm/sec), 80.625(deg/sec)로 설정
    accx =[1000.0, 322.5]   # 태스크 가속도를 1000(mm/sec2), 322.5(deg/sec2)로 설정

    j1 = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]   #joint
   
    sj1 =[[10.00, 0.00, 0.00, 0.00, 10.00, 20.00], [15.00, 0.00, -10.00, 0.00, 10.00, 20.00]]

    x1 = [0.0, 0.0, -100.0, 0.0, 0.0, 0.0]   #task
    x2 = [545,100,514,0,-180,0]  #jx task
    cx1 = [[544.00, 100.00, 500.00, 0.00, -180.00, 0.00], [543.00, 106.00, 479.00, 7.00, -180.00, 7.00]]
    sx1 = [[10.00, -10.00, 20.00, 0.00, 10.00, 0.00], [15.00, 10.00, -10.00, 0.00, 10.00, 0.00]]

    bx1 = [564.00, 200.00, 690.00, 0.00, 180.00, 0.00] + [0, 0, 0, 0, 0, 0]
    bx_type1= [0]; bx_radius1=[40.0]

    bx2 = [564.00, 100.00, 590.00, 0.00, 180.00, 0.00] + [564.00, 150.00, 590.00, 0.00, 180.00, 0.00]
    bx_type2= [1]; bx_radius2=[20.0]

    amp = [10.00, 0.00, 20.00, 0.00, 0.50, 0.00]
    period = [1.00, 0.00, 1.50, 0.00, 0.00, 0.00]
    # ??? MOVE_POSB posb[2];
  
    while not rospy.is_shutdown():

        movej(j1, 60, 30)

        movel(x1, velx, accx, 0, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE)

        movejx(x2, 60, 30, 2, 0, MOVE_REFERENCE_BASE, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_DUPLICATE, 2)

        movec(cx1, velx, accx)

        movesj(sj1, 2, 60, 30, 0, MOVE_MODE_RELATIVE)

        movesx(sx1, 2, velx, accx, 0, MOVE_MODE_RELATIVE)

 
        bx_type1= [0]; bx_radius1=[40.0]
        bx_type2= [1]; bx_radius2=[20.0]

        seg1 = bx1 + bx_type1 + bx_radius1
        seg2 = bx2 + bx_type2 + bx_radius2
    
        posb = [seg1, seg2]
        posCnt = len(posb)

        moveb(posb, posCnt, velx, accx)

        move_spiral(1.00, 20.00, 20.00, velx, accx, 5, TASK_AXIS_Z, MOVE_REFERENCE_TOOL)

        move_periodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE)

        #---- async motions --------------------------------------------------------------------------------- 
        amovej(j1, 60, 30, 0, 0, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_DUPLICATE, 1)
        move_wait()

        amovel(x1, velx, accx, 0, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE, BLENDING_SPEED_TYPE_DUPLICATE, 1)
        move_wait()

        amovejx(x2, 60, 30, 2, MOVE_MODE_ABSOLUTE,MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 2, 1)
        move_wait()

        amovec(cx1, velx, accx, 0, 0, MOVE_REFERENCE_BASE, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_DUPLICATE, 1)
        move_wait()

        amovesj(sj1, 2, 60, 30, 0, MOVE_MODE_RELATIVE, 1)
        move_wait()

        amovesx(sx1, 2, velx, accx, 0, MOVE_MODE_RELATIVE, MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION_DEFAULT, 1)
        move_wait()

        amoveb(posb, 2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 1)
        move_wait()

        amove_spiral(1.00, 20.00, 20.00, velx, accx, 5, TASK_AXIS_Z, MOVE_REFERENCE_TOOL, 1)
        move_wait()

        amove_periodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE, 1)
        move_wait()
    

    '''
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
        move_spiral(revolution, maxRadius, maxLength, velx, accx, time, taskAxis, ref, syncType)
        move_periodic(amp, period, atime, repeat, ref, syncType)
        move_blending(mb_seg, posCnt, velx, accx, time, mode, ref, syncType)
    '''

    print 'good bye!'
    print 'good bye!'
    print 'good bye!' 


