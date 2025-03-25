#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import threading, time
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import threading, time
from kdl_parser_py.urdf import treeFromFile
import math
import sys
import pathlib
import numpy as np


sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from my_robot_arm import My_RobotArm
from my_rrt_star import My_RRTStar
from publisher_utilies import publish_end_effector_trajectory,publish_obstacles,JointStateSubscriber

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *
show_animation = False
verbose = False

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
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()



def convert_to_degrees(path):
    """
    Converte una lista di traiettorie in radianti a gradi.
    
    path: lista di configurazioni articolari in radianti
    return: lista di configurazioni articolari in gradi
    """
    return [[np.degrees(angle) for angle in q] for q in path]




if __name__ == "__main__":
    
    rospy.init_node('rrt_motion_planning')
    rospy.on_shutdown(shutdown)

    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

    # Carica la catena cinematcca dal file URDF
    urdf_file = "/home/jntlbap/catkin_ws/src/doosan-robot/dsr_description/urdf/m0609.urdf"
    chain = treeFromFile(urdf_file)[1].getChain("base_0", "link6")
    doosan=My_RobotArm(chain)
    joint_state_subscriber=JointStateSubscriber(doosan)




    goal_cartesian = [0.4, 0.4, 0.6]  # [x, y, z]
    #goal_rot = [math.pi/2, math.pi/2, math.pi/2]  # Orientamento desiderato (roll, pitch, yaw)

    start_cartesian = [-0.4, 0.4, 0.6]
    #start_rot = [-math.pi/2, -math.pi/2, math.pi/2]  # Orientamento desiderato (roll, pitch, yaw)

    # Carica la catena cinematcca dal file URDF
    urdf_file = "/home/jntlbap/catkin_ws/src/doosan-robot/dsr_description/urdf/m0609.urdf"
    chain = treeFromFile(urdf_file)[1].getChain("base_0", "link6")

        # Converti in spazio dei giunti

    start_joint_space =doosan.inverse_kinematics(start_cartesian)
    goal_joint_space =doosan.inverse_kinematics(goal_cartesian)

    if start_joint_space is None or goal_joint_space is None:
        rospy.logerr("Errore nella cinematica inversa: impossibile trovare una soluzione valida")
    else:
        rospy.loginfo(f"Configurazione iniziale in joint space: {start_joint_space}")
        rospy.loginfo(f"Configurazione finale in joint space: {goal_joint_space}")


    obstacle_list = [("parallelepipedo", (0.5, 0.0, 0.0), (0.2,0.2,3.50)),
                     ("parallelepipedo", (0.0,  0.5, 0.0), (0.2,0.2,3.50)),
                     ("parallelepipedo", (0.0, -0.5, 0.0), (0.2,0.2,3.50))]
    
    #settin rrt star
    rrt_star = My_RRTStar(
        start=start_joint_space,
        goal=goal_joint_space,
        rand_area=[-2*math.pi, 2*math.pi],  # Esplora tutto il joint space
        max_iter=15000,  # Aumenta la ricerca
        expand_dis=0.8,  # Copre più spazio tra i nodi
        goal_sample_rate=20,  # Favorisce l'esplorazione
        connect_circle_dist=90.0,  # Riconnette più nodi
        robot=doosan,
        obstacle_list=obstacle_list,
        path_resolution=0.2
    )
    rospy.loginfo(f"joint cartesian: {doosan.get_joint_positions}")

    path = rrt_star.planning(search_until_max_iter=False)
    if path is None:
        print("Cannot find path.")
    else:
        print("Found path!")
        path.reverse()
        rospy.loginfo(f"Percorso calcolato in radianti...")
        rospy.loginfo(f"numero di nodi trovati: {len(path)}")

    ee_positions = []
    for joint_angles in path:  # Per ogni configurazione dei giunti trovata
        cartesian_pos = doosan.forward_kinematics(joint_angles)  # Cinematica diretta
        if cartesian_pos is not None:
            ee_positions.append(cartesian_pos)

    path_deg = convert_to_degrees(path)
    rospy.loginfo("Percorso calcolato in gradi:")  # Stampa il percorso convertito
    marker_pub_traj = rospy.Publisher("/ee_trajectory", MarkerArray, queue_size=len(path))
    marker_pub_obstacles = rospy.Publisher("/obs", Marker, queue_size=1)

    publish_end_effector_trajectory(ee_positions, marker_pub_traj)



    while not rospy.is_shutdown():
        publish_end_effector_trajectory(ee_positions, marker_pub_traj)
        publish_obstacles(obstacle_list, marker_pub_obstacles)

        if path_deg:
            qlist = []
            for point in path_deg:            
                pos= posj(point[0], point[1], point[2],point[3], point[4], point[5]) #joint
                qlist.append(pos)
        else:
            rospy.logerr("Nessun percorso trovato!")
            
        movesj(qlist, vel=30, acc=50)


    print('good bye!')