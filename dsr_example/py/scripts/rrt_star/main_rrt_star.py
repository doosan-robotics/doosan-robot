#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import threading, time
import sys
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import threading, time
from kdl_parser_py.urdf import treeFromFile
import math
import sys
import pathlib
from sensor_msgs.msg import JointState
import numpy as np

from n_joint_arm_3d.NLinkArm3d import NLinkArm
from robot_arm import RobotArm
from marker_publisher import publish_markers,publish_end_effector_trajectory
from utilies_rrt_star import inverse_kinematics,convert_to_degrees
from rrt_star import RRTStar




sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 
sys.path.append(str(pathlib.Path(__file__).parent.parent))



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
    #rospy.spinner(2)    
  

if __name__ == "__main__":
    
    rospy.init_node('rrt_motion_planning')
    rospy.on_shutdown(shutdown)
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    marker_pub_1 = rospy.Publisher("/start", Marker, queue_size=1)
    marker_pub_2 = rospy.Publisher("/stop", Marker, queue_size=1)
    marker_pub_3 = rospy.Publisher("/obs", Marker, queue_size=1)
    marker_pub_1_text = rospy.Publisher("/start_text", Marker, queue_size=1)
    marker_pub_3_text = rospy.Publisher("/obs_text", Marker, queue_size=1)
    marker_pub_2_text = rospy.Publisher("/stop_text", Marker, queue_size=1)
    r = rospy.Rate(100) 
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    #IL PROBLEMA CHE PER OTTENERE LE POSIZIONI DI PARTENZA E TARGET USO DELLE FUNZIONI DIVERSI DI CINEMATICA DIRETTA E INVERSA
    #RISPETTO A QUELLE CHE USO NLINKARM3D NON HO CAPITO COME SONO IMPLEMENTATE
    #IL PROBLEMA È SORTO ANDATO A VISUALIZZARE LE POSIZIONI DEI GIUNTI CON GETPOSITION DELLA CLASSE CHE NON CORRISPONDO 
    # A QUELLE DEL ROBOT MA SEMBRANO, A PARTE LA BASE, PARTIRE DA SOTTO IL PIANO COME SE FOSSERO SPECULARI.
    # QUESTO VUOL DIRE CHE LE POSIZIONI OTTENUTE DA GET POSITION NON SONO CORRETTE E QUINDI NON SO COME EFFETTUA  I CONTROLLI
    # FORSE L'END EFFECTOR È QUELLA CONTROLLATA MA IL RESTO DEL CORRPO È SBAGLIATO
    # NON FUNZIONA !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!









    



    ### CON CLASSE ROBOT ARM
    #
    #PRESI DAL FILE URDF DEL DOOSAN M0609 MA HO COMPENSATO ALCUNI VALORI NEGATIVI AGGIUNGENDO +PI ALLA SECONDA COLONNA
    #CAPIRE SE È SENSATO
    six_joint_arm= RobotArm([
        [0., math.pi/2., 0., 0.1525],  # Link 1
        [0., 0., 0.006, 0.0],          # Link 2
        [0., 0., 0.411, 0.0],          # Link 3
        [math.pi, math.pi/2., 0.368, 0.223],  # Link 4 (+pi per compensare il valore negativo)
        [0., -math.pi/2., 0., 0.103],  # Link 5
        [math.pi, 0., 0.121, 0.0]      # Link 6 (+pi per compensare il valore negativo)
    ])
    # Inizializza il braccio Doosan m0609 con i parametri DH
    # [theta, alpha, a, d]
    # MI TORNAVA MA NON SO DOVE SONO STATI PRESI
    # six_joint_arm = RobotArm([
    #     [0., math.pi/2., 0., 0.127],
    #     [0., 0., 0.350, 0.0],
    #     [0., 0., 0.290, 0.0],
    #     [0., math.pi/2., 0., 0.223],
    #     [0., -math.pi/2., 0., 0.103],
    #     [0., 0., 0., 0.100]
    # ])
  

    goal_cartesian = [0.4, 0.4, 0.4]  # [x, y, z]
    #goal_rot = [math.pi/2, math.pi/2, math.pi/2]  # Orientamento desiderato (roll, pitch, yaw)

    start_cartesian = [-0.4, 0.4, 0.4]
    #start_rot = [-math.pi/2, -math.pi/2, math.pi/2]  # Orientamento desiderato (roll, pitch, yaw)

    # Carica la catena cinematcca dal file URDF
    urdf_file = "/home/jntlbap/catkin_ws/src/doosan-robot/dsr_description/urdf/m0609.urdf"
    chain = treeFromFile(urdf_file)[1].getChain("base_0", "link6")

        # Converti in spazio dei giunti

    start_joint_space =inverse_kinematics(chain, start_cartesian)
    goal_joint_space =inverse_kinematics(chain, goal_cartesian)

    if start_joint_space is None or goal_joint_space is None:
        rospy.logerr("Errore nella cinematica inversa: impossibile trovare una soluzione valida")
    else:
        rospy.loginfo(f"Configurazione iniziale in joint space: {start_joint_space}")
        rospy.loginfo(f"Configurazione finale in joint space: {goal_joint_space}")


    obstacle_list = [
        #virtual test
        #  ("parallelepipedo", ( 0.55, -0.55, 0.0), (0.6, 0.6, 3.70)),#("parallelepipedo", (0.51, 0.33, 0.24), (0.21, 0.50, 0.73)),
        #  ("parallelepipedo", ( -0.65, -0.65, 0.0), (0.6, 0.6, 3.70)),#("parallelepipedo", (0.51, 0.33, 0.24), (0.21, 0.50, 0.73)),
        #  ("parallelepipedo", ( -0.55, 0.55, 0.0), (0.6, 0.6, 3.70)),#("parallelepipedo", (0.51, 0.33, 0.24), (0.21, 0.50, 0.73)),
        #real test
         #("parallelepipedo", (-0.28, 0.55, 0.30), (0.44,0.17,0.59)),
        # ("parallelepipedo", (-0.20, 0.40, 0.0), (0.44,0.17,1.59)),
        #("parallelepipedo", (0.0, 0.5, 0.0), (0.2,0.2,.5)),
        ("parallelepipedo", (0.5, 0.0, 0.0), (0.2,0.2,3.50)),
        ("parallelepipedo", (0.0, 0.5, 0.0), (0.2,0.2,3.50))
    ]

    
    publish_markers(start_cartesian,
                     goal_cartesian,
                     obstacle_list,
                     marker_pub_1,
                     marker_pub_2,
                     marker_pub_3,
                     marker_pub_1_text,
                     marker_pub_2_text,
                     marker_pub_3_text)

    rrt_star = RRTStar(
        start=start_joint_space,
        goal=goal_joint_space,
        rand_area=[-2*math.pi, 2*math.pi],  # Esplora tutto il joint space
        max_iter=20000,  # Aumenta la ricerca
        expand_dis=0.8,  # Copre più spazio tra i nodi
        goal_sample_rate=10,  # Favorisce l'esplorazione
        connect_circle_dist=85.0,  # Riconnette più nodi
        robot=six_joint_arm,
        obstacle_list=obstacle_list,
        path_resolution=0.25
    )

    path = rrt_star.planning(animation=show_animation,
                             search_until_max_iter=False)
    if path is None:
        print("Cannot find path.")
    else:
        print("Found path!")
        path.reverse()
        rospy.loginfo(f"Percorso calcolato in radianti...")
        rospy.loginfo(f"numero di nodi trovati: {len(path)}")



    marker_pub_traj = rospy.Publisher("/ee_trajectory", Marker, queue_size=len(path))
    # Convertiamo il percorso in gradi
    path_deg = convert_to_degrees(path)
    rospy.loginfo(f"Percorso calcolato in gradi..")




    while not rospy.is_shutdown():

        
        publish_markers(start_cartesian, goal_cartesian, obstacle_list,marker_pub_1,marker_pub_2,marker_pub_3,marker_pub_1_text,marker_pub_2_text,marker_pub_3_text)
        rospy.loginfo("Pubblicazione della traiettoria dell'end-effector su RViz...")
        publish_end_effector_trajectory(path, chain, marker_pub_traj)
        r.sleep()

        if path_deg:


            rospy.loginfo("Inizio esecuzione della traiettoria...")

            qlist = []
            for point in path_deg:
                pos= posj(point[0], point[1], point[2],point[3], point[4], point[5]) #joint
                #rospy.loginfo(f"Spostamento a punto: {pos}")
                qlist.append(pos)
                # movej(pos, vel=50, acc=60)
                # rospy.loginfo(f"raggiunto: {pos}")
                # rospy.sleep(0.1)  # Attesa tra i movimenti per dare tempo al robot
        else:
            rospy.logerr("Nessun percorso trovato!")
        
        movesj(qlist, vel=70, acc=90)


    print('good bye!')