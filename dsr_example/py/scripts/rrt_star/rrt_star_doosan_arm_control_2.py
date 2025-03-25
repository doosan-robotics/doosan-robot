#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# # ##
#Motion planning cartesiano con rrt, seconda implementazione soltanto per vedere come si muove con ostacoli sferici e parallepipedi
# abbiamo cambiato i paraemtri dell'algoritmo orgininale e aumentato i numeri degli ostacli da evitare
# abbiamo implementato un test reale e virtuale
# usa dei segmenti tra i vari link e controlla se questi segmenti entrano i contatto con gli ostacoli
# TO DO: implementare il controllo dei volumi (NON CONSIDERA I VOLUMI PER ADESSO)
# PENALIZZA I NODI VICINO AGLI OSTACOLI ESCLUDENDOLI DAI PERCORSI POSSIBILI!!
import rospy
import os
import threading, time
import sys

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA

import PyKDL
import urdf_parser_py.urdf
import threading, time
from kdl_parser_py.urdf import treeFromFile

import math
import random
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 


sys.path.append(str(pathlib.Path(__file__).parent.parent))
from n_joint_arm_3d.NLinkArm3d import NLinkArm


# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m0609"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *
show_animation = False
verbose = False
import PyKDL


############ DA SOSTITUIRE PERCHé USO INVERSIONE CINEMATICHE DIVERSE   ############

def inverse_kinematics(chain, target_pos, target_rot=None):
    """
    Calcola la cinematica inversa per ottenere i valori dei giunti dati (x, y, z).
    :param chain: Catena cinematica del robot
    :param target_pos: Posizione (x, y, z) desiderata
    :param target_rot: (opzionale) Matrice di rotazione desiderata
    :return: Lista con gli angoli di giunto o None se non trovato
    """
    ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)

    # Configurazione iniziale casuale
    q_init = PyKDL.JntArray(chain.getNrOfJoints())

    # Definisce la posizione finale desiderata
    end_effector_frame = PyKDL.Frame()
    end_effector_frame.p = PyKDL.Vector(*target_pos)

    if target_rot:
        end_effector_frame.M = target_rot

    joint_result = PyKDL.JntArray(chain.getNrOfJoints())

    if ik_solver.CartToJnt(q_init, end_effector_frame, joint_result) >= 0:
        return [joint_result[i] for i in range(chain.getNrOfJoints())]
    else:
        return None  # Nessuna soluzione trovata








def create_marker(marker_id, position, color, scale=1.0, shape=Marker.SPHERE, text=None):
    """
    Crea un marker RViz.
    :param marker_id: ID del marker
    :param position: Posizione [x, y, z]
    :param color: Colore (RGBA)
    :param scale: Dimensione del marker (float o tuple)
    :param shape: Forma del marker (default: SPHERE)
    :return: Marker configurato
    """
    marker = Marker()
    marker.header.frame_id = "base_0"  # Nome del frame
    marker.header.stamp = rospy.Time.now()
    marker.id = marker_id
    marker.type = shape
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Se scale è un singolo valore, usa per tutti gli assi
    if isinstance(scale, (int, float)):
        marker.scale.x = float(scale)
        marker.scale.y = float(scale)
        marker.scale.z = float(scale)
    elif isinstance(scale, (list, tuple)) and len(scale) == 3:
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
    else:
        rospy.logerr(f"Errore: `scale` deve essere float o lista di 3 float, ricevuto {scale}")
        return None

    marker.color = color

    if text:
        marker.text = text
        marker.scale.x = 0.0  # Per i testi, x e y devono essere 0
        marker.scale.y = 0.0

    return marker
def publish_markers(start, goal, obstacle_list, marker_pub_1, marker_pub_2, marker_pub_obstacles, 
                    marker_pub_1_text, marker_pub_2_text, marker_pub_obstacles_text):
    """
    Pubblica i marker su RViz per i punti di partenza, arrivo e ostacoli (sia sfere che parallelepipedi).
    """
    rospy.sleep(0.01)

    # Marker del punto di partenza (Verde)
    start_marker = create_marker(
        marker_id=1,
        position=start,
        color=ColorRGBA(0.0, 1.0, 0.0, 1.0),
        scale=0.1
    )
    marker_pub_1.publish(start_marker)
    rospy.sleep(0.01)

    start_marker_text = create_marker(
        marker_id=4,
        position=[start[0], start[1], start[2] + 0.6],
        color=ColorRGBA(0.0, 1.0, 0.0, 1.0),
        scale=0.3,
        shape=Marker.TEXT_VIEW_FACING,
        text="START"
    )
    marker_pub_1_text.publish(start_marker_text)
    rospy.sleep(0.01)

    rospy.loginfo("________ Punti Start/Goal pubblicati ________")

    # Marker del punto di arrivo (Rosso)
    goal_marker = create_marker(
        marker_id=2,
        position=goal,
        color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
        scale=0.1
    )
    marker_pub_2.publish(goal_marker)
    rospy.sleep(0.01)

    goal_marker_text = create_marker(
        marker_id=5,
        position=[goal[0], goal[1], goal[2] + 0.6],
        color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
        scale=0.3,
        shape=Marker.TEXT_VIEW_FACING,
        text="GOAL"
    )
    marker_pub_2_text.publish(goal_marker_text)
    rospy.sleep(0.01)

    # Pubblicazione degli ostacoli
    for idx, obstacle in enumerate(obstacle_list):
        obstacle_type = obstacle[0]  # "sfera" o "parallelepipedo"

        if obstacle_type == "sfera":
            x, y, z = obstacle[1]
            radius = float(obstacle[2])  # Assicurati che sia un float
            shape = Marker.SPHERE
            scale = (radius * 2, radius * 2, radius * 2)  # Float separati

        elif obstacle_type == "parallelepipedo":
            x, y, z = obstacle[1]
            lx, ly, lz = map(float, obstacle[2])  # Converti le dimensioni in float
            shape = Marker.CUBE
            scale = (lx, ly, lz)  # Float separati

        else:
            rospy.logwarn(f"Tipo di ostacolo non riconosciuto: {obstacle_type}")
            continue

        # Pubblica il marker dell'ostacolo
        obstacle_marker = create_marker(
            marker_id=10 + idx,
            position=[x, y, z],
            color=ColorRGBA(0.0, 0.0, 1.0, 0.5),  # Blu trasparente
            scale=scale,  # Passiamo tuple di float
            shape=shape
        )
        marker_pub_obstacles.publish(obstacle_marker)
        rospy.sleep(0.01)

        # Pubblica il testo identificativo dell'ostacolo
        obstacle_marker_text = create_marker(
            marker_id=20 + idx,
            position=[x, y, z + 0.6],
            color=ColorRGBA(0.0, 0.0, 1.0, 0.5),
            scale=0.3,
            shape=Marker.TEXT_VIEW_FACING,
            text=f"Obstacle {idx+1}"
        )
        marker_pub_obstacles_text.publish(obstacle_marker_text)
        rospy.sleep(0.01)

    rospy.loginfo("________ Ostacoli pubblicati ________")

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
  








class RobotArm(NLinkArm):
    def get_points(self, joint_angle_list):
        self.set_joint_angles(joint_angle_list)

        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)

        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])

        return x_list, y_list, z_list


class RRTStar:
    """
    Class for RRT Star planning
    """

    class Node:
        def __init__(self, x, orientation=None):
            self.x = x  # Posizione [x, y, z] o angoli dei giunti
            self.orientation = orientation if orientation else [0, 0, 0]  # [roll, pitch, yaw]
            self.parent = None
            self.cost = 0.0


    def __init__(self, start, goal, robot, obstacle_list, rand_area,
                 expand_dis=.30,
                 path_resolution=.1,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0
                 ):
        """
        Setting Parameter

        start:Start Position [q1,...,qn]
        goal:Goal Position [q1,...,qn]
        obstacleList:obstacle Positions [[x,y,z,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.dimension = len(start)
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot = robot
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal)
        self.node_list = []
        if show_animation:
            self.ax = plt.axes(projection='3d')

    def planning(self, animation=False, search_until_max_iter=False):
        """
        rrt star path planning

        animation: flag for animation on or off
        search_until_max_iter: search until max iteration for path
        improving or not
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            if verbose:
                print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind],
                                  rnd,
                                  self.expand_dis)

            if self.check_collision(new_node, self.robot, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)

            if animation and i % 5 == 0 and self.dimension <= 3:
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)
        if verbose:
            print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        # Lista per salvare i costi dei nodi validi
        costs = []
        
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)

            if t_node and self.check_collision(t_node, self.robot, self.obstacle_list):
                base_cost = self.calc_new_cost(near_node, new_node)

                # **Penalizzazione per la vicinanza agli ostacoli**
                penalty = self.obstacle_proximity_penalty(new_node)  
                total_cost = base_cost + penalty

                costs.append(total_cost)
            else:
                costs.append(float("inf"))  # Nodo non valido

        min_cost = min(costs)

        if min_cost == float("inf"):
            print("Nessun percorso valido: tutti i nodi attraversano ostacoli.")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

    def obstacle_proximity_penalty(self, node):
        """
        Aumenta il costo del percorso se è vicino agli ostacoli
        """
        penalty = 0.0
        safe_distance = 0.1  # Distanza minima di sicurezza

        # Prendi solo le coordinate cartesiane (x, y, z), ignorando gli angoli dei giunti
        node_pos = np.array(node.x[:3])  

        for obstacle in self.obstacle_list:
            shape, center, size = obstacle
            obstacle_center = np.array(center)

            if shape == "sfera":
                radius = size
                distance = np.linalg.norm(node_pos - obstacle_center) - radius
            elif shape == "parallelepipedo":
                lx, ly, lz = size
                min_bounds = obstacle_center - np.array([lx / 2, ly / 2, lz / 2])
                max_bounds = obstacle_center + np.array([lx / 2, ly / 2, lz / 2])

                # Corretto: confrontiamo solo le prime 3 coordinate
                distance = np.min(np.maximum(min_bounds - node_pos, node_pos - max_bounds))  

            if distance < safe_distance:
                penalty += (safe_distance - distance) * 10  # Penalità più alta per distanze minori

        return penalty


    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x)
                             for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i)
                     for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.robot, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in
        # a range no more than expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [np.sum((np.array(node.x) - np.array(new_node.x)) ** 2)
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_inds

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node,
                                                self.robot,
                                                self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def generate_final_course(self, goal_ind):
        path = [self.end.x]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.x)
            node = node.parent
        path.append(node.x)
        reversed(path)
        return path

    def calc_dist_to_goal(self, x):
        distance = np.linalg.norm(np.array(x) - np.array(self.end.x))
        return distance
    def get_random_node(self):
        max_attempts = 10  # Prova fino a 10 volte a trovare un nodo sicuro
        for _ in range(max_attempts):
            if random.randint(0, 100) > self.goal_sample_rate:
                rnd = self.Node(np.random.uniform(self.min_rand, self.max_rand, self.dimension),
                                [random.uniform(-np.pi, np.pi) for _ in range(3)])  # Orientamento casuale
            else:
                rnd = self.Node(self.end.x, self.end.orientation)

            # Generiamo un nodo temporaneo per verificare la collisione con un breve step
            temp_node = self.steer(self.start, rnd, extend_length=0.1)
            if self.check_collision(temp_node, self.robot, self.obstacle_list):
                return rnd  # Se il nodo è sicuro, lo usiamo

        return None  # Se non troviamo nulla di valido, restiamo fermi


    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(list(from_node.x), list(from_node.orientation))
        d, phi, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [list(new_node.x)]
        new_node.path_orientation = [list(new_node.orientation)]

        if extend_length > d:
            extend_length = d

        n_expand = max(2, math.floor(extend_length / self.path_resolution))  # Assicuriamoci di avere almeno 2 punti

        start, end = np.array(from_node.x), np.array(to_node.x)
        v = end - start
        u = v / (np.linalg.norm(v) + 1e-6)  # Evita divisioni per zero

        for _ in range(n_expand):
            new_node.x += u * self.path_resolution
            new_node.orientation += (np.array(to_node.orientation) - np.array(from_node.orientation)) / n_expand
            new_node.path_x.append(list(new_node.x))
            new_node.path_orientation.append(list(new_node.orientation))

        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(list(to_node.x))
            new_node.path_orientation.append(list(to_node.orientation))

        new_node.parent = from_node
        return new_node



    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [np.sum((np.array(node.x) - np.array(rnd_node.x))**2)
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind


    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x[0] - from_node.x[0]
        dy = to_node.x[1] - from_node.x[1]
        dz = to_node.x[2] - from_node.x[2]
        d = np.sqrt(np.sum((np.array(to_node.x) - np.array(from_node.x))**2))
        phi = math.atan2(dy, dx)
        theta = math.atan2(math.hypot(dx, dy), dz)
        return d, phi, theta

    @staticmethod
    def calc_distance_and_angle2(from_node, to_node):
        dx = to_node.x[0] - from_node.x[0]
        dy = to_node.x[1] - from_node.x[1]
        dz = to_node.x[2] - from_node.x[2]
        d = math.sqrt(dx**2 + dy**2 + dz**2)
        phi = math.atan2(dy, dx)
        theta = math.atan2(math.hypot(dx, dy), dz)
        return d, phi, theta
    @staticmethod
    def check_collision(node, robot, obstacleList, min_samples=30, max_samples=80):
        if node is None:
            return False

        for obstacle in obstacleList:
            shape = obstacle[0]

            for x in node.path_x:
                x_list, y_list, z_list = robot.get_points(x)

                for i in range(len(x_list) - 1):
                    p1 = np.array([x_list[i], y_list[i], z_list[i]])
                    p2 = np.array([x_list[i + 1], y_list[i + 1], z_list[i + 1]])

                    # Il numero di campioni è proporzionale alla lunghezza del segmento
                    segment_length = np.linalg.norm(p2 - p1)
                    num_samples = max(min_samples, min(max_samples, int(segment_length / 0.02)))  
                    sampled_points = np.linspace(p1, p2, num_samples)

                    for p in sampled_points:
                        if shape == "sfera":
                            ox, oy, oz = obstacle[1]
                            radius = obstacle[2]

                            if np.linalg.norm(p - np.array([ox, oy, oz])) <= radius:
                                return False  # Collisione con una sfera

                        elif shape == "parallelepipedo":
                            cx, cy, cz = obstacle[1]
                            lx, ly, lz = obstacle[2]

                            if (abs(p[0] - cx) <= lx / 2 and
                                abs(p[1] - cy) <= ly / 2 and
                                abs(p[2] - cz) <= lz / 2):
                                return False  # Collisione con un parallelepipedo

        return True  # Nessuna collisione

def segment_sphere_collision(p1, p2, center, radius):
    """Controlla se il segmento p1-p2 interseca una sfera di centro 'center' e raggio 'radius'."""
    v = p2 - p1
    w = p1 - center
    c1 = np.dot(w, v)
    c2 = np.dot(v, v)
    b = c1 / c2 if c2 != 0 else 0
    closest_point = p1 + b * v
    closest_dist = np.linalg.norm(closest_point - center)

    return closest_dist <= radius


def segment_box_collision(p1, p2, center, size):
    """Controlla se il segmento p1-p2 interseca un parallelepipedo centrato in 'center' e dimensione 'size'."""
    min_bounds = center - size / 2
    max_bounds = center + size / 2
    for i in range(3):  # Controllo separato sugli assi x, y, z
        if (p1[i] < min_bounds[i] and p2[i] < min_bounds[i]) or (p1[i] > max_bounds[i] and p2[i] > max_bounds[i]):
            return False
    return True  # Il segmento attraversa il parallelepipedo



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
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    marker_pub_1 = rospy.Publisher("/start", Marker, queue_size=1)
    marker_pub_2 = rospy.Publisher("/stop", Marker, queue_size=1)
    marker_pub_3 = rospy.Publisher("/obs", Marker, queue_size=1)
    marker_pub_1_text = rospy.Publisher("/start_text", Marker, queue_size=1)
    marker_pub_2_text = rospy.Publisher("/stop_text", Marker, queue_size=1)
    marker_pub_3_text = rospy.Publisher("/obs_text", Marker, queue_size=1)
    # Definizione delle coordinate cartesiane di partenza e arrivo
    # start_cartesian = [0.3, 0.3, 0.6]  # [x, y, z]
    # goal_cartesian = [-0.3, -0.3, 0.6]
    goal_cartesian = [0.3, 0.3, 0.6]  # [x, y, z]
    start_cartesian = [-0.3, -0.3, 0.6]


    #virtual testt
    # start_cartesian = [0.5, 0.4, 0.6]  # [x, y, z]
    # #goal_cartesian = [-0.3, -0.3, 0.5]
    # #goal_cartesian = [-0.1, -0.5, 0.6]
    # goal_cartesian = [-0.5, 0.0, 0.6]

    # Carica la catena cinematcca dal file URDF
    urdf_file = "/home/jntlbap/catkin_ws/src/doosan-robot/dsr_description/urdf/m0609.urdf"
    chain = treeFromFile(urdf_file)[1].getChain("base_0", "link6")

        # Converti in spazio dei giunti
    start_joint_space = inverse_kinematics(chain, start_cartesian)
    goal_joint_space = inverse_kinematics(chain, goal_cartesian)

    if start_joint_space is None or goal_joint_space is None:
        rospy.logerr("Errore nella cinematica inversa: impossibile trovare una soluzione valida")
    else:
        rospy.loginfo(f"Configurazione iniziale in joint space: {start_joint_space}")
        rospy.loginfo(f"Configurazione finale in joint space: {goal_joint_space}")


    r = rospy.Rate(100) 
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()
    




    # Inizializza il braccio Doosan m0609 con i parametri DH
    # [theta, alpha, a, d]
    # Inizializza il braccio Doosan M0609 con i parametri DH
    six_joint_arm = RobotArm([
        [0., math.pi/2., 0., 0.127],
        [0., 0., 0.350, 0.0],
        [0., 0., 0.290, 0.0],
        [0., math.pi/2., 0., 0.223],
        [0., -math.pi/2., 0., 0.103],
        [0., 0., 0., 0.100]
    ])



    obstacle_list = [
        #virtual test
        #  ("parallelepipedo", ( 0.55, -0.55, 0.0), (0.6, 0.6, 3.70)),#("parallelepipedo", (0.51, 0.33, 0.24), (0.21, 0.50, 0.73)),
        #  ("parallelepipedo", ( -0.65, -0.65, 0.0), (0.6, 0.6, 3.70)),#("parallelepipedo", (0.51, 0.33, 0.24), (0.21, 0.50, 0.73)),
        #  ("parallelepipedo", ( -0.55, 0.55, 0.0), (0.6, 0.6, 3.70)),#("parallelepipedo", (0.51, 0.33, 0.24), (0.21, 0.50, 0.73)),
        #real test
         #("parallelepipedo", (-0.28, 0.55, 0.30), (0.44,0.17,0.59)),
        ("parallelepipedo", (-0.20, 0.40, 0.30), (0.44,0.17,0.59)),


    ]
    publish_markers(start_cartesian, goal_cartesian, obstacle_list,marker_pub_1,marker_pub_2,marker_pub_3,marker_pub_1_text,marker_pub_2_text,marker_pub_3_text)

    rrt_star = RRTStar(
        start=start_joint_space,
        goal=goal_joint_space,
        rand_area=[-2*math.pi, 2*math.pi],  # Esplora tutto il joint space
        max_iter=5000,  # Aumenta la ricerca
        expand_dis=0.2,  # Copre più spazio tra i nodi
        goal_sample_rate=10,  # Favorisce l'esplorazione
        connect_circle_dist=85.0,  # Riconnette più nodi
        robot=six_joint_arm,
        obstacle_list=obstacle_list,
        path_resolution=0.01

    )

    path = rrt_star.planning(animation=show_animation,
                             search_until_max_iter=False)
    if path is None:
        print("Cannot find path.")
    else:
        print("Found path!")
        path.reverse()
        rospy.loginfo(f"Percorso calcolato in radianti: {path}")

        # # Draw final path
        # if show_animation:
        #     ax = rrt_star.draw_graph()

        #     # Plot final configuration
        #     x_points, y_points, z_points = six_joint_arm.get_points(path[-1])
        #     ax.plot([x for x in x_points],
        #             [y for y in y_points],
        #             [z for z in z_points],
        #             "o-", color="red", ms=5, mew=0.5)

        #     for i, q in enumerate(path):
        #         x_points, y_points, z_points = six_joint_arm.get_points(q)
        #         ax.plot([x for x in x_points],
        #                 [y for y in y_points],
        #                 [z for z in z_points],
        #                 "o-", color="grey",  ms=4, mew=0.5)
        #         plt.pause(0.1)

        #     plt.show()



    # Convertiamo il percorso in gradi
    path_deg = convert_to_degrees(path)
    while not rospy.is_shutdown():
        publish_markers(start_cartesian, goal_cartesian, obstacle_list,marker_pub_1,marker_pub_2,marker_pub_3,marker_pub_1_text,marker_pub_2_text,marker_pub_3_text)
        r.sleep()

        if path_deg:
            rospy.loginfo(f"Percorso calcolato in gradi : {path_deg}")
            # Esecuzione della traiettoria
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
        
        movesj(qlist, vel=30, acc=40)


    print('good bye!')