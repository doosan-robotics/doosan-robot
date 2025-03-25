#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
#Motion planning cartesiano con rrt, prima implementazione soltanto per vedere come si muove con ostacoli sferici e parallepipedi
# abbiamo mantenuto le  stesse carattestiche dell'algoritmo orgininale
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

def create_marker(marker_id, position, color, scale=1, shape=Marker.SPHERE,text=None):
    """
    Crea un marker RViz.
    :param marker_id: ID del marker
    :param position: Posizione [x, y, z]
    :param color: Colore (RGBA)
    :param scale: Dimensione del marker
    :param shape: Forma del marker (default: SPHERE)
    :return: Marker configurato
    """
    marker = Marker()
    marker.header.frame_id = "base_0"  # Nome del frame (adattalo se necessario)
    marker.header.stamp = rospy.Time.now()
    # marker.ns = "markers"
    marker.id = marker_id
    marker.type = shape
    # marker.action = Marker.ADD
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color = color
    if text:
        marker.text = text

    # marker.lifetime = rospy.Duration(0)  # Persistente
    return marker
def publish_markers(start, goal, obstacle_list, marker_pub_1, marker_pub_2, marker_pub_obstacles, marker_pub_1_text, marker_pub_2_text, marker_pub_obstacles_text):
    """
    Pubblica i marker su RViz per i punti di partenza, arrivo e ostacoli.
    """
    rospy.sleep(0.01) 

    # Marker del punto di partenza
    start_marker = create_marker(
        marker_id=1,
        position=start,
        color=ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Verde
        scale=0.1
    )
    marker_pub_1.publish(start_marker)
    rospy.sleep(0.01) 

    start_marker_text = create_marker(
        marker_id=4,
        position=[start[0], start[1], start[2] + 0.6],
        color=ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Verde
        scale=0.3,
        shape=Marker.TEXT_VIEW_FACING,
        text="start"
    )
    marker_pub_1_text.publish(start_marker_text)
    rospy.sleep(0.01) 

    rospy.loginfo("________ pubblicato ________")

    # Marker del punto di arrivo
    goal_marker = create_marker(
        marker_id=2,
        position=goal,
        color=ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Rosso
        scale=0.1
    )
    marker_pub_2.publish(goal_marker)
    rospy.sleep(0.01) 

    goal_marker_text = create_marker(
        marker_id=5,
        position=[goal[0], goal[1], goal[2] + 0.6],
        color=ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Rosso
        scale=0.3,
        shape=Marker.TEXT_VIEW_FACING,
        text="goal"
    )
    marker_pub_2_text.publish(goal_marker_text)
    rospy.sleep(0.01) 

    # Pubblicazione degli ostacoli
    for idx, obstacle in enumerate(obstacle_list):
        x, y, z, radius = obstacle
        obstacle_marker = create_marker(
            marker_id=10 + idx,  # ID univoco per ogni ostacolo
            position=[x, y, z],
            color=ColorRGBA(0.0, 0.0, 1.0, 0.5),  # Blu trasparente
            scale=radius * 2,  # Diametro della sfera
            shape=Marker.SPHERE
        )
        marker_pub_obstacles.publish(obstacle_marker)
        rospy.sleep(0.01)

        obstacle_marker_text = create_marker(
            marker_id=20 + idx,  # ID univoco per il testo
            position=[x, y, z + 0.6],
            color=ColorRGBA(0.0, 0.0, 1.0, 0.5),  # Blu trasparente
            scale=0.3,
            shape=Marker.TEXT_VIEW_FACING,
            text=f"obstacle {idx+1}"
        )
        marker_pub_obstacles_text.publish(obstacle_marker_text)
        rospy.sleep(0.01)


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
        def __init__(self, x):
            self.x = x
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

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node,
                                               self.robot,
                                               self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

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
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(np.random.uniform(self.min_rand,
                                              self.max_rand,
                                              self.dimension))
        else:  # goal point sampling
            rnd = self.Node(self.end.x)
        return rnd

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(list(from_node.x))
        d, phi, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [list(new_node.x)]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        start, end = np.array(from_node.x), np.array(to_node.x)
        v = end - start
        u = v / (np.sqrt(np.sum(v ** 2)))
        for _ in range(n_expand):
            new_node.x += u * self.path_resolution
            new_node.path_x.append(list(new_node.x))

        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(list(to_node.x))

        new_node.parent = from_node

        return new_node

    def draw_graph(self, rnd=None):
        plt.cla()
        self.ax.axis([-1, 1, -1, 1, -1, 1])
        self.ax.set_zlim(0, 1)
        self.ax.grid(True)
        for (ox, oy, oz, size) in self.obstacle_list:
            self.plot_sphere(self.ax, ox, oy, oz, size=size)
        if self.dimension > 3:
            return self.ax
        if rnd is not None:
            self.ax.plot([rnd.x[0]], [rnd.x[1]], [rnd.x[2]], "^k")
        for node in self.node_list:
            if node.parent:
                path = np.array(node.path_x)
                plt.plot(path[:, 0], path[:, 1], path[:, 2], "-g")
        self.ax.plot([self.start.x[0]], [self.start.x[1]],
                     [self.start.x[2]], "xr")
        self.ax.plot([self.end.x[0]], [self.end.x[1]], [self.end.x[2]], "xr")
        plt.pause(0.01)
        return self.ax

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [np.sum((np.array(node.x) - np.array(rnd_node.x))**2)
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def plot_sphere(ax, x, y, z, size=1, color="k"):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        xl = x+size*np.cos(u)*np.sin(v)
        yl = y+size*np.sin(u)*np.sin(v)
        zl = z+size*np.cos(v)
        ax.plot_wireframe(xl, yl, zl, color=color)

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
    def check_collision(node, robot, obstacleList):

        if node is None:
            return False

        for (ox, oy, oz, size) in obstacleList:
            for x in node.path_x:
                x_list, y_list, z_list = robot.get_points(x)
                dx_list = [ox - x_point for x_point in x_list]
                dy_list = [oy - y_point for y_point in y_list]
                dz_list = [oz - z_point for z_point in z_list]
                d_list = [dx * dx + dy * dy + dz * dz
                          for (dx, dy, dz) in zip(dx_list,
                                                  dy_list,
                                                  dz_list)]

                if min(d_list) <= size ** 2:
                    return False  # collision

        return True  # safe



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
    start_cartesian = [0.14, -0.25, 0.3]  # [x, y, z]
    goal_cartesian = [-0.2, 0.2, 0.2]


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


    # ====Search Path with RRT====
    obstacle_list = [
        (0.4, 0.65, 0.6, 0.1),
        (0.3, 0.5, 0.7, 0.1),
        (0.3, 0.3, 0.4, 0.1),
        (0.4, 0.3, 0.5, 0.1),
        (0.4, 0.55, 0.3, 0.1),

    ]  # [x,y,size(radius)]


    rrt_star = RRTStar(
        start=start_joint_space,
        goal=goal_joint_space,
        rand_area=[-math.pi, math.pi],  # Esplora tutto il joint space
        max_iter=3000,  # Aumenta la ricerca
        expand_dis=0.7,  # Copre più spazio tra i nodi
        goal_sample_rate=10,  # Favorisce l'esplorazione
        connect_circle_dist=80.0,  # Riconnette più nodi
        robot=six_joint_arm,
        obstacle_list=obstacle_list
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
            for point in path_deg:
                pos= posj(point[0], point[1], point[2],point[3], point[4], point[5]) #joint
                rospy.loginfo(f"Spostamento a punto: {pos}")
                movej(pos, vel=30, acc=60)
                rospy.loginfo(f"raggiunto: {pos}")

                rospy.sleep(0.1)  # Attesa tra i movimenti per dare tempo al robot
        else:
            rospy.logerr("Nessun percorso trovato!")

    print('good bye!')