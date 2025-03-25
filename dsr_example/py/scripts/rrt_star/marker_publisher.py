#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from utilies_rrt_star import forward_kinematics
from geometry_msgs.msg import Point
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy
import numpy as np
from geometry_msgs.msg import Point

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

def publish_end_effector_trajectory(path, chain, marker_pub):
    # Marker per la linea della traiettoria (LINE_STRIP)
    marker_line = Marker()
    marker_line.header.frame_id = "base_0"
    marker_line.header.stamp = rospy.Time.now()
    marker_line.ns = "trajectory"
    marker_line.id = 0
    marker_line.type = Marker.LINE_STRIP
    marker_line.action = Marker.ADD
    marker_line.scale.x = 0.005  # Spessore della linea

    marker_line.color.r = 1.0
    marker_line.color.g = 0.0
    marker_line.color.b = 0.0
    marker_line.color.a = 1.0  # Opacità

    # Marker per le sfere (SPHERE_LIST)
    marker_spheres = Marker()
    marker_spheres.header.frame_id = "base_0"
    marker_spheres.header.stamp = rospy.Time.now()
    marker_spheres.ns = "trajectory_points"
    marker_spheres.id = 1
    marker_spheres.type = Marker.SPHERE_LIST
    marker_spheres.action = Marker.ADD
    # Impostazione delle dimensioni per ogni sfera
    marker_spheres.scale.x = 0.01
    marker_spheres.scale.y = 0.01
    marker_spheres.scale.z = 0.01

    marker_spheres.color.r = 0.0
    marker_spheres.color.g = 0.0
    marker_spheres.color.b = 1.0
    marker_spheres.color.a = 1.0  # Opacità

    # Per ogni configurazione dei giunti, calcoliamo la posizione dell'end-effector
    for joint_angles in path:
        ee_pos = forward_kinematics(chain, joint_angles)
        if ee_pos is None:
            continue
        point = Point()
        point.x, point.y, point.z = ee_pos
        marker_line.points.append(point)
        marker_spheres.points.append(point)

    # Pubblica entrambi i marker
    marker_pub.publish(marker_line)
    marker_pub.publish(marker_spheres)

    rospy.loginfo("________ Traiettoria Pubblicata ________")





