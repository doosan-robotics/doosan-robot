#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import ColorRGBA

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

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
def publish_end_effector_trajectory(ee_positions, marker_pub):
    # Creazione del MarkerArray
    marker_array = MarkerArray()

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
    marker_spheres.scale.x = 0.01
    marker_spheres.scale.y = 0.01
    marker_spheres.scale.z = 0.01

    marker_spheres.color.r = 0.0
    marker_spheres.color.g = 0.0
    marker_spheres.color.b = 1.0
    marker_spheres.color.a = 1.0  # Opacità

    # Per ogni configurazione dei giunti, calcoliamo la posizione dell'end-effector
    for ee_pos in ee_positions:
        if ee_pos is None:
            continue
        point = Point()
        point.x, point.y, point.z = ee_pos
        marker_line.points.append(point)
        marker_spheres.points.append(point)

    # Aggiungi i marker al MarkerArray
    marker_array.markers.append(marker_line)
    marker_array.markers.append(marker_spheres)

    # Pubblica il MarkerArray
    marker_pub.publish(marker_array)

    rospy.loginfo("________ Traiettoria Pubblicata come MarkerArray ________")



def publish_obstacles(obstacle_list, marker_pub_obstacles):
    """
    Pubblica solo gli ostacoli su RViz (sia sfere che parallelepipedi), senza i testi identificativi.

    :param obstacle_list: Lista di ostacoli. Ogni ostacolo è una tupla (tipo, posizione, dimensione).
    :param marker_pub_obstacles: Publisher per i marker degli ostacoli.
    """
    rospy.sleep(0.01)

    for idx, obstacle in enumerate(obstacle_list):
        obstacle_type = obstacle[0]  # "sfera" o "parallelepipedo"

        if obstacle_type == "sfera":
            x, y, z = obstacle[1]
            radius = float(obstacle[2])  # Converti in float
            shape = Marker.SPHERE
            scale = (radius * 2, radius * 2, radius * 2)  # Diametro della sfera

        elif obstacle_type == "parallelepipedo":
            x, y, z = obstacle[1]
            lx, ly, lz = map(float, obstacle[2])  # Converti in float
            shape = Marker.CUBE
            scale = (lx, ly, lz)  # Dimensioni parallelepipedo

        else:
            rospy.logwarn(f"Tipo di ostacolo non riconosciuto: {obstacle_type}")
            continue

        # Crea e pubblica il marker dell'ostacolo
        obstacle_marker = create_marker(
            marker_id=10 + idx,
            position=[x, y, z],
            color=ColorRGBA(0.0, 0.0, 1.0, 0.5),  # Blu trasparente
            scale=scale,
            shape=shape
        )
        marker_pub_obstacles.publish(obstacle_marker)
        rospy.sleep(0.01)

    rospy.loginfo("🟦 Ostacoli pubblicati su RViz senza testo!")


    

class JointStateSubscriber:
    def __init__(self, doosan):
        self.doosan = doosan
        self.current_joint_angles = None
        rospy.Subscriber("/dsr01m0609/joint_states", JointState, self.joint_state_callback)
        self.marker_publisher = rospy.Publisher("/joint_cartesian_positions", MarkerArray, queue_size=10)


    def joint_state_callback(self, msg):
        """
        Callback che riceve gli angoli attuali dei giunti dal topic /joint_states
        """
        self.current_joint_angles = list(msg.position)
        self.publish_joint_markers()

    def publish_joint_markers(self):
        """
        Pubblica le posizioni cartesiane dei giunti come sfere in un MarkerArray per RViz.
        """
        if self.current_joint_angles is not None:
            joint_positions = self.doosan.get_joint_positions(self.current_joint_angles)
            marker_array = MarkerArray()

            for i, (x, y, z) in enumerate(joint_positions):
                marker = Marker()
                marker.header = Header()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "base_0"  # Cambia con il frame corretto

                marker.ns = "joint_spheres"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z

                marker.scale.x = 0.15  # Raggio della sfera
                marker.scale.y = 0.15
                marker.scale.z = 0.15

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.2  # Opacità piena

                marker.lifetime = rospy.Duration()

                marker_array.markers.append(marker)

            self.marker_publisher.publish(marker_array)
