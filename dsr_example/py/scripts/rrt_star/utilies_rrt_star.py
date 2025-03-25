#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import PyKDL
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from PyKDL import ChainFkSolverPos_recursive, JntArray

def convert_to_degrees(path):
    """
    Converte una lista di traiettorie in radianti a gradi.
    
    path: lista di configurazioni articolari in radianti
    return: lista di configurazioni articolari in gradi
    """
    return [[np.degrees(angle) for angle in q] for q in path]


def forward_kinematics(chain, joint_angles):
    """
    Calcola la cinematica diretta e restituisce la posizione dell'end-effector
    """
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    jnt_array = PyKDL.JntArray(len(joint_angles))

    for i in range(len(joint_angles)):
        jnt_array[i] = joint_angles[i]

    ee_frame = PyKDL.Frame()  # Inizializza un frame vuoto

    if fk_solver.JntToCart(jnt_array, ee_frame) < 0:
        rospy.logerr("Errore nella cinematica diretta")
        return None

    position = ee_frame.p  # Estrarre la posizione
    return [position.x(), position.y(), position.z()]


def inverse_kinematics(chain, target_pos, target_rot=None):
    """
    Calcola la cinematica inversa per ottenere i valori dei giunti dati (x, y, z).
    :param chain: Catena cinematica del robot
    :param target_pos: Posizione (x, y, z) desiderata
    :param target_rot: (opzionale) Matrice di rotazione desiderata (o angoli di Euler)
    :return: Lista con gli angoli di giunto o None se non trovato
    """
    ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)

    # Configurazione iniziale dei giunti (puoi usare una configurazione più intelligente)
    q_init = PyKDL.JntArray(chain.getNrOfJoints())
    # Qui potresti usare una configurazione predefinita (ad esempio, posizione di riposo)
    q_init = initialize_q_init(chain)

    # Definisce la posizione finale desiderata
    end_effector_frame = PyKDL.Frame()
    end_effector_frame.p = PyKDL.Vector(*target_pos)

    # Se è fornito un orientamento, usiamo la matrice di rotazione
    if target_rot:
        if isinstance(target_rot, PyKDL.Rotation):
            end_effector_frame.M = target_rot  # Matrice di rotazione
        else:
            # Se l'orientamento è in formato angoli di Euler (roll, pitch, yaw)
            roll, pitch, yaw = target_rot
            rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)  # Crea la matrice di rotazione
            end_effector_frame.M = rotation

    # Soluzione per i giunti
    joint_result = PyKDL.JntArray(chain.getNrOfJoints())

    # Risolvi la cinematica inversa
    if ik_solver.CartToJnt(q_init, end_effector_frame, joint_result) >= 0:
        return [joint_result[i] for i in range(chain.getNrOfJoints())]
    else:
        return None  # Nessuna soluzione trovata

def initialize_q_init(chain):
    """
    Funzione per inizializzare la configurazione dei giunti. Puoi personalizzare questa logica.
    :param chain: Catena cinematica del robot
    :return: JntArray inizializzato
    """
    q_init = PyKDL.JntArray(chain.getNrOfJoints())
    for i in range(chain.getNrOfJoints()):
        q_init[i] = 0.0  # Configurazione di riposo (o altra configurazione iniziale)
    return q_init
