import PyKDL
import numpy as np
import rospy

class My_RobotArm:
    def __init__(self, chain):
        """
        Inizializza il braccio robotico dato un oggetto KDL Chain.
        :param chain: Catena cinematica del robot
        """
        self.chain = chain
        self.n_joints = chain.getNrOfJoints()  # Numero di giunti
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)  # Risolutore cinematica diretta
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)  # Risolutore cinematica inversa
    
    def forward_kinematics(self, joint_angles):
        """
        Calcola la cinematica diretta (posizione dell'effettore finale) dato un set di angoli dei giunti.
        :param joint_angles: Lista o array di angoli dei giunti in radianti
        :return: Lista di coordinate [x, y, z] della posizione dell'effettore finale
        """
        # Imposta gli angoli dei giunti
        jnt_array = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            jnt_array[i] = joint_angles[i]
        
        # Calcola la cinematica diretta
        ee_frame = PyKDL.Frame()
        if self.fk_solver.JntToCart(jnt_array, ee_frame) < 0:
            rospy.logerr("Errore nella cinematica diretta")
            return None
        
        # Estrai la posizione dell'effettore finale
        position = ee_frame.p
        return [position.x(), position.y(), position.z()]
    
    def inverse_kinematics(self, target_pos, target_rot=None):
        """
        Calcola la cinematica inversa per ottenere gli angoli dei giunti dati una posizione (x, y, z) e opzionalmente un orientamento.
        :param target_pos: Posizione desiderata (x, y, z)
        :param target_rot: (opzionale) Matrice di rotazione desiderata
        :return: Lista degli angoli dei giunti (o None se non trovato)
        """
        # Definisci la configurazione iniziale dei giunti
        q_init = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            q_init[i] = 0.0  # Configurazione di riposo
        
        # Definisci la posizione finale desiderata
        end_effector_frame = PyKDL.Frame()
        end_effector_frame.p = PyKDL.Vector(*target_pos)

        # Se fornito, aggiungi l'orientamento

        if target_rot:
            if isinstance(target_rot, PyKDL.Rotation):
                end_effector_frame.M = target_rot
            else:
                # Quando target_rot è una lista di rotazione [roll, pitch, yaw]
                roll, pitch, yaw = target_rot
                rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
                end_effector_frame.M = rotation

        # Risoluzione cinematica inversa
        joint_result = PyKDL.JntArray(self.n_joints)
        if self.ik_solver.CartToJnt(q_init, end_effector_frame, joint_result) >= 0:
            return [joint_result[i] for i in range(self.n_joints)]
        else:
            return None  # Nessuna soluzione trovata


    def get_joint_positions(self, joint_angles, resolution=3):
        """
        Calcola la posizione di tutti i giunti e dei punti intermedi per rappresentare meglio la struttura del braccio.

        :param joint_angles: Lista di angoli dei giunti
        :param resolution: Numero di punti intermedi tra un giunto e l'altro
        :return: Lista delle coordinate [x, y, z] di giunti e punti intermedi
        """
        jnt_array = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            jnt_array[i] = joint_angles[i]

        trans = PyKDL.Frame.Identity()

        # Lista delle posizioni reali dei giunti
        real_joint_positions = []

        for i in range(self.chain.getNrOfSegments()):
            segment = self.chain.getSegment(i)
            joint = segment.getJoint()

            if joint.getType() != getattr(PyKDL.Joint, "None"):
                self.fk_solver.JntToCart(jnt_array, trans, i + 1)
                real_joint_positions.append([trans.p.x(), trans.p.y(), trans.p.z()])

        return real_joint_positions