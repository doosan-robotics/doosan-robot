#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import random
import math
verbose = False

class My_RRTStar:
    """
    Class for RRT Star planning
    """

    class Node:
        def __init__(self, x, orientation=None):
            self.x = x  # Posizione [x, y, z] o angoli dei giunti, uso solo angoli dei giunti
            self.orientation = orientation if orientation else [0, 0, 0]  # [roll, pitch, yaw] non uso orientation
            self.parent = None
            self.cost = 0.0


    def __init__(self,
                 start,
                 goal,
                 robot, 
                 obstacle_list,
                 rand_area,
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


    def planning(self, search_until_max_iter=False):
        """
        rrt star path planning

        Inizializza la lista dei nodi con il nodo iniziale.
        Genera nodi casuali nell'area di campionamento.
        Trova il nodo più vicino nella lista dei nodi.
        Tenta di connettere il nuovo nodo (steer).
        Verifica la collisione con gli ostacoli.
        Trova i nodi vicini per il rewire (find_near_nodes).
        Sceglie il miglior nodo genitore (choose_parent).
        Ottimizza la connessione dei nodi (rewire).
        Se il nodo raggiunge la destinazione, genera il percorso.

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
    

    # Seleziona il miglior nodo genitore per il nuovo nodo basandosi sul costo del percorso.
    # Aggiunge una penalità se il nodo è vicino a un ostacolo (obstacle_proximity_penalty).
    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        costs = []
        # di tutti i nodi vicini cerca di collegare il nodo vicino al nuovo nodo9
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
        #selezione miglior nodo genitore
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("Nessun percorso valido: tutti i nodi attraversano ostacoli.")
            return None
        #creazione del nuovo nodo e fa il collegamento 
        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node
        
        # Se il nodo è troppo vicino a un ostacolo, viene penalizzato con un costo aggiuntivo.
        # Il costo aumenta quanto più il nodo si avvicina agli ostacoli.
    def obstacle_proximity_penalty(self, node):
        """
        Aumenta il costo del percorso se è vicino agli ostacoli
        """
        penalty = 0.0
        safe_distance = 0.2  # Distanza minima di sicurezza

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
    
    #Questa funzione restituisce gli indici dei nodi vicini a new_node nella lista self.node_list.
    #Questi nodi vicini sono entro un certo raggio r, che dipende dal numero totale di nodi e dal parametro connect_circle_dist.
    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # hasattr verifca se c'è un attributo
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
            #Viene verificato se il nuovo percorso (edge_node.cost) è più economico rispetto
            #  al percorso precedente che porta al nodo vicino (near_node.cost). 
            improved_cost = near_node.cost > edge_node.cost
            # Se il percorso è valido (non ci sono collisioni) e il costo è migliorato,
            #  il nodo vicino (near_node) viene
            # sostituito con il nuovo nodo di steering (edge_node). 
            # Inoltre, viene propagato il costo aggiornato ai nodi discendenti dal nodo 
            # appena modificato, propagate_cost_to_leaves
            if no_collision and improved_cost:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(new_node)
    #Questa funzione calcola il costo di un percorso tra due nodi.
    def calc_new_cost(self, from_node, to_node):
        d, _, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d
    # Questa funzione aggiorna ricorsivamente i costi di tutti i nodi discendenti di un nodo dato (parent_node). 
    # Dopo aver aggiornato il costo di un nodo,
    #  la funzione viene chiamata ricorsivamente per aggiornare i costi dei
    #  suoi "figli" (nodi che dipendono da questo nodo).
    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    #genera il percorso a ritroso
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
        new_node = self.Node(list(from_node.x), list(from_node.orientation))  # Crea una copia del nodo di partenza
        d, phi, theta = self.calc_distance_and_angle(new_node, to_node)  # Calcola la distanza e gli angoli

        new_node.path_x = [list(new_node.x)]  # Lista che contiene la posizione iniziale del nuovo nodo
        new_node.path_orientation = [list(new_node.orientation)]  # Lista che contiene l'orientamento iniziale del nuovo nodo

        if extend_length > d:  # Se la lunghezza di estensione è maggiore della distanza, limita la lunghezza
            extend_length = d  # Estendi fino alla destinazione

        # Calcola il numero di passi necessari per coprire la distanza con la risoluzione del percorso
        n_expand = max(2, math.floor(extend_length / self.path_resolution))  # Almeno 2 passi

        start, end = np.array(from_node.x), np.array(to_node.x)  # Posizione del nodo di partenza e di destinazione
        v = end - start  # Vettore direzionale tra il nodo di partenza e quello di destinazione
        u = v / (np.linalg.norm(v) + 1e-6)  # Normalizza il vettore direzionale per ottenere una direzione unitaria
        #in modo che abbia una lunghezza unitaria, garantendo che ogni passo venga compiuto alla stessa velocità:
        # Espande il nodo lungo il vettore direzionale
        for _ in range(n_expand):
            new_node.x += u * self.path_resolution  # Avanza lungo il percorso
            new_node.orientation += (np.array(to_node.orientation) - np.array(from_node.orientation)) / n_expand  # Aggiorna l'orientamento
            new_node.path_x.append(list(new_node.x))  # Aggiungi la nuova posizione alla lista dei percorsi
            new_node.path_orientation.append(list(new_node.orientation))  # Aggiungi il nuovo orientamento alla lista

        # Verifica se la distanza tra il nuovo nodo e la destinazione è sufficiente
        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(list(to_node.x))  # Aggiungi la destinazione al percorso
            new_node.path_orientation.append(list(to_node.orientation))  # Aggiungi l'orientamento finale al percorso

        new_node.parent = from_node  # Imposta il nodo di partenza come genitore del nuovo nodo
        return new_node  # Restituisci il nuovo nodo



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

    

    # @staticmethod
    # def check_collision(node, robot, obstacleList, min_samples=30, max_samples=80):
    #     if node is None:
    #         return False
    #     for obstacle in obstacleList:
    #         shape = obstacle[0]
    #         for x in node.path_x:
    #             x_list, y_list, z_list = robot.get_points(x)
    #             for i in range(len(x_list) - 1):
    #                 p1 = np.array([x_list[i], y_list[i], z_list[i]])
    #                 p2 = np.array([x_list[i + 1], y_list[i + 1], z_list[i + 1]])
    #                 # Il numero di campioni è proporzionale alla lunghezza del segmento
    #                 segment_length = np.linalg.norm(p2 - p1)
    #                 num_samples = max(min_samples, min(max_samples, int(segment_length / 0.02)))  
    #                 sampled_points = np.linspace(p1, p2, num_samples)
    #                 for p in sampled_points:
    #                     if shape == "sfera":
    #                         ox, oy, oz = obstacle[1]
    #                         radius = obstacle[2]
    #                         if np.linalg.norm(p - np.array([ox, oy, oz])) <= radius:
    #                             return False  # Collisione con una sfera
    #                     elif shape == "parallelepipedo":
    #                         cx, cy, cz = obstacle[1]
    #                         lx, ly, lz = obstacle[2]
    #                         if (abs(p[0] - cx) <= lx / 2 and
    #                             abs(p[1] - cy) <= ly / 2 and
    #                             abs(p[2] - cz) <= lz / 2):
    #                             return False  # Collisione con un parallelepipedo

    #     return True  # Nessuna collisione
    
    #controllo con i volumi
    @staticmethod
    def check_collision(node, robot, obstacleList, interpolation_steps=5, sphere_radius=0.15):
        """
        Verifica le collisioni lungo il percorso del robot utilizzando sfere interpolate lungo ogni link.
        
        :param node: Nodo contenente il percorso (node.path_x), dove per ogni configurazione viene
                    utilizzato robot.get_points(config) per ottenere i giunti e la base.
        :param robot: Oggetto robot che possiede il metodo get_points, il quale restituisce le posizioni
                    dei giunti (inclusa la base) per una data configurazione.
        :param obstacleList: Lista degli ostacoli. Ogni ostacolo è una tupla:
                            - ('sfera', (ox, oy, oz), radius)
                            - ('parallelepipedo', (cx, cy, cz), (lx, ly, lz))
        :param interpolation_steps: Numero di punti interpolati per ogni link (oltre agli estremi).
        :param sphere_radius: Raggio delle sfere che approssimano il volume del robot.
        :return: True se il percorso è libero da collisioni, False se viene rilevata una collisione.
        """
        if node is None:
            return False

        # Per ogni configurazione (ad esempio, lungo un percorso pianificato con RRT*)
        for config in node.path_x:
            # Ottieni le posizioni (giunti e base) per la configurazione corrente
            x_list, y_list, z_list = zip(*robot.get_joint_positions(config))
            points = [np.array([x, y, z]) for x, y, z in zip(x_list, y_list, z_list)]
            
            # Per ogni segmento (link) tra due punti consecutivi
            for i in range(len(points) - 1):
                p_start = points[i]
                p_end = points[i + 1]
                # Interpolazione lungo il segmento (includendo inizio e fine)
                for t in np.linspace(0, 1, interpolation_steps + 2):
                    center = p_start + t * (p_end - p_start)
                    # Per ogni ostacolo, esegui il controllo con la sfera di raggio sphere_radius
                    for obstacle in obstacleList:
                        shape = obstacle[0]
                        
                        if shape == "sfera":
                            # Controllo sfera-sfera:
                            # Collisione se la distanza tra i centri è minore o uguale alla somma dei raggi.
                            ox, oy, oz = obstacle[1]
                            obs_radius = obstacle[2]
                            if np.linalg.norm(center - np.array([ox, oy, oz])) <= (sphere_radius + obs_radius):
                                return False  # Collisione rilevata

                        elif shape == "parallelepipedo":
                            # Controllo sfera-parallelepipedo:
                            # Calcola il punto del box più vicino al centro della sfera
                            cx, cy, cz = obstacle[1]
                            lx, ly, lz = obstacle[2]
                            min_box = np.array([cx - lx/2, cy - ly/2, cz - lz/2])
                            max_box = np.array([cx + lx/2, cy + ly/2, cz + lz/2])
                            closest_point = np.maximum(min_box, np.minimum(center, max_box))
                            # Se la distanza dal centro della sfera al punto più vicino è inferiore al raggio della sfera,
                            # c'è collisione.
                            if np.linalg.norm(center - closest_point) <= sphere_radius:
                                return False  # Collisione rilevata

        return True  # Nessuna collisione lungo tutto il percorso
