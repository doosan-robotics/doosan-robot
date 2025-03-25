#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# robot_arm.py
import numpy as np
from n_joint_arm_3d.NLinkArm3d import NLinkArm

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
