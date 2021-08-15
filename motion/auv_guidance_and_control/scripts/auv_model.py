#!/usr/bin/env python
# Written by Aksel Kristoffersen
# Documentation can be found in my master's thesis, chapter __ 

import numpy as np
from functions import R_from_eul

class AUVModel:
    def __init__(self, m, r_g, r_b, inertia, volume, M_A, D, rho=997.0, g=9.81):
        self.m = m
        self.r_g = r_g
        self.r_b = r_b
        self.inertia = inertia
        self.volume = volume
        self.rho = rho
        self.g = g
        self.M_RB = get_M_RB(self.m, self.r_g, self.inertia)
        self.M = np.diag(np.diag(self.M_RB + M_A))
        self.D = np.array(D)
        self.gvect = self.get_gvect([0, 0, 0])

    def get_gvect(self, eul):
        gvect = np.zeros(6)
        W = self.m*self.g
        B = self.rho*self.g*self.volume
        R = R_from_eul(eul)
        f_g = np.dot(R.T, [0, 0, W])
        f_b = np.dot(R.T, [0, 0, -B])
        gvect[0:3] = f_g + f_b
        gvect[3:] = np.cross(self.r_g, f_g) + np.cross(self.r_b, f_b)
        return -gvect

def get_M_RB(m, r_g, inertia):
    M_11 = m*np.eye(3)
    M_12 = -m*skew(r_g)
    M_21 = m*skew(r_g)
    M_22 = np.array(inertia) - m*skew(r_g)**2
    return np.block([[M_11, M_12], [M_21, M_22]])

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

