#!/usr/bin/env python
# Written by Aksel Kristoffersen
# Documentation can be found in my master's thesis, chapter __ and __

import numpy as np
from functions import ssa

class CurrentEstimator:
    def __init__(self, delta_psi, X_u, Y_v):
        self.delta_psi = delta_psi
        self.X_u = X_u
        self.Y_v = Y_v
        self.tol_error = 0.05
        self.tol_sec = 5
        self.data_est = {'U_c': [], 'chi_c': [], 'U_c_mean': [], 'chi_c_mean': [], 'psi': [], 't': []}
        self.online = False
        
    def initailize(self, eta_r, t):
        self.eta_r = eta_r
        self.dot_x_c = 0
        self.dot_y_c = 0
        self.U_c = 0.0
        self.chi_c = 0
        self.i = 0.0
        self.prev_t = t
        self.online = True

    def check_steady_state(self, eta, t):
        if (np.linalg.norm(eta - np.array(self.eta_r)) < self.tol_error) and (t-self.prev_t > self.tol_sec):
            self.prev_t = t
            result = True
        elif np.linalg.norm(eta - np.array(self.eta_r)) < self.tol_error:
            result = False
        else:
            self.prev_t = t
            result = False
        return result
        
    def estimate_current_velocity(self, eta, N, int_x, int_y, t):
        if self.check_steady_state(eta, t):
            if self.i > 0:
                delta_int_x = int_x - self.prev_int_x 
                delta_int_y = int_y - self.prev_int_y
                new_U_c = (1/(2*np.sin(self.delta_psi/2)))*np.sqrt((delta_int_x/(self.X_u))**2 + (delta_int_y/(self.Y_v))**2)
                self.U_c = (self.U_c*(self.i-1) + new_U_c)/self.i
                sign = -np.sign(delta_int_x)*np.sign(self.delta_psi)
                new_chi_c = ssa(sign*np.arccos((delta_int_y/(self.Y_v))/(np.sqrt((delta_int_x/(self.X_u))**2 + (delta_int_y/(self.Y_v))**2))) + self.eta_r[5] - self.delta_psi/2)
                self.chi_c = (self.chi_c*(self.i-1) + new_chi_c)/self.i
            self.prev_int_x = int_x
            self.prev_int_y = int_y
            self.dot_x_c = self.U_c*np.cos(self.chi_c)
            self.dot_y_c = self.U_c*np.sin(self.chi_c)
            if self.i == N:
                self.online = False
            else:
                self.eta_r[5] += self.delta_psi
                self.eta_r[5] = ssa(self.eta_r[5])
            self.i += 1

           

