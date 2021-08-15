#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
from functions import J_from_eul, ssa, skew, current_to_body

class DPControlSystem():
    '''Dynamic-positioning control system based on linear SISO models and a pole placement algorithm'''
    def __init__(self, M, D, gvect, omega_b, zeta):
        self.M = M
        self.D = D
        self.gvect = gvect
        self.integral_terms = np.zeros(6)
        # Initialise the SISO controller
        K_P = np.zeros((6,6))
        K_D = np.zeros((6,6))
        K_I = np.zeros((6,6))
        k = [0, 0, 0, 0, 0, 0] # Placeholder. Find spring coefficient for roll and pitch if it's desired to regulate these.
        for i in range(6):
            K_P[i][i], K_D[i][i], K_I[i][i] = pid_pole_placement_algorithm(M[i][i], D[i][i], k[i], omega_b[i], zeta[i])
        self.controller = PIDController(K_P, K_D, K_I)

    def pid_regulate(self, eta, nu, eta_d, nu_d, dot_nu_d, t, dot_eta_c=[0,0,0,0,0,0]):
        eta_error = np.array(eta) - np.array(eta_d)
        J = J_from_eul([0, 0, eta[5]])
        eta_error_body = np.dot(J.T, eta_error)
        J_target = J_from_eul([0, 0, eta[5]-eta_d[5]])
        nu_d = np.dot(J_target.T, nu_d)
        nu_error = np.array(nu)-np.array(nu_d)
        '''
        nu_c = np.dot(J.T, dot_eta_c)
        dot_v_c = np.dot(-skew([0, 0, nu[5]]), nu_c[:3])
        dot_nu_c = np.hstack([dot_v_c, [0, 0, 0]])
        '''
        nu_c, dot_nu_c = current_to_body(eta, nu, dot_eta_c)
        tau_rff = np.zeros(6)
        tau_cff = np.zeros(6)
        for i in range(6):
            if i >= 3:
                eta_error_body[i] = ssa(eta_error_body[i])
            tau_rff[i] = self.M[i][i]*dot_nu_d[i] + self.D[i][i]*nu_d[i] + self.gvect[i]
            tau_cff[i] = - self.M[i][i]*dot_nu_c[i] - self.D[i][i]*nu_c[i]
        tau_pid, integral_terms = self.controller.pid_regulate(eta_error_body, nu_error, t)
        self.integral_terms = -np.array(integral_terms) + tau_cff
        return tau_cff + tau_rff + tau_pid

    def pd_regulate(self, eta, nu, eta_d, nu_d, dot_nu_d, dot_eta_c=[0,0,0,0,0,0]):
        eta_error = np.array(eta) - np.array(eta_d)
        J = J_from_eul([0, 0, eta[5]])
        eta_error_body = np.dot(J.T, eta_error)
        J_target = J_from_eul([0, 0, eta[5]-eta_d[5]])
        nu_d = np.dot(J_target.T, nu_d)
        nu_error = np.array(nu)-np.array(nu_d)
        '''
        nu_c = np.dot(J.T, dot_eta_c)
        dot_v_c = np.dot(-skew([0, 0, nu[5]]), nu_c[:3])
        dot_nu_c = np.hstack([dot_v_c, [0, 0, 0]])
        '''
        nu_c, dot_nu_c = current_to_body(eta, nu, dot_eta_c)
        tau_ff = np.zeros(6)
        for i in range(6):
            if i >= 3:
                eta_error_body[i] = ssa(eta_error_body[i])
            tau_ff[i] = self.M[i][i]*(dot_nu_d[i]-dot_nu_c[i]) + self.D[i][i]*(nu_d[i]-nu_c[i]) + self.gvect[i]
        tau_pd = self.controller.pd_regulate(eta_error_body, nu_error)
        return tau_ff + tau_pd

class PIDController:
    """Multidimensional PID controller"""
    def __init__(self, K_P, K_D, K_I, u_max=np.inf):
        """Initialize the PID controller

        Args:
        K_P	(list)   Proportional gain
        K_I	(list)   Integral gain
        K_D	(list)   Derivative gain
        u_max (list) Upper and lower output limit
        """
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.u_max = u_max
        self.prev_t = 0
        self.integral = np.zeros(len(K_P))

    def initialize(self, t):
        self.prev_t = t
        self.integral = 0

    def pid_regulate(self, x_err, dot_x_err, t):
        dt = t - self.prev_t
        if self.prev_t > 0.0 and dt > 0.0:
            self.integral += np.array(x_err)*dt
        u_unsat = - (np.dot(self.K_P, x_err) + np.dot(self.K_D, dot_x_err) + np.dot(self.K_I, self.integral))
        self.prev_t = t
        u = np.zeros(len(u_unsat))
        for i in range(len(u_unsat)):
            if abs(u_unsat[i]) > self.u_max:
                # Anti-wind-up
                u[i] = np.sign(u_unsat[i]) * self.u_max
                self.integral[i] += - (dt/self.K_I[i][i]) * (u[i] - u_unsat[i])
            else:
                u[i] = u_unsat[i]
        return u, (np.dot(self.K_I, self.integral))

    def pd_regulate(self, x_err, dot_x_err):
        return -(np.dot(self.K_P, x_err) + np.dot(self.K_D, dot_x_err))

def pid_pole_placement_algorithm(m, d, k, omega_b, zeta):
    omega_n = omega_b/np.sqrt(1 - 2*zeta**2+np.sqrt(4*zeta**4 - 4*zeta**2 + 2))
    k_p = m*omega_n**2 - k
    k_d = 2*zeta*omega_n*m - d
    k_i = omega_n*k_p/7
    return k_p, k_d, k_i










