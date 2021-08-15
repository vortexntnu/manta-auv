#!/usr/bin/env python
# Written by Aksel Kristoffersen
# Documentation can be found in my master's thesis, chapter __ and __

import numpy as np
import pandas as pd
import quadprog
from functions import R_from_eul, T_from_eul
import yaml

class ControlAllocationSystem:
    def __init__(self, thruster_positions, thruster_orientations, rotor_time_constant, u_max, u_min, w):
        self.input_matrix = compute_input_matrix(thruster_positions, thruster_orientations)
        self.rotor_time_constant = rotor_time_constant
        self.u_max = np.array(u_max)
        self.u_min = np.array(u_min)
        self.pseudo_inv_input_matrix = moore_penrose_pseudo_inverse(self.input_matrix)
        W = np.diag(w)
        self.P = np.block([[W*1.000001, -W], [-W, W*1.000001]]) # Small cheat to make P positive definite
        self.c = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.A = np.block([[self.pseudo_inv_input_matrix, -self.pseudo_inv_input_matrix], [-self.pseudo_inv_input_matrix, self.pseudo_inv_input_matrix]])
        self.tau = np.array([0, 0, 0, 0, 0, 0])
    
    def compute_control_inputs(self, tau_c):
        return np.dot(self.pseudo_inv_input_matrix, tau_c)
    
    def program_feasible_control_forces(self, tau_unb):
        u_unb = np.dot(self.pseudo_inv_input_matrix, tau_unb)
        if any([x>self.u_max or x<self.u_min for x in u_unb]):
            # If any unfeasible control inputs, generate feasible control forces using QP
            b = np.block([self.u_max - u_unb, -self.u_min + u_unb])
            x = quadprog_solve_qp(self.P, self.c, self.A, b)
            tau_b = tau_unb + x[0:6] - x[6:12]
            if np.linalg.norm(tau_b) > 40:
                tau_b = self.tau
        else:
            tau_b = tau_unb
        self.tau = tau_b
        '''
        tau_b[3] = 0 # Ensure no actuation in roll or pitch
        tau_b[4] = 0
        '''
        return tau_b

    def compute_optimized_control_inputs(self, tau_unb):
        tau = self.program_feasible_control_forces(tau_unb)
        u = self.compute_control_inputs(tau)
        return u


def moore_penrose_pseudo_inverse(matrix):
    return np.dot(np.array(matrix).T, np.linalg.inv(np.dot(np.array(matrix), np.array(matrix).T)))

def compute_input_matrix(thruster_positions, thruster_orientations, location_frame='ENU', output_frame='NED'):
    r = len(thruster_positions)
    input_matrix = np.zeros((6, r))
    F_p = np.array([1, 0, 0])
    if location_frame != output_frame:
        eul_offset = [np.pi, 0, 0]
    else:
        eul_offset = [0, 0, 0]
    R_offset = R_from_eul(eul_offset)
    T_offset = T_from_eul(eul_offset)
    for i in range(r):
        p = np.dot(R_offset, thruster_positions[i,:])
        eul = np.dot(T_offset, thruster_orientations[i,:])
        R = R_from_eul(eul)
        F_b = np.dot(R, F_p)
        input_matrix[:3,i] = F_b
        input_matrix[3:,i] = np.cross(p, F_b)
    return input_matrix

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -numpy.vstack([A, G]).T
        qp_b = -numpy.hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(qp_G.astype(np.float), qp_a.astype(np.float), qp_C.astype(np.float), qp_b.astype(np.float), meq)[0]

if __name__ == '__main__':
    with open('testing/beluga_test.yaml', 'r') as stream:
        data = yaml.safe_load(stream)
    thruster_positions = np.array(data['thrusters']['positions'])
    thruster_orientations = np.array(data['thrusters']['orientations'])
    rotor_time_constant = data['thrusters']['first_order_time_constant']
    W = data['guidance_and_control_system']['control_forces_weights']

    u_max = 10.5
    u_min = -10.5
    control_allocation = ControlAllocationSystem(thruster_positions, thruster_orientations, rotor_time_constant, u_max, u_min, W)
    tau_unb = [50, 20, 10, 0, 0, 10]
    u_unb = control_allocation.compute_control_inputs(tau_unb)
    u_sat = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for i in range(8):
        if u_unb[i] > u_max:
            u_sat[i] = u_max
        if u_unb[i] < u_min:
            u_sat[i] = u_min
    tau = np.dot(control_allocation.input_matrix, u_sat)
    tau_b = control_allocation.program_feasible_control_forces(tau_unb)
    u_b = control_allocation.compute_control_inputs(tau_b)

    data_tau = {'tau_unb': tau_unb, 'tau': tau, 'tau_b': tau_b}
    data_u = {'u_unb': u_unb, 'u_b': u_b, 'u_sat': u_sat}

    df1 = pd.DataFrame(data_tau)
    df2 = pd.DataFrame(data_u)

    df1.to_csv('matlab/tau.csv')
    df2.to_csv('matlab/u.csv')
    

    



