
import os
import sys
import ctypes
import numpy as np
from scipy.linalg import expm

from proto.proto_gen.data_pb2 import Frame
from controller.control_api import ControlAPI
from utils.utils import load_json
from utils.c2d import lsys_c2d

pip3_control_path = "/home/deeproute/.local/lib/python3.8/site-packages"
sys.path.insert(0, pip3_control_path)
import control  # control system library

"""
    LQR class
"""
class LQR(ControlAPI):
    def __init__(self, super_param_path, control_param_path):
        self.super_param_ = load_json(super_param_path)
        self.control_param_ = load_json(control_param_path)
        self.frame_msg_ = Frame()
        
        lqr_cfg = self.control_param_["lqr_cfg"]
        if lqr_cfg["use_finite_lqr"]:
            print(f"Finite LQR used (DARE solver): hp={lqr_cfg['hp']}, dt={lqr_cfg['lqr_dt']} !")
        else:
            print("Infinite LQR used !")

    def control(self, state, target, last_control):
        dt = self.super_param_["ctrl_time_step"]
        u_lqr = self._lqr_control(state, target, last_control)
        
        u = u_lqr
        # rate limit
        u_step_limit = dt * self.control_param_["force_rate_limit"]
        u = max(last_control - u_step_limit, u)
        u = min(last_control + u_step_limit, u)

        # saturation limit
        u_limit = self.control_param_["force_limit"]
        u = max(-u_limit, u)
        u = min(u_limit, u)

        # update frame_msg_
        self.frame_msg_.x = state[0]
        self.frame_msg_.dx = state[1]
        self.frame_msg_.theta = state[2]
        self.frame_msg_.dtheta = state[3]
        self.frame_msg_.force = u
        self.frame_msg_.status = True

        return u

    def get_frame_msg(self):
        return self.frame_msg_
    
    def _lqr_control(self, state, target, last_control):
        model_param = self.control_param_['model_param']
        M = model_param['mass_cart']
        m = model_param['mass_pole']
        b = model_param['damping_cart']
        l = model_param['length_com_pole']
        I = model_param['inertia_pole']
        g = model_param['gravity']

        p = (M + m) * (I + m * l**2) - (m * l)**2
        Ac = np.array([[0.0, 1.0,                0.0,        0.0], \
                      [0.0, -(I + m*l*l)*b/p,   m*m*l*l*g/p,  0.0], \
                      [0.0, 0.0,                0.0,        1.0], \
                      [0.0, m*b*l/p,            m*g*l*(M+m)/p, 0.0] \
                     ])
        Bc = np.array([[0.0], \
                      [(I + m*l*l)/p], \
                      [0.0], \
                      [-m*l/p] \
                     ])

        lqr_cfg = self.control_param_["lqr_cfg"]
        Qx = lqr_cfg["Qx"]
        Qtheta = lqr_cfg["Qtheta"]
        R_u = lqr_cfg["R_u"]
        R_du = lqr_cfg["R_du"]

        Q = np.diag([Qx, 0.0, Qtheta, 0.0])
        R = np.diag([R_u])
        x = np.array([[state[0] - target[0]],
                    [state[1]],
                    [state[2] - target[1]],
                    [state[3]]])

        if lqr_cfg["use_finite_lqr"]:
            u = self._finite_lqr_solver(x, Ac, Bc, Q, R, R_du, last_control, \
                                   lqr_cfg["hp"], lqr_cfg["lqr_dt"], target)
        else:
            u = self._infinite_lqr_solver(x, Ac, Bc, Q, R, R_du, last_control, \
                                          lqr_cfg["use_dare"], lqr_cfg["lqr_dt"])
        return u

    def _infinite_lqr_solver(self, x, Ac, Bc, Q, R, R_du, last_control, use_dare=False, dt=None):
        if dt is None:
            use_dare = False    # override
        msg = "Solve Infinite Horizon LQR problem ... "
        msg += " DARE solver used ... " if use_dare else " CARE solver used ... "
        print(msg)

        is_lqr_aug = True  # NOTE: if True, use Augmented LQR with control rate penality
        if abs(R_du) < 1.0e-6:
            is_lqr_aug = False # avoid solver fail due to singularity
        
        if not is_lqr_aug:
            u = self._ARE_solver_inf(x, Ac, Bc, Q, R, use_dare, dt)
        else:
            # Augmented with state [x, u], control [du]
            Ac_aug = np.block([[Ac, Bc], \
                                [np.zeros((1, 5))]])
            Bc_aug = np.array([[0.0], \
                                [0.0], \
                                [0.0], \
                                [0.0], \
                                [1.0]])
            Q_aug = np.block([[Q, np.zeros((4, 1))], \
                            [np.zeros((1, 4)), R]])
            R_aug = np.diag([R_du])

            x_aug = np.block([[x], \
                              [last_control]])
            u_aug = self._ARE_solver_inf(x_aug, Ac_aug, Bc_aug, Q_aug, R_aug, use_dare, dt)
            u = last_control + u_aug * self.super_param_["ctrl_time_step"]
        
        # TODO: may add cost calculation here for inifite horizon LQR

        return u
    
    def _finite_lqr_solver(self, x, Ac, Bc, Q, R, R_du, last_control, hp, dt, target):
        Ad, Bd = lsys_c2d(Ac, Bc, dt)

        is_lqr_aug = True if abs(R_du) > 1.0e-6 else False
        if not is_lqr_aug:
            u_list, x_list = self._DARE_solver_finite(x, Ad, Bd, Q, R, hp)
        else:
            # Augmented with state [x, u], control [du]
            Ad_aug = np.block([[Ad, Bd], \
                                [np.zeros((1, 4)), 1.0]])
            Bd_aug = np.array([[0.0], \
                                [0.0], \
                                [0.0], \
                                [0.0], \
                                [dt]])
            Q_aug = np.block([[Q, np.zeros((4, 1))], \
                            [np.zeros((1, 4)), R]])
            R_aug = np.diag([R_du])
            x_aug = np.block([[x], \
                              [last_control]])
            u_aug_list, x_aug_list = self._DARE_solver_finite(x_aug, Ad_aug, Bd_aug, Q_aug, R_aug, hp)
            
            u_list = []     # k = 0, 1, ..., hp-1
            x_list = []     # [x - x_r, dx, theta - theta_r, ], k = 1, 2, ..., hp
            for i in range(hp):
                u_list.append(x_aug_list[i][4][0])
                x_list.append(x_aug_list[i][:4])

        # fill the horizon
        self.frame_msg_.ClearField("mpc_horizon")
        mpc_horizon = self.frame_msg_.mpc_horizon
        t = 0.0
        assert len(u_list) == hp, "Error: length of u_list should be hp!"
        mpc_horizon.t.append(t)
        mpc_horizon.x.append(x[0,0] + target[0])
        mpc_horizon.dx.append(x[1,0])
        mpc_horizon.theta.append(x[2,0] + target[1])
        mpc_horizon.dtheta.append(x[3,0])
        mpc_horizon.force.append(u_list[0])
        for i in range(hp):
            t += dt
            mpc_horizon.t.append(t)
            mpc_horizon.x.append(x_list[i][0][0] + target[0])
            mpc_horizon.dx.append(x_list[i][1][0])
            mpc_horizon.theta.append(x_list[i][2][0] + target[1])
            mpc_horizon.dtheta.append(x_list[i][3][0])
            if i < hp-1:
                mpc_horizon.force.append(u_list[i+1])
            else:
                mpc_horizon.force.append(u_list[-1])

        # calculate costs
        try:
            self._calculate_costs(x_list, u_list, u_aug_list)
        except NameError:
            self._calculate_costs(x_list, u_list)

        return u_list[0]

    """
        ARE solver for Infinite Horizon
    """
    def _ARE_solver_inf(self, x0, Ac, Bc, Q, R, use_dare=False, dt=None):
        if dt is None:
            use_dare = False    # override

        if use_dare:
            Ad, Bd = lsys_c2d(Ac, Bc, dt)
            u = self._DARE_solver_inf(x0, Ad, Bd, Q, R)
        else:
            u = self._CARE_solver_inf(x0, Ac, Bc, Q, R)
        return u

    """
        CARE solver for Infinite Horizon
    """
    def _CARE_solver_inf(self, x0, Ac, Bc, Q, R):
        K, S, E = control.lqr(Ac, Bc, Q, R)
        u = -np.matmul(K, x0)
        u = u.item()
        return u

    """
        DARE solver for Infinite Horizon
    """
    def _DARE_solver_inf(self, x0, Ad, Bd, Q, R):
        P = Q
        for i in range(999999):
            P_next = Ad.T @ P @ Ad - Ad.T @ P @ Bd @ np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad + Q
            if np.all(np.abs(P_next - P) < 1.0e-6):
                print(f"Converged \t {i}")
                break   # converge
            P = P_next
            if (i == 999999):
                print("Error: DARE solver not converged!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ")
        F = np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad

        # debug: compare with control.dlqr
        if False:
            K, S, E = control.dlqr(A, B, Q, R)
            print("K: ", S)
            print("F: ", F)
        u = -F @ x0
        u = u.item()
        return u

    """
        DARE solver for Finite Horizon hp
    """
    def _DARE_solver_finite(self, x0, Ad, Bd, Q, R, hp):
        F_list = []
        P = Q
        for i in range(hp): 
            P = Ad.T @ P @ Ad - Ad.T @ P @ Bd @ np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad + Q
            F = np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad    # gain
            F_list.append(F)
        F_list.reverse()

        # Forward propagation
        x_list = [] # k = 1, 2, ..., hp
        u_list = [] # k = 0, 1, ..., hp-1
        x = x0
        for i in range(hp):
            u = -F_list[i] @ x
            x = Ad @ x + Bd @ u
            u_list.append(u)
            x_list.append(x)
        return u_list, x_list
    
    def _calculate_costs(self, x_list, u_list, du_list=None):
        lqr_cfg = self.control_param_["lqr_cfg"]
        Qx = lqr_cfg["Qx"]
        Qtheta = lqr_cfg["Qtheta"]
        R_u = lqr_cfg["R_u"]
        R_du = lqr_cfg["R_du"]

        cost_x = 0.0
        cost_theta = 0.0
        cost_u = 0.0
        cost_du = 0.0
        for i in range(len(x_list)):
            x = x_list[i]
            u = u_list[i]
            x0 = x[0][0]
            x2 = x[2][0]
            cost_x += x0 * Qx * x0
            cost_theta += x2 * Qtheta * x2
            cost_u += u * R_u * u
            if du_list is not None:
                du = du_list[i].item()
                cost_du += du * R_du * du
        cost_total = cost_x + cost_theta + cost_u + cost_du

        self.frame_msg_.ClearField("costs")
        costs = self.frame_msg_.costs
        costs.cost_total = cost_total
        costs.cost_x = cost_x
        costs.cost_theta = cost_theta
        costs.cost_u = cost_u
        costs.cost_du = cost_du
