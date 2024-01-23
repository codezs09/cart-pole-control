
import os
import sys
import ctypes
import numpy as np

from proto.proto_gen.data_pb2 import Frame
from controller.control_api import ControlAPI
from utils.utils import load_json

pip3_control_path = "/home/deeproute/.local/lib/python3.8/site-packages"
sys.path.insert(0, pip3_control_path)
import control  # control system library

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
        u_lqr = self._lqr_solve(state, target, last_control)
        
        dt = self.super_param_["ctrl_time_step"]
        # update costs
        x_diff = state[0] - target[0]
        theta_diff = state[2] - target[1]
        du_lqr = (u_lqr - last_control) / dt
        lqr_cfg = self.control_param_["lqr_cfg"]
        cost_x = x_diff * lqr_cfg["Qx"] * x_diff
        cost_theta = theta_diff * lqr_cfg["Qtheta"] * theta_diff
        cost_u = u_lqr * lqr_cfg["R_u"] * u_lqr
        cost_du = du_lqr * lqr_cfg["R_du"] * du_lqr
        cost_total = cost_x + cost_theta + cost_u + cost_du

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
        self.frame_msg_.costs.cost_total = cost_total
        self.frame_msg_.costs.cost_x = cost_x
        self.frame_msg_.costs.cost_theta = cost_theta
        self.frame_msg_.costs.cost_u = cost_u
        self.frame_msg_.costs.cost_du = cost_du

        self.frame_msg_.status = True

        return u

    def get_frame_msg(self):
        return self.frame_msg_
    
    def _lqr_solve(self, state, target, last_control):
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
                                   lqr_cfg["hp"], lqr_cfg["lqr_dt"])
        else:
            u = self._infinite_lqr_solver(x, Ac, Bc, Q, R, R_du, last_control)
        return u

    def _infinite_lqr_solver(self, x, Ac, Bc, Q, R, R_du, last_control):
        is_lqr_aug = True  # NOTE: if True, use Augmented LQR with control rate penality
        if abs(R_du) < 1.0e-6:
            is_lqr_aug = False # avoid solver fail due to singularity
        
        if not is_lqr_aug:
            K, S, E = control.lqr(Ac, Bc, Q, R)
            u = -np.matmul(K, x)
            u = u.item()
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

            K, S, E = control.lqr(Ac_aug, Bc_aug, Q_aug, R_aug)
            x_aug = np.block([[x], \
                              [last_control]])
            u_aug = -np.matmul(K, x_aug)
            u_agug = u_aug.item()
            u = last_control + u_agug * self.super_param_["ctrl_time_step"]
        return u
    
    def _finite_lqr_solver(self, x, Ac, Bc, Q, R, R_du, last_control, hp, dt):
        return -1
    
