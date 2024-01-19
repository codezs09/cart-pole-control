
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

        print("sys.path", sys.path)
        K, S, E = control.lqr(Ac, Bc, Q, R)

        x = np.array([[state[0] - target[0]],
                      [state[1]],
                      [state[2] - target[1]],
                      [state[3]]])
        u = -np.matmul(K, x)
        return u.item()
    
