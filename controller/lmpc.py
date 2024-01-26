import os
import sys
import ctypes
import numpy as np
from scipy.linalg import expm
import scipy.sparse as sp
import osqp

from proto.proto_gen.data_pb2 import Frame
from controller.control_api import ControlAPI
from utils.utils import load_json
from utils.c2d import lsys_c2d

"""
    LMPC: Linear Model Predictive Control
"""
class LMPC(ControlAPI):
    def __init__(self, super_param_path, control_param_path):
        self.super_param_ = load_json(super_param_path)
        self.control_param_ = load_json(control_param_path)
        self.frame_msg_ = Frame()
        
        lmpc_cfg = self.control_param_["lmpc_cfg"]
        if lmpc_cfg["is_unconstrained"]:
            s = "LMPC (Unconstrained, Analytical Solver): "
        else:
            s = "LMPC (Constrained, QP Solver): "
        s += f"hp={lmpc_cfg['hp']}, hc={lmpc_cfg['hc']}, dt={lmpc_cfg['mpc_dt']} !"
        print(s)

    def control(self, state, target, last_control):
        dt = self.super_param_["ctrl_time_step"]
        u_lmpc = self._lmpc_control(state, target, last_control)
        
        u = u_lmpc
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
    
    def _lmpc_control(self, state, target, last_control):
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
        
        lmpc_cfg = self.control_param_["lmpc_cfg"]
        Qx = lmpc_cfg["Qx"]
        Qtheta = lmpc_cfg["Qtheta"]
        R_u = lmpc_cfg["R_u"]
        R_du = lmpc_cfg["R_du"]
        dt = lmpc_cfg["mpc_dt"]
        hp = lmpc_cfg["hp"]
        hc = lmpc_cfg["hc"]
        use_qp = not lmpc_cfg["is_unconstrained"]

        Q = np.diag([Qx, 0.0, Qtheta, 0.0])
        R = np.diag([R_u])
        x0 = np.array([[state[0] - target[0]],
                    [state[1]],
                    [state[2] - target[1]],
                    [state[3]]])
        
        # hard constraint bounds
        u_ub = self.control_param_["force_limit"]
        u_lb = -u_ub
        du_ub = self.control_param_["force_rate_limit"]
        du_lb = -du_ub
        # h_lb <= h_ineq @ x <= h_ub
        h_ineq = np.array([[0.0, 0.0, 1.0, 0.0]])
        h_ub = self.control_param_["theta_limit"]
        h_lb = -h_ub

        # discretize
        Ad, Bd = lsys_c2d(Ac, Bc, dt)

        # QP problem formulation (with/without augment state)
        is_lmpc_aug = True if abs(R_du) > 1.0e-6 else False
        if not is_lmpc_aug:
            A_tilde, B_tilde, Q_tilde, R_tilde = self._state_matrix_reorg(Ad, Bd, Q, R, Q, hp, hc)
            H_tilde, H_tilde_lb, H_tilde_ub = self._constraint_matrix_reorg(A_tilde, B_tilde, x0, \
                                                    h_ineq, h_lb, h_ub, u_lb, u_ub, hp, hc) # no constraint on du

            u_list, x_list = self._lmpc_solver(A_tilde, B_tilde, Q_tilde, R_tilde, x0, hp, hc, use_qp, \
                                               H_tilde, H_tilde_lb, H_tilde_ub)
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
            x0_aug = np.block([[x0], \
                              [last_control]])
            A_tilde_aug, B_tilde_aug, Q_tilde_aug, R_tilde_aug = \
                self._state_matrix_reorg(Ad_aug, Bd_aug, Q_aug, R_aug, Q_aug, hp, hc)
            H_tilde_aug, H_tilde_lb_aug, H_tilde_ub_aug = self._constraint_matrix_reorg_aug(
                                                        A_tilde_aug, B_tilde_aug, x0_aug, \
                                                        h_ineq, h_lb, h_ub, u_lb, u_ub, du_lb, du_ub, hp, hc)
            
            u_aug_list, x_aug_list = self._lmpc_solver(A_tilde_aug, B_tilde_aug, \
                                                       Q_tilde_aug, R_tilde_aug, \
                                                        x0_aug, hp, hc, use_qp, \
                                                        H_tilde_aug, H_tilde_lb_aug, H_tilde_ub_aug)
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
        mpc_horizon.x.append(x0[0,0] + target[0])
        mpc_horizon.dx.append(x0[1,0])
        mpc_horizon.theta.append(x0[2,0] + target[1])
        mpc_horizon.dtheta.append(x0[3,0])
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
        Reorganize the STATE matrices to represent Quadratic problem
    """
    def _state_matrix_reorg(self, Ad, Bd, Q, R, Qf, hp, hc):
        n = Ad.shape[0] # state dimension
        m = Bd.shape[1] # control dimension
        
        A_tilde = np.zeros((n*hp, n))
        A_tmp = Ad.copy()
        for i in range(hp):
            A_tilde[i*n:(i+1)*n, :] = A_tmp
            A_tmp = A_tmp @ Ad
        
        B_tilde = np.zeros((n*hp, m*hc))
        B_row_tmp = np.zeros((n, m*hc))
        B_row_tmp[:, 0:m] = Bd.copy()
        for i in range(hp):
            B_tilde[i*n:(i+1)*n, :] = B_row_tmp.copy()
            B_row_tmp = Ad @ B_row_tmp
            j = min(i+1, hc-1)
            B_row_tmp[:, j*m:(j+1)*m] = Bd.copy()

        Q_csc = sp.csc_matrix(Q)
        Qf_csc = sp.csc_matrix(Qf)
        Q_tilde = sp.block_diag([Q_csc if i < hp else Qf_csc for i in range(hp)])

        R_csc = sp.csc_matrix(R)
        R_tilde = sp.block_diag([R_csc if i < hc else (hp-hc+1)*R_csc for i in range(hc)])

        return A_tilde, B_tilde, Q_tilde, R_tilde
    
    """
        Reorganize the CONSTRAINT matrices
    """
    def _constraint_matrix_reorg(self, A_tilde, B_tilde, x0, h_ineq, h_lb, h_ub, u_lb, u_ub, hp, hc):
        U_lb = np.array([[u_lb] for i in range(hc)])
        U_ub = np.array([[u_ub] for i in range(hc)])

        h_tilde_ineq = sp.block_diag([h_ineq for i in range(hp)])
        h_tilde_lb = np.array([[h_lb] for i in range(hp)])
        h_tilde_ub = np.array([[h_ub] for i in range(hp)])

        H_tilde = np.block([[h_tilde_ineq @ B_tilde], [np.eye(hc)]])
        H_tilde_lb = np.block([[h_tilde_lb - h_tilde_ineq @ A_tilde @ x0], [U_lb]])
        H_tilde_ub = np.block([[h_tilde_ub - h_tilde_ineq @ A_tilde @ x0], [U_ub]])

        return H_tilde, H_tilde_lb, H_tilde_ub

    """
        Reorganize the CONSTRAINT matrices (Augmented)
    """
    def _constraint_matrix_reorg_aug(self, A_tilde_aug, B_tilde_aug, x0_aug, h_ineq, h_lb, h_ub, u_lb, u_ub, du_lb, du_ub, hp, hc):
        # augmented with state [x, u], control [du]
        n = x0_aug.shape[0] -1  # state dimension

        # du constraints
        U_lb = np.array([[du_lb] for i in range(hc)])
        U_ub = np.array([[du_ub] for i in range(hc)])

        # h_ineq @ augmented x constraints
        h_ineq_aug = np.block([[h_ineq, np.zeros((1, 1))],
                                [np.zeros((1, n)), 1.0]])
        h_lb_aug = np.block([[h_lb], [u_lb]])
        h_ub_aug = np.block([[h_ub], [u_ub]])
        n_ineq = h_ineq_aug.shape[0]

        h_tilde_ineq = sp.block_diag([h_ineq_aug for i in range(hp)])
        h_tilde_lb = np.zeros((n_ineq*hp, 1))
        h_tilde_ub = np.zeros((n_ineq*hp, 1))
        for i in range(hp):
            h_tilde_lb[i*n_ineq:(i+1)*n_ineq, :] = h_lb_aug
            h_tilde_ub[i*n_ineq:(i+1)*n_ineq, :] = h_ub_aug

        H_tilde_aug = np.block([[h_tilde_ineq @ B_tilde_aug], [np.eye(hc)]])
        H_tilde_lb_aug = np.block([[h_tilde_lb - h_tilde_ineq @ A_tilde_aug @ x0_aug], [U_lb]])
        H_tilde_ub_aug = np.block([[h_tilde_ub - h_tilde_ineq @ A_tilde_aug @ x0_aug], [U_ub]])

        return H_tilde_aug, H_tilde_lb_aug, H_tilde_ub_aug

    """
        LMPC solver
    """
    def _lmpc_solver(self, A_tilde, B_tilde, Q_tilde, R_tilde, x0, hp, hc, use_qp, \
                    H_tilde, H_tilde_lb, H_tilde_ub):
        n = x0.shape[0] # state dimension
        if not use_qp:
            # Analytical solver
            U = self._lmpc_solver_unconstrained(A_tilde, B_tilde, Q_tilde, R_tilde, x0)
        else:
            # use QP solver, return U of length hc
            U = self._lmpc_solver_constrained(A_tilde, B_tilde, Q_tilde, R_tilde, x0, hp, hc, \
                                                    H_tilde, H_tilde_lb, H_tilde_ub)
        X = A_tilde @ x0 + B_tilde @ U  # size (n*hp,1)

        u_list = [] 
        x_list = []
        for i in range(hp):
            u_list.append(U[i][0] if i < hc else U[-1][0])
            x_list.append(X[i*n:(i+1)*n])

        return u_list, x_list
    
    """
        LMPC solver (unconstrained, analytical solution)
    """
    def _lmpc_solver_unconstrained(self, A_tilde, B_tilde, Q_tilde, R_tilde, x0):
        p = B_tilde.T @ Q_tilde @ B_tilde + R_tilde
        q = B_tilde.T @ Q_tilde @ A_tilde @ x0
        U = np.linalg.solve(p, -q)  # length hc
        return U
    
    """
        LMC solver (constrained, QP solution)
    """
    def _lmpc_solver_constrained(self, A_tilde, B_tilde, Q_tilde, R_tilde, x0, hp, hc, \
                                H_tilde, H_tilde_lb, H_tilde_ub):
        P = B_tilde.T @ Q_tilde @ B_tilde + R_tilde
        q = B_tilde.T @ Q_tilde @ A_tilde @ x0
        
        P = sp.csc_matrix(P)
        H_tilde = sp.csc_matrix(H_tilde)
        # OSQP problem formulation: https://osqp.org/docs/index.html
        prob = osqp.OSQP()
        prob.setup(P, q, H_tilde, H_tilde_lb, H_tilde_ub, warm_start=True, verbose=False)
        res = prob.solve()
        
        U = np.array(res.x)
        U = U.reshape((hc, 1))
        return U
    
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