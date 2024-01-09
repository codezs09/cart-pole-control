
import json
import numpy as np
from math import sin, cos

from rk4step import rk4step

class Model:
    def __init__(self, mdl_param_path):
        with open(mdl_param_path, 'r') as f:
            mdl_param = json.load(f)
            self.mdl_param = mdl_param
        self._state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, x_dot, psi, psi_dot]

    def set_initial_state(self, state):
        self._state =  np.array(state) if not isinstance(state, np.ndarray) else state

    def step(self, force, duration):
        t_step = 1.0e-4
        t = 0.0
        while (t < duration):
            dt = min(t_step, duration - t)
            self._state = rk4step(self._dynamic_function, self._state, dt, \
                                  force, self.mdl_param)
            t += dt
        return self._state

    """
    calculates derivate of state
    Equations and sybmoles follow the following reference: 
    Reference: https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling 
    
    Note: replace theta with: psi = pi - theta
        - x: right direction is positive
        - psi: clockwise is positive, psi = 0 at the upright position
    """
    def _dynamic_function(self, state, *args):
        F = args[0]
        mdl_param = args[1]
        M = mdl_param['mass_cart']
        m = mdl_param['mass_pole']
        b = mdl_param['damping_cart']
        l = mdl_param['length_com_pole']
        I = mdl_param['inertia_pole']
        g = mdl_param['gravity']

        x = state[0]
        dx = state[1]
        psi = state[2]
        dpsi = state[3]

        next_dx = dx
        next_ddx = ((I + m * l**2) * (F - b * dx + m * l * dpsi * dpsi * sin(psi)) - \
                    m * m * l * l * g * sin(psi) * cos(psi)) / \
                     ((I + m * l**2) * (M + m) - (m * l * cos(psi))**2 )
        next_dpsi = dpsi
        next_ddpsi = ((M + m) * m * g * l * sin(psi) + m * l * b * cos(psi) * dx - \
                      (m * l * dpsi) ** 2 * sin(psi) * cos(psi) - F * m * l * cos(psi)) / \
                      ((I + m * l**2) * (M + m) - (m * l * cos(psi))**2 )

        return np.array([next_dx, next_ddx, next_dpsi, next_ddpsi])
    

"""
For test purpose. To test the script, run "python3 model.py".
"""
if __name__=="__main__":
    import os

    ENABLE_VIS = True
    if ENABLE_VIS:
        import time
        import gymnasium as gym
        env = gym.make('CartPole-v1', render_mode="human")
        env.reset()

    def test():
        dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        mdl_param_path = os.path.join(dir, "config/model_param.json")
        model = Model(mdl_param_path)
        state = np.array([-1.0, 0.0, 0.0, 0.0])
        model.set_initial_state(state)
        force = 0.05    # [N]
        dt = 0.1
        i = 0
        t = 0.0
        while t < 1.0:
            if ENABLE_VIS:
                env.unwrapped.state = state
                env.render()
                time.sleep(dt)

            state = model.step(force, dt)
            print("#", i, ", t=", t, "[s], state: ", state)
            i += 1
            t += dt

        if ENABLE_VIS:
            time.sleep(1)
            env.close()

    test()