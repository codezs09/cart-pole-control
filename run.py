#!/usr/bin/env python3

import numpy as np


def run(): 
    dt = 0.01   # [s], time step
    time = 0.0

    # load super params
    target = [0.2, 0.1] # [x, theta]

    # initial state
    state = [0.0, 0.0, 0.0, 0.0] # [x, x_dot, theta, theta_dot]

    for i in range(1000):
        # call nmpc to get the optimal control
        force = nmpc.control(state, target)     # the control method calls ctypes c++ ipopt solver

        # call model to get the next step
        state = model.step(state, force, dt)

        # save data to protobuf
        


        time += dt




if __name__ == "__main__":
    run()
