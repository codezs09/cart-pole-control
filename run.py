#!/usr/bin/env python3

import sys
import os
import numpy as np
import argparse

from data_pb2 import Data, Frame

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vis_only', action='store_true', 
                        help='skip simulation and visualize the results only.')
    parser.add_argument('-s', '--second', type=float, default = -1.0,
                        help='Show result at the given second.'
                        'Result will not be shown if s is out of simulation time range.')

    args = parser.parse_args()
    return args

def simu(args): 
    dt = 0.01   # [s], time step
    time = 0.0

    # load super params
    target = [0.2, 0.1] # [x, theta]

    # initial state
    state = [0.0, 0.0, 0.0, 0.0] # [x, x_dot, theta, theta_dot]
    model = Model() 
    model.init(state)
    data_msg = Data()
    for i in range(1000):
        # call nmpc to get the optimal control
        force = nmpc.control(state, target)     # the control method calls ctypes c++ ipopt solver
        
        # add to data_msg
        new_frame = Frame()
        new_frame.id = i
        new_frame.time = time
        new_frame.x = state[0]
        new_frame.dx = state[1]
        new_frame.theta = state[2]
        new_frame.dtheta = state[3]
        new_frame.force = force
        new_frame.horizon.t = nmpc.horizon().t 
        new_frame.horizon.x = nmpc.horizon().x
        new_frame.horizon.dx = nmpc.horizon().dx
        new_frame.horizon.theta = nmpc.horizon().theta
        new_frame.horizon.dtheta = nmpc.horizon().dtheta
        new_frame.horizon.force = nmpc.horizon().force
        data_msg.frames.add().CopyFrom(new_frame)

        # update
        state = model.step(force, dt)
        time += dt
    
    # save data to protobuf
    save_data(data_msg)

def vis(args):
    # load protobuf data
    # generate animation
    # plot data at args.s if given
    # save animation to gif
    pass

def run():
    args = parse_args()
    simu(args)
    vis(args)

if __name__ == "__main__":
    run()
