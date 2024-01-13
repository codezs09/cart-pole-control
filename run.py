#!/usr/bin/env python3

import sys
import os
import numpy as np
import argparse

from utils.json_utils import load_json
from model.model import Model
from control.nmpc import NMPC
from proto.proto_gen.data_pb2 import Data, Frame

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MODEL_PARAM_PATH = os.path.join(SCRIPT_DIR, "config/model_param.json")
SUPER_PARAM_PATH = os.path.join(SCRIPT_DIR, "config/super_param.json")
CONTROL_PARAM_PATH = os.path.join(SCRIPT_DIR, "config/control_param.json")

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
    ctrl_dt = 0.01   # [s], time step of ctrl ZOH
    time = 0.0

    target = [0.2, 0.1] # [x, theta]

    # initial state
    state = [0.0, 0.0, 0.0, 0.0] # [x, x_dot, theta, theta_dot]
    force = 0.0
    model = Model(MODEL_PARAM_PATH)
    nmpc = NMPC(SUPER_PARAM_PATH, CONTROL_PARAM_PATH)
    model.set_initial_state(state)
    data_msg = Data()
    for i in range(1000):
        nmpc.control(state, target, force)
        
        new_frame = nmpc.get_frame_msg()
        new_frame.id = i    # set frame_id and time
        new_frame.time = time
        data_msg.frames.add().CopyFrom(new_frame)

        # update
        force = new_frame.force
        state = model.step(force, ctrl_dt)
        time += ctrl_dt

    # # save data to protobuf
    # save_data(data_msg)

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
