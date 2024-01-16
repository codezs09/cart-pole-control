#!/usr/bin/env python3

import sys
import os
import numpy as np
import argparse

from utils.utils import load_json, load_data, save_data
from vis.vis_data import vis_data
from model.model import Model
from control.nmpc import NMPC
from proto.proto_gen.data_pb2 import Data, Frame

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
RESULT_DIR = os.path.join(SCRIPT_DIR, "result")
MODEL_PARAM_PATH = os.path.join(SCRIPT_DIR, "config/model_param.json")
SUPER_PARAM_PATH = os.path.join(SCRIPT_DIR, "config/super_param.json")
CONTROL_PARAM_PATH = os.path.join(SCRIPT_DIR, "config/control_param.json")

super_param = load_json(SUPER_PARAM_PATH)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vis_only', action='store_true', 
                        help='skip simulation and visualize the results only.')
    parser.add_argument('-s', '--second', type=float, default = -1.0,
                        help='Show result at the given second.'
                        'Result will not be shown if s is out of simulation time range.')
    parser.add_argument('--save_gif', action='store_true',
                        help='Save the animation to a gif file.')
    args = parser.parse_args()
    return args

def simu(args):
    ctrl_dt = super_param["ctrl_time_step"]    # [s], time step of ctrl ZOH
    target = [super_param["target"]["x"], super_param["target"]["theta"]] # [x, theta]
    # initial state
    state = [super_param["init_state"]["x"], super_param["init_state"]["dx"], \
                super_param["init_state"]["theta"], super_param["init_state"]["dtheta"]] # [x, x_dot, theta, theta_dot]
    force = super_param["init_state"]["force"]  # [N]

    time = 0.0
    model = Model(MODEL_PARAM_PATH)
    nmpc = NMPC(SUPER_PARAM_PATH, CONTROL_PARAM_PATH)
    model.set_initial_state(state)
    data_msg = Data()    
    for i in range(100):
        nmpc.control(state, target, force)
        
        new_frame = nmpc.get_frame_msg()
        new_frame.id = i    # set frame_id and time
        new_frame.time = time
        data_msg.frames.add().CopyFrom(new_frame)

        # update
        force = new_frame.force
        state = model.step(force, ctrl_dt)
        time += ctrl_dt

        print("frame_id: ", new_frame.id, ", time: ", new_frame.time, ", force: ", new_frame.force, ", state: ", state)
    save_data(data_msg, os.path.join(RESULT_DIR, "data.bin"))

def vis(args):
    data_msg = load_data(os.path.join(RESULT_DIR, "data.bin"))
    ctrl_dt = super_param["ctrl_time_step"]
    vis_data(data_msg, ctrl_dt)
    if args.save_gif:
        vis_data(data_msg, ctrl_dt, RESULT_DIR, args.save_gif)
    
    # plot data at args.s if given
    # save animation to gif


def run():
    args = parse_args()
    simu(args)
    vis(args)

if __name__ == "__main__":
    run()
