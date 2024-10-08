#!/usr/bin/env python3

import sys
import os
import numpy as np
import argparse

from utils.utils import load_json, load_data, save_data
from vis.vis_data import vis_data, plot_data_frames
from system.plant import Plant
from controller.nmpc import NMPC
from controller.lqr import LQR
from controller.lmpc import LMPC
from proto.proto_gen.data_pb2 import Data, Frame

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
RESULT_DIR = os.path.join(SCRIPT_DIR, "result")
PLANT_PARAM_PATH = os.path.join(SCRIPT_DIR, "config/plant_param.json")
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

def get_controller():
    ctrl_param = load_json(CONTROL_PARAM_PATH)
    type = ctrl_param["type"]

    if type == "NMPC":
        return NMPC(SUPER_PARAM_PATH, CONTROL_PARAM_PATH)
    elif type == "LMPC":
        return LMPC(SUPER_PARAM_PATH, CONTROL_PARAM_PATH)
    elif type == "LQR":
        return LQR(SUPER_PARAM_PATH, CONTROL_PARAM_PATH)
    else:
        print("Error: invalid controller type arg: ", type)
        return None

def simu(args):
    ctrl_dt = super_param["ctrl_time_step"]    # [s], time step of ctrl ZOH
    simulation_duration = super_param["simulation_duration"]  # [s]
    target = [super_param["target"]["x"], super_param["target"]["theta"]] # [x, theta]
    # initial state
    state = [super_param["init_state"]["x"], super_param["init_state"]["dx"], \
                super_param["init_state"]["theta"], super_param["init_state"]["dtheta"]] # [x, x_dot, theta, theta_dot]
    force = super_param["init_state"]["force"]  # [N]

    time = 0.0
    plant = Plant(PLANT_PARAM_PATH)
    controller = get_controller()
    plant.set_initial_state(state)
    data_msg = Data()
    num_frames = int(simulation_duration / ctrl_dt)
    for i in range(num_frames):
        # Assuming Full-state is known
        # TODO: May add observer to estimate the full-state
        controller.control(state, target, force)
        
        new_frame = controller.get_frame_msg()
        new_frame.id = i    # set frame_id and time
        new_frame.time = time
        data_msg.frames.add().CopyFrom(new_frame)

        # update
        force = new_frame.force
        state = plant.step(force, ctrl_dt)
        time += ctrl_dt

        np.set_printoptions(precision=4)
        print(f"frame_id: {new_frame.id}, \t time: {new_frame.time:.2f}, \t force: {new_frame.force:.4f}, \t state:", state)
    save_data(data_msg, os.path.join(RESULT_DIR, "data.bin"))

def vis(args):
    data_msg = load_data(os.path.join(RESULT_DIR, "data.bin"))
    ctrl_dt = super_param["ctrl_time_step"]
    vis_data(data_msg, ctrl_dt)
    if args.save_gif:
        vis_data(data_msg, ctrl_dt, RESULT_DIR, args.save_gif)
    
    plot_data_frames(data_msg, super_param, ctrl_dt, RESULT_DIR)


def run():
    args = parse_args()
    simu(args)
    vis(args)

if __name__ == "__main__":
    run()
