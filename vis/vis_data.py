import os
import gymnasium as gym
import imageio
import numpy as np
import time
import numpy as np
import matplotlib.pyplot as plt
from math import pi
from utils.utils import make_gif

def vis_data(data_msg, dt, save_dir="", save_gif=False):
    # Create the cart-pole environment
    if save_gif:
        env = gym.make('CartPole-v1', render_mode="rgb_array")
    else:
        env = gym.make('CartPole-v1', render_mode="human")

    env.reset()
    frames = []  # List to store frames
    data_size = len(data_msg.frames)
    for i in range(data_size):
        frame_data = data_msg.frames[i]
        cart_pole_state = [frame_data.x, frame_data.dx, frame_data.theta, frame_data.dtheta]
        env.unwrapped.state = np.array(cart_pole_state)

        # env.render()
        frames.append(env.render())
        if not save_gif:
            time.sleep(dt)
    if not save_gif:
        time.sleep(1)
    env.close()

    if save_gif:
        imageio.mimsave(os.path.join(save_dir, 'cartpole.gif'), \
                        frames, \
                        duration=dt*1000,
                        loop=0)

def plot_data_frames(data_msg, super_param, ctrl_dt, result_dir):
    # set visualization time step
    vis_dt = 0.1    # [s]
    vis_gap = int(vis_dt / ctrl_dt)

    frames_dir = os.path.join(result_dir, "frames")
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)
    # remove all files in frames_dir
    for f in os.listdir(frames_dir):
        os.remove(os.path.join(frames_dir, f))

    x_ref = super_param["target"]["x"]
    theta_ref = super_param["target"]["theta"]

    # plot
    data_frames = data_msg.frames
    t_past = []
    x_past = []
    dx_past = []
    theta_past = []
    dtheta_past = []
    force_past = []
    cost_total_past = []
    cost_x_past = []
    cost_theta_past = []
    cost_u_past = []
    cost_du_past = []
    status_past = []

    plt.figure(figsize=(12, 8))
    for i, frame in enumerate(data_frames):
        # update data
        t_past.append(frame.time)
        x_past.append(frame.x)
        dx_past.append(frame.dx)
        theta_past.append(frame.theta)
        dtheta_past.append(frame.dtheta)
        force_past.append(frame.force)
        cost_total_past.append(frame.costs.cost_total)
        cost_x_past.append(frame.costs.cost_x)
        cost_theta_past.append(frame.costs.cost_theta)
        cost_u_past.append(frame.costs.cost_u)
        cost_du_past.append(frame.costs.cost_du)
        status_past.append(frame.status)

        t_hp = [frame.time + t for t in frame.mpc_horizon.t]
        x_hp = frame.mpc_horizon.x
        dx_hp = frame.mpc_horizon.dx
        theta_hp = frame.mpc_horizon.theta
        dtheta_hp = frame.mpc_horizon.dtheta
        force_hp = frame.mpc_horizon.force

        if i % vis_gap != 0:
            continue    # skip if not visualization time step

        plt.clf()
        plt.suptitle(f"Time = {frame.time:.2f} [s]")
        ax1 = plt.subplot(331)
        plt.plot(t_past, x_past, 'b-', label='past')
        if len(t_hp) > 0:
            plt.plot(t_hp, x_hp, 'rx', label='hp')
            plt.plot([t_hp[0], t_hp[-1]], [x_ref, x_ref], 'g--', label='target')
        plt.xlabel('time [s]')
        plt.ylabel('x [m]')
        plt.grid(True)
        plt.legend()

        plt.subplot(332,sharex=ax1)
        plt.plot(t_past, np.array(theta_past)*180.0/pi, 'b-', label='past')
        if len(t_hp) > 0:
            plt.plot(t_hp, np.array(theta_hp)*180.0/pi, 'rx', label='hp')
            plt.plot([t_hp[0], t_hp[-1]], [theta_ref*180.0/pi for i in range(2)], 'g--', label='target')
        plt.xlabel('time [s]')
        plt.ylabel('theta [deg]')
        plt.grid(True)
        plt.legend()

        plt.subplot(333,sharex=ax1)
        plt.plot(t_past, status_past, 'b-', label='past')
        plt.xlabel('time [s]')
        plt.ylabel('Status')
        plt.grid(True)
        
        plt.subplot(334,sharex=ax1)
        plt.plot(t_past, dx_past, 'b-', label='past')
        if len(t_hp) > 0:
            plt.plot(t_hp, dx_hp, 'rx', label='hp')
        plt.xlabel('time [s]')
        plt.ylabel('x_dot [m/s]')
        plt.grid(True)
        plt.legend()

        plt.subplot(335,sharex=ax1)
        plt.plot(t_past, np.array(dtheta_past)*180.0/pi, 'b-', label='past')
        if len(t_hp) > 0:
            plt.plot(t_hp, np.array(dtheta_hp)*180.0/pi, 'rx', label='hp')
        plt.xlabel('time [s]')
        plt.ylabel('theta_dot [deg/s]')
        plt.grid(True)
        plt.legend()

        plt.subplot(336,sharex=ax1)
        plt.plot(t_past, cost_total_past, 'b-', label='total')
        plt.plot(t_past, cost_x_past, label='x')
        plt.plot(t_past, cost_theta_past, label='theta')
        plt.plot(t_past, cost_u_past, label='u')
        plt.plot(t_past, cost_du_past, label='du')
        plt.xlabel('time [s]')
        plt.ylabel('costs')
        plt.grid(True)
        plt.legend()

        plt.subplot(337,sharex=ax1)
        plt.plot(t_past, force_past, 'b-', label='past')
        if len(t_hp) > 0:
            plt.plot(t_hp, force_hp, 'rx', label='hp')
        plt.xlabel('time [s]')
        plt.ylabel('force [N]')
        plt.grid(True)
        plt.legend()
        
        plt.savefig(os.path.join(frames_dir, f"{i}.png"))

    make_gif(frames_dir, os.path.join(result_dir, "plots.gif"), vis_dt)