import os
import gymnasium as gym
import imageio
import numpy as np
import time

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
                        duration=dt*1000)
