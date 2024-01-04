import gymnasium as gym
import time

# Create the cart-pole environment
env = gym.make('CartPole-v1', render_mode="human")

# Set the environment to a specific state
# State vector: [cart position, cart velocity, pole angle, pole velocity at tip]
# For example: [0.0, 0.0, 0.1, 0.0] - Slightly tilted pole
custom_state = [0.0, 0.0, 0.1, 0.0]

env.reset()

for i in range(20):
    print(f"#{i}: state = {custom_state}")

    env.unwrapped.state = custom_state

    # Render the environment
    env.render()

    # Keep the window open for a short time
    time.sleep(0.2)

    # generate a random double within range [a, b]
    # a + (b-a) * random()
    pos_change = 0.02
    pole_change = -0.02

    custom_state[0] += pos_change
    custom_state[2] += pole_change

time.sleep(1)
# Close the environment
env.close()
