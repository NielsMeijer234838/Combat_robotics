from gym_wrapper import TwoWheeledRobotEnv

# Create the environment
env = TwoWheeledRobotEnv(render=True)

# Reset the environment
env.reset()

# Print the observation space
print(f'Observation space: {env.observation_space}')

# Print the action space
print(f'Action space: {env.action_space}')

# Simulate the environment for 100 steps
for _ in range(100):
    action = [-0.5, -0.5]
    observation, reward, terminated, truncated, info = env.step(action)
    print(f'Observation: {observation}, Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}')
    if terminated or truncated:
        print('Episode terminated or truncated')
        env.reset()
