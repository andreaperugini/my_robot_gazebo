import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from .main_rl_environment import MyRLEnvironmentNode

class GazeboReachEnv(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()

        rclpy.init()  # ROS2 init
        self.node = MyRLEnvironmentNode("policy_model.pth")

        # Attendi che l’ambiente si inizializzi (state != None)
        while self.node.state_space_funct() is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        obs_dim = self.node.state_space_funct().shape[0]
        act_dim = 6  # 6 joint angles

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-3.14, -0.57, -2.51, -3.14, -3.14, -3.14]),
                                       high=np.array([3.14, 0.57, 2.51, 3.14, 3.14, 3.14]),
                                       dtype=np.float32)

    def reset(self, seed=None, options=None):
        self.node.reset_environment_request()
        while self.node.state_space_funct() is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        obs = self.node.state_space_funct()
        return obs, {}

    def step(self, action):
        # Clip action for safety
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.node.action_step_service(action.tolist())
        reward, done = self.node.calculate_reward_funct()

        while self.node.state_space_funct() is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        obs = self.node.state_space_funct()
        return obs, reward, done, False, {}

    def close(self):
        rclpy.shutdown()
