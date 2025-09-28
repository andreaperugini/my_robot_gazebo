import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from .main_rl_environment import MyRLEnvironmentNode  # il file che hai postato

class MyGymEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # Inizializza ROS2
        rclpy.init()
        self.node = MyRLEnvironmentNode()

        # Azione: 6 joint angles
        self.action_space = spaces.Box(low=np.array([-3.14, -0.57, -2.5, -3.14, -3.14, -3.14]),
                                       high=np.array([3.14, 0.57, 2.5, 3.14, 3.14, 3.14]),
                                       dtype=np.float32)

        # Osservazione: posizione effettore, posizioni giunti, posizione bersaglio
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.node.reset_environment_request()
        rclpy.spin_once(self.node, timeout_sec=2.0)

        obs = self.node.state_space_funct()
    
        return obs, {}
    
    def step(self, action):
        self.node.action_step_service(action)
        rclpy.spin_once(self.node, timeout_sec=2.5)

        obs = self.node.state_space_funct()
        reward, done = self.node.calculate_reward_funct()
        info = {}
        
        return obs, reward, done, False, info

    def render(self, mode="human"):
        pass  # opzionale

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
