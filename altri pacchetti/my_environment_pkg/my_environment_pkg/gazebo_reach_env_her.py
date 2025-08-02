import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from .main_rl_environment import MyRLEnvironmentNode

class GazeboReachHEREnv(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()

        rclpy.init()
        self.node = MyRLEnvironmentNode("policy_model.pth")
        self.current_step = 0
        self.max_steps = 4

        # Attendi che i dati siano pronti
        while self.node.state_space_funct() is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.obs_dim = 12  # 3 effector + 6 joints + 3 target
        self.goal_dim = 3  # posizione del goal

        self.observation_space = spaces.Dict({
            "observation":    spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32),
            "achieved_goal":  spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "desired_goal":   spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
        })

        self.action_space = spaces.Box(
            low=np.array([-3.14, -0.57, -2.51, -3.14, -3.14, -3.14]),
            high=np.array([3.14, 0.57, 2.51, 3.14, 3.14, 3.14]),
            dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        self.current_step = 0
        self.node.reset_environment_request()
        while self.node.state_space_funct() is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._get_obs(), {}

    def step(self, action):
        self.current_step += 1

        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.node.action_step_service(action.tolist())
        reward, success = self.node.calculate_reward_funct()

        # success = True quando raggiunge il goal → terminated
        terminated = success
        truncated = self.current_step >= self.max_steps

        while self.node.state_space_funct() is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        obs = self._get_obs()
        info = {"TimeLimit.truncated": truncated}

        return obs, reward, terminated, truncated, info

    def _get_obs(self):
        full_obs = self.node.state_space_funct()
        if full_obs is None:
            return None

        observation = full_obs
        achieved_goal = np.array([self.node.robot_x, self.node.robot_y, self.node.robot_z], dtype=np.float32)
        desired_goal = np.array([self.node.pos_sphere_x, self.node.pos_sphere_y, self.node.pos_sphere_z], dtype=np.float32)

        return {
            "observation": observation,
            "achieved_goal": achieved_goal,
            "desired_goal": desired_goal
        }

    def compute_reward(self, achieved_goal, desired_goal, info):
        distance = np.linalg.norm(achieved_goal - desired_goal)
        reward = 1.0 - distance
        reward *= 5.0
        if distance <= 0.05:
            reward += 10.0
        return reward

    def close(self):
        rclpy.shutdown()
