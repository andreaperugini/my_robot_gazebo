
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from .main_rl_environment import MyRLEnvironmentNode
import rclpy


class GazeboCustmEnv(gym.Env):
    def __init__(self):
        super().__init__()

        rclpy.init()
        run_env_node =MyRLEnvironmentNode()
        rclpy.spin_once(run_env_node)

def resey(self, seed=None, option=None):
    return obs, {}

def step(self, action):
    return obs, reward, done, False, {}

def get_observation(self):
    return obs

def compute_reward(self, obs):
    return "da vedere"
        
def check_done(self, obs):
    return False