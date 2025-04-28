import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

TARGET_TOPIC = "/target_point"
EE_TOPIC = "/ee_pose"
ACTION_TOPIC = "/arm_action"

class ReachEnv(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()

        # Azione: 6 valori continui (posizione giunti) ----PROVARE NP.PI/2
        self.action_space = gym.spaces.Box(low=-np.pi, high=np.pi, shape=(6,), dtype=np.float32)

        # Osservazione: posizione EE (3) + target (3)
        self.observation_space = gym.spaces.Box(low=-10, high=10, shape=(6,), dtype=np.float32)

        rclpy.init()
        self.node = rclpy.create_node("rl_env_node")

        self.ee_pose = np.zeros(3)
        self.target_pose = np.zeros(3)

        # Subscribers
        self.node.create_subscription(PoseStamped, EE_TOPIC, self.ee_cb, 10)
        self.node.create_subscription(PoseStamped, TARGET_TOPIC, self.target_cb, 10)

        # Publisher
        self.pub_action = self.node.create_publisher(Float64MultiArray, ACTION_TOPIC, 10)

    def ee_cb(self, msg):
        self.ee_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def target_cb(self, msg):
        self.target_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def get_obs(self):
        return np.concatenate([self.ee_pose, self.target_pose]).astype(np.float32)

    def compute_reward(self):
        dist = np.linalg.norm(self.ee_pose - self.target_pose)
        return -dist  # Ricompensa negativa proporzionale alla distanza

    def step(self, action):
        # Pubblica l'azione
        msg = Float64MultiArray()
        #msg.data = list(action)
        msg.data = [float(x) for x in action]
        self.pub_action.publish(msg)

        # Aspetta aggiornamenti da ROS
        rclpy.spin_once(self.node, timeout_sec=0.1)

        obs = self.get_obs()
        reward = self.compute_reward()
        done = reward > -0.05  # Se è abbastanza vicino

        return obs, reward, done, False, {}

    def reset(self, seed=None, options=None):
        # Reset (puoi pubblicare un reset se hai un nodo apposito)
        super().reset(seed=seed)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        return self.get_obs(), {}

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
