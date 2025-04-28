import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ContactsState  # msg per collisioni

TARGET_TOPIC = "/target_point"
EE_TOPIC    = "/ee_pose"
ACTION_TOPIC= "/arm_action"
COLLISION_TOPIC = "/collision_states"   # es: plugin contacts

class ReachEnv(gym.Env):
    def __init__(self, render_mode="human"):
        super().__init__()

        # Azione: 6 valori continui (posizione giunti)
        self.action_space = gym.spaces.Box(
        low=np.array([-np.pi, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/4], dtype=np.float32),
        high=np.array([np.pi, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/4], dtype=np.float32)
)

        # Osservazione: posizione EE (3) + target (3)
        self.observation_space = gym.spaces.Box(
            low=-2, high=2, shape=(6,), dtype=np.float32)

        # ROS2 setup
        rclpy.init()
        self.node = rclpy.create_node("rl_env_node")

        # stato interno
        self.ee_pose       = np.zeros(3)
        self.target_pose   = np.zeros(3)
        self.previous_action = np.zeros(6)
        self.collision     = False

        # Subs / pubs
        self.node.create_subscription(PoseStamped, EE_TOPIC,     self.ee_cb,       10)
        self.node.create_subscription(PoseStamped, TARGET_TOPIC, self.target_cb, 10)
        self.node.create_subscription(ContactsState, COLLISION_TOPIC, self.collision_cb, 10)
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

    def collision_cb(self, msg: ContactsState):
        # se c'è almeno un contatto, segnala collisione
        self.collision = len(msg.states) > 0

    def get_obs(self):
        return np.concatenate([self.ee_pose, self.target_pose]).astype(np.float32)

    def compute_reward(self, action: np.ndarray):
        # 1) distanza
        dist = np.linalg.norm(self.ee_pose - self.target_pose)
        reward_distance = -dist

        # 2) fluidità
        #delta = np.linalg.norm(action - self.previous_action)
        #reward_smoothness = -0.1 * delta

        # 3) collisioni
        reward_collision = -50.0 if self.collision else 0.0

        return reward_distance + reward_collision #+ reward_smoothness  

    def step(self, action):
        # 1) invio comando
        msg = Float64MultiArray(data=[float(x) for x in action])
        self.pub_action.publish(msg)

        # 2) aggiorna ROS
        #rclpy.spin_once(self.node, timeout_sec=3)

        # 3) osservazione e reward
        obs    = self.get_obs()
        reward = self.compute_reward(action)

        # 4) check done: raggiunto o collisione
        done   = (np.linalg.norm(self.ee_pose - self.target_pose) < 0.05) or self.collision

        # 5) aggiorna stato interno
        self.previous_action = action.copy()

        return obs, reward, done, False, {"distance": np.linalg.norm(self.ee_pose - self.target_pose),
                                          "collision": self.collision}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # se hai un nodo di reset ROS, puoi chiamarlo qui
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # reset interno
        self.previous_action = np.zeros(6)
        self.collision       = False

        return self.get_obs(), {}

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
