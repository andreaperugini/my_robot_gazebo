import sys
import os
import time
import numpy as np
import torch
import rclpy

# Importa il nodo del tuo ambiente ROS2
from my_environment_pkg.main_rl_environment import MyRLEnvironmentNode

# Importa SAC e loader
from my_environment_pkg.models.sac_agent import SAC
from my_environment_pkg.utils.model_saver import load_agent


# Limiti reali dei giunti (rad)
JOINT_LIMITS = [
    (-3.14159, 3.14159),    # Joint 1
    (-0.57595, 0.57595),    # Joint 2
    (-2.51327, 2.51327),    # Joint 3
    (-3.14159, 3.14159),    # Joint 4
    (-3.14159, 3.14159),    # Joint 5
    (-3.14159, 3.14159)     # Joint 6
]

def scale_action(norm_action):
    """
    Converte un'azione normalizzata [-1, 1] nei limiti reali dei giunti.
    """
    scaled = []
    for i, a in enumerate(norm_action):
        low, high = JOINT_LIMITS[i]
        scaled.append(low + (a + 1.0) * 0.5 * (high - low))
    return np.array(scaled, dtype=np.float32)

def main():
    rclpy.init()

    # Inizializza nodo ambiente ROS2
    env_node = MyRLEnvironmentNode()
    rclpy.spin_once(env_node)

    # Ottieni dimensioni stato/azione
    sample_state = env_node.state_space_funct()
    #state_dim = len(sample_state['observation']) + len(sample_state['desired_goal'])
    state_dim=12
    action_dim = len(JOINT_LIMITS)

    # Setup dispositivo
    device = "cpu"

    # Percorso modello
    model_path = os.path.expanduser("/home/andrea/checkpoints/sac_agent_ep300.pth")

    # Inizializza SAC
    sac = SAC(state_dim, action_dim, device=device)

    # Carica modello
    sac = load_agent(sac, model_path, device)
    print(f"Loaded model from {model_path}")

    # Parametri test
    num_episodes = 10
    episode_horizon = 50

    for episode in range(num_episodes):
        env_node.reset_environment_request()
        time.sleep(1.0)  # attesa reset
        obs = env_node.state_space_funct()
        episode_reward = 0

        for step in range(episode_horizon):
            rclpy.spin_once(env_node, timeout_sec=0.01)

            # Prepara stato per SAC
            state_vec = np.concatenate([obs['observation'], obs['desired_goal']])

            # Seleziona azione normalizzata
            norm_action = sac.select_action(state_vec)
            norm_action = np.clip(norm_action, -1.0, 1.0)

            # Scala ai limiti reali
            action = scale_action(norm_action)
            action = [float(a) for a in action]

            # Esegui azione
            env_node.action_step_service(action)

            # Ottieni ricompensa e stato
            reward, done = env_node.calculate_reward_funct()
            obs = env_node.state_space_funct()
            episode_reward += reward

            if done:
                print(f"Goal raggiunto in {step+1} step!")
                break

        print(f"[EPISODIO {episode+1}] Reward totale: {episode_reward}")

    print("Test completato")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
