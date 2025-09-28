#!/usr/bin/env python3
import os
import time
import numpy as np
import rclpy
from .main_rl_environment import MyRLEnvironmentNode
from my_environment_pkg.models.sac_agent import SAC
from my_environment_pkg.utils.model_saver import save_agent, save_replay_buffer
from my_environment_pkg.buffers.her_replay_buffer import HERReplayBuffer
# importa il tuo agente SAC / utilità (adatta il path)
# from my_sac import SAC, save_agent, save_replay_buffer

# --- MOCK imports (sostituisci con i tuoi moduli reali) ---
# Qui sotto è solo un esempio; sostituisci con il vero SAC implementation


# -------------------------------------------------------
def scale_action(normalized_action, low, high):
		"""
		Map normalized_action in [-1,1] to [low, high] for each joint.
		"""
		a = low + (0.5 * (normalized_action + 1.0) * (high - low))
		return a



def main(args=None):
    rclpy.init(args=args)
    env_node = MyRLEnvironmentNode()

    # Spin iniziale per avviare il nodo
    rclpy.spin_once(env_node, timeout_sec=0.01)

    # Ottieni uno stato di esempio per dimensioni
    sample_state = env_node.state_space_funct()
    #state_dim = sample_state['observation'].shape[0] + sample_state['desired_goal'].shape[0]
    action_dim = len(env_node.generate_action_funct())
    state_dim = 12


    # Device
    import torch
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Inizializza SAC
    sac = SAC(state_dim, action_dim, device=device)

    # Hyperparametri
    max_episodes = 300
    episode_horizon = 50
    batch_size = 256
    num_random_episodes = 1
    save_dir = "checkpoints"
    os.makedirs(save_dir, exist_ok=True)

    success_history = []
    #angoli limite 
    joint_mins = np.array([
        -3.14159,   # j1
        -0.57595,   # j2
        -2.51327,   # j3
        -3.14159,   # j4
        -3.14159,   # j5
        -3.14159    # j6
    ], dtype=np.float32)

    joint_maxs = np.array([
        3.14159,    # j1
        0.57595,    # j2
        2.51327,    # j3
        3.14159,    # j4
        3.14159,    # j5
        3.14159     # j6
    ], dtype=np.float32)

    try:
        for episode in range(max_episodes):
            env_node.reset_environment_request()
            #time.sleep(1.0)
            rclpy.spin_once(env_node, timeout_sec=0.01)

            obs = env_node.state_space_funct()
            episode_reward = 0.0
            trajectory = []

            random_policy = episode < num_random_episodes

            for step in range(episode_horizon):
                rclpy.spin_once(env_node, timeout_sec=0.01)

                if random_policy:
                    action = env_node.generate_action_funct()
                else:
                    # Concatena observation + desired_goal per passarlo al SAC
                    state_vec = np.concatenate([obs['observation'], obs['desired_goal']])
                    action = sac.select_action(state_vec)
                    action = action.round(decimals=3)
                    norm_action = np.clip(action, -1.0, 1.0)  # scala se necessario
                    action = scale_action(norm_action, joint_mins, joint_maxs)

                action = [float(a) for a in action]
                env_node.action_step_service(action)
                time.sleep(0.1)
                rclpy.spin_once(env_node, timeout_sec=0.01)

                reward, done = env_node.calculate_reward_funct()
                next_obs = env_node.state_space_funct()

                trajectory.append((obs, action, reward, next_obs, done))
                obs = next_obs
                episode_reward += reward

                if done:
                    print(f"[Episode {episode+1}] Goal reached at step {step+1}")
                    break

            # Salva la traiettoria nel replay buffer (HER incluso)
            sac.replay_buffer.store_trajectory(trajectory)

            # Aggiorna SAC se abbiamo abbastanza dati
            if len(sac.replay_buffer) > batch_size:
                for _ in range(episode_horizon):
                    sac.update(batch_size)

            # Calcola success (norma tra achieved e desired goal)
            success = np.linalg.norm(
                obs['achieved_goal'] - obs['desired_goal']
            ) < 0.05
            success_history.append(1.0 if success else 0.0)
            if len(success_history) > 100:
                success_history.pop(0)
            succ_rate = np.mean(success_history)

            print(f"Episode {episode+1} | Reward: {episode_reward:.3f} | Success: {success} | SuccRate(last100): {succ_rate:.2f}")

            # Salvataggio periodico
            if (episode + 1) % 100 == 0:
                agent_path = os.path.join(save_dir, f"sac_agent_ep{episode+1}.pth")
                buffer_path = os.path.join(save_dir, f"replay_buffer_ep{episode+1}.pkl")
                print(buffer_path)
                save_agent(sac, agent_path)
                save_replay_buffer(sac.replay_buffer, buffer_path)
                #print(f"Saved agent and buffer at episode {episode+1}")

    except KeyboardInterrupt:
        print("Training interrupted by user.")
    finally:
        rclpy.shutdown()
        print("Exited cleanly.")

if __name__ == "__main__":
    main()