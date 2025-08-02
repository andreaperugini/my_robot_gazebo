from .gazebo_reach_env_her import GazeboReachHEREnv
from stable_baselines3 import SAC
from stable_baselines3.her.her_replay_buffer import HerReplayBuffer


def main():
    env = GazeboReachHEREnv()

    model = SAC(
        "MultiInputPolicy",
        env,
        replay_buffer_class=HerReplayBuffer,
        replay_buffer_kwargs=dict(
            n_sampled_goal=4,
            goal_selection_strategy="future",
        ),
        learning_starts=64,  # Deve essere > max_episode_length
        verbose=1,
        buffer_size=1000000,
        batch_size=8,
        gamma=0.95,
        tau=0.05,
    )


    model.learn(total_timesteps=400)
    model.save("sac_her_gazebo")


if __name__ == '__main__':
    main()
