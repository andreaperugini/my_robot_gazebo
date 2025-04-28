from env_2 import ReachEnv

env = ReachEnv()

from stable_baselines3 import PPO, DDPG


# Funzione che riceve il progresso dell'allenamento (tra 0 e 1) e restituisce il learning rate
def linear_schedule(initial_value):
    def func(progress_remaining):
        return progress_remaining * initial_value  # decresce linearmente
    return func

# Usa la funzione come learning_rate
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=linear_schedule(1e-3),  # esempio: parte da 1e-4 e cala linearmente
    verbose=1
)

model.learn(total_timesteps=50_000)
model.save("PPO_prova_50k")
