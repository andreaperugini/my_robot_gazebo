import time
from env_2 import ReachEnv
from stable_baselines3 import PPO

# Carica il modello addestrato (se lo hai salvato)
model = PPO.load("PPO_model_50k")

# Ambiente di test
env = ReachEnv()

# Numero di episodi di test
num_episodes = 100

# Ciclo di test
for episode in range(num_episodes):
    obs, _ = env.reset()  # Resetta l'ambiente all'inizio di ogni episodio
    done = False
    total_reward = 0
    while not done:
        # Seleziona un'azione usando la politica del modello
        action, _ = model.predict(obs)
        
        # Esegui l'azione nell'ambiente
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        
        # Aggiungi la ricompensa accumulata
        total_reward += reward
        
        # Pausa per vedere il movimento in Gazebo
        time.sleep(0.5)  # Pausa di 0.1 secondi tra ogni passo per visualizzare il movimento
    
    print(f"Episodio {episode + 1} ---",env.get_obs(),"---: Ricompensa totale = {total_reward}")
