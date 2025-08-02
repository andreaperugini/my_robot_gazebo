'''
Author: David Valencia
Date     : 12 / 10 /2021
Modified : 07 /04  /2022


Describer: 

		I use this file to run my main environment easier, the whole body of the environment is in 
		the file main_rl_environmet.py, but here I import that file and run the ROS node more organized

		This file just generates a vector of random action and moves the robot until the number of episodes 
		and the number of steps is completed, nothing more
		
		To use this script I need to launch first:
			my_environment.launch.py

		Executable name of this file in the setup file: run_environment
'''
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import time
import rclpy
from .main_rl_environment import MyRLEnvironmentNode
from .env_policy import PolicyNetwork
import os


# Get path of the current script (run_environment.py)
script_dir = os.path.dirname(os.path.realpath(__file__))
save_path = os.path.join(script_dir, "policy_model.pth")
#sovrascrivo per il momento

save_path = "policy_model.pth"
print(save_path)



def compute_returns(rewards, gamma=0.99):
	returns = []
	R = 0
	for r in reversed(rewards):
		R = r + gamma * R
		returns.insert(0, R)
	return torch.tensor(returns, dtype=torch.float32)

def main(args=None):

	rclpy.init(args=args)
	run_env_node = MyRLEnvironmentNode(save_path)
	rclpy.spin_once(run_env_node)

	num_episodes = 200
	episonde_horizont = 4
	
	#policy
	try:
		print(save_path)
		run_env_node.policy.load_state_dict(torch.load(save_path))
		run_env_node.policy.eval()
		print("carico policy")
	except Exception as e:
		print(e)
		run_env_node.policy = PolicyNetwork()
		print("nuova policy, la vecchia non c'è in "+ save_path)


	optimizer = torch.optim.Adam(run_env_node.policy.parameters(), lr=1e-3)

	for episode in range (num_episodes):

		run_env_node.reset_environment_request()					
		#time.sleep(2.0)
		step = 0
		log_probs = []
		rewards = []

		for step in range (episonde_horizont):
			print (f'----------------Episode:{episode+1} Step:{step+1}--------------------')

			action, log_prob = run_env_node.generate_action_funct_policy(training=True) # generate a sample action vector
			run_env_node.action_step_service(action) # take the action

			reward, done  = run_env_node.calculate_reward_funct()
			state  = run_env_node.state_space_funct()
			rewards.append(reward)
			if log_prob is not None:
				log_probs.append(log_prob)

			if done:
				print(f'Goal reached at step {step+1}, episode ends.')
				break

			#time.sleep(0.5)
			
		# Update policy via REINFORCE
		if log_probs:
			returns = compute_returns(rewards)
			returns = (returns - returns.mean()) / (returns.std() + 1e-5)  # normalize

			loss = -torch.stack(log_probs) * returns
			loss = loss.sum()

			optimizer.zero_grad()
			loss.backward()
			optimizer.step()
		print (f'Episode {episode+1} Ended')
		

	print ("Total num of episode completed, Exiting ....")
	torch.save(run_env_node.policy.state_dict(), save_path)
	rclpy.shutdown()
	

if __name__ == '__main__':
	main()
