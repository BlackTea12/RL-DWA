# Dynamic Window Approach (Local Planning) with moving obstacles
# For Training
# Yaeohn Kim 2022 September

from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure
import os
import time
from DWA_custom_env import DifferentialDrive2D
import csv

save_csv = True
    
data_dir = "data_record_ADWA.csv"
models_dir = f"models/PPO-2DDWA-ADWA-{int(time.time())}"
logdir = f"logs/PPO-2DDWA-ADWA-{int(time.time())}"
# models_dir = f"models/SAC-Tracer-{int(time.time())}"
# logdir = f"logs/SAC-Tracer-{int(time.time())}"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)
    
if not os.path.exists(logdir):
    os.makedirs(logdir)

# c_logger = configure(logdir, ["stdout", "csv", "tensorboard"])

env = DifferentialDrive2D()
env.reset()
print("Firstly, Resetted environment")
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=logdir)
# model.set_logger(c_logger)
TIMESTEPS=100
i = 1
print("Model 'PPO' Ready!")

# recording data
avg_box = 50
collided_nums = 0
if save_csv:
    f = open(data_dir, 'w', encoding='utf-8', newline='')
    wr = csv.writer(f)
    wr.writerow(["avg_vel [m/s]", "avg_traj [m]", "avg_traveled time [s]", "success_rate", "collided numbers"])

episodes = 2000
for ep in range(1, episodes+1):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name='PPO')
    
    collided_nums += env.collided
    
    if i%avg_box==0: # saving data for 50 episodes
        if env.success_num == 0:
            wr.writerow(["FAILED", "FAILED", "FAILED", 0, "FAILED"])
        else:
            wr.writerow([env.avg_vel/env.success_num, env.avg_traj/env.success_num, env.avg_traveledtime / env.success_num, env.success_num/avg_box, collided_nums/env.success_num])
        # resetting values
        env.success_num = 0 
        env.avg_vel = 0
        env.avg_traj = 0
        env.avg_traveledtime = 0
        collided_nums = 0
        
    if i%2==0 and save_csv:
        model.save(f"{models_dir}/{TIMESTEPS*i}")
    
    i += 1

if save_csv:
    f.close()