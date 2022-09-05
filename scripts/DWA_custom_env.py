# Dynamic Window Approach (Local Planning) with moving obstacles
# For Training
# Yaeohn Kim 2022 September

import numpy as np
import gym
from gym import spaces
from dynamic_window_approach_game import dwa_pygame

class DifferentialDrive2D(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self): #, render_mode: Optional[str] = None, size: int = 5):
        
        # observation space
        # vehicle left and right linear wheel velocity
        highs = np.array([1.0, 1.0], dtype=np.float64)
        lows = np.array([-1.0, -1.0], dtype=np.float64)
        self.observation_space = spaces.Box(low = lows, high=highs, dtype = np.float64) 
        
        # for all elements, values most be modified for action space
        # goaldist(heading), obstacledist(dist), N_predict (1~5 secs)
        # highs = np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float64)
        # lows = np.array([-1.0, -1.0, -1.0, -1.0], dtype=np.float64)
        highs = np.array([1.0, 1.0, 1.0], dtype=np.float64)
        lows = np.array([-1.0, -1.0, -1.0], dtype=np.float64)
        self.action_space = spaces.Box(low = lows, high=highs, dtype = np.float64)
        
        
        self.collided = 0  # obstacle collision check
        self.done = False   # True if obstacle is collided or goal point made
        self.success_num = 0   # will calculated by episodes
        self.avg_traj = 0
        self.avg_vel = 0
        self.avg_traveledtime = 0
        
        # environment based on pygame
        self.agent_py = dwa_pygame()
        
    def step(self, action):
        
        # input of action
        hd_weight = (float(action[0]) + 1.0) / 2.0  # 0~1
        obsd_weight = (float(action[1]) + 1.0) / 2.0  # 0~1
        vel_weight = (float(action[2]) + 1.0) / 2.0  # 0~1
        # n_predict = (float(action[3]) + 1.5) * 2 / 2.5  # 0.4~2.5
        self.agent_py.main_step(hd_weight, obsd_weight, 2, vel_weight)
        
        # 2. from server information and return
        norm_lin_vel = (self.agent_py.lin_vel  - (self.agent_py.MAXLIN_VEL - self.agent_py.MINLIN_VEL) * 0.5) / (self.agent_py.MAXLIN_VEL - self.agent_py.MINLIN_VEL)
        norm_rot_vel = (self.agent_py.rot_vel  - (self.agent_py.MAXROT_VEL - self.agent_py.MINROT_VEL) * 0.5) / (self.agent_py.MAXROT_VEL - self.agent_py.MINROT_VEL)
        # obs = np.array([self.agent_py.vL, self.agent_py.vR], dtype=np.float64)
        obs = np.array([norm_lin_vel, norm_rot_vel], dtype=np.float64)
        
        # 3. based on the signals that is sent from unity
        
        # reward based on distance between obstacles and vehicle
        if self.agent_py.min_dist > 1.5:    
            self.reward += 800
        elif self.agent_py.min_dist > 1.0:
            self.reward += 150
        elif self.agent_py.min_dist > 0.6:
            self.reward += 1
        else:
            self.reward += -50
        
        # reward based on collsion
        if self.agent_py.collision:
            self.reward -= 8000
            self.collided += 1
        else:
            self.reward += 500
        
        # reward based on goal to distance
        if self.agent_py.disttotarget < 1.0:
            self.reward += 300
        elif self.agent_py.disttotarget < 3.0:
            self.reward += 100
        
        self.done = self.agent_py.done
        # information
        # info = {
        #     "total reward" :        self.total_reward,
        #     "goal arrived" :        self.done,
        #     "goal distance left" :  obs[2]            
        # }
        info = {}
        
        # resetting collsision count
        self.agent_py.collision = False
        return obs, self.reward, self.done, info
        
    def reset(self):
        # only saving when successfully reached its goal
        if self.agent_py.success:
            self.success_num += 1
            self.avg_traj += self.agent_py.traj
            self.avg_vel += self.agent_py.lin_vel
            self.avg_traveledtime += self.agent_py.duration
        else:   # not counting for failed cases
            self.collided = 0
            
        self.agent_py.reset_var()

        # initializing environment for retraining
        self.done = False
        self.reward = 0
        # and then get new observation
        
        norm_lin_vel = (self.agent_py.lin_vel  - (self.agent_py.MAXLIN_VEL - self.agent_py.MINLIN_VEL) * 0.5) / (self.agent_py.MAXLIN_VEL - self.agent_py.MINLIN_VEL)
        norm_rot_vel = (self.agent_py.rot_vel  - (self.agent_py.MAXROT_VEL - self.agent_py.MINROT_VEL) * 0.5) / (self.agent_py.MAXROT_VEL - self.agent_py.MINROT_VEL)
        # obs = np.array([self.agent_py.vL, self.agent_py.vR], dtype=np.float64)
        obs = np.array([norm_lin_vel, norm_rot_vel], dtype=np.float64)
                
        return obs  # reward, done, info can't be included
        
