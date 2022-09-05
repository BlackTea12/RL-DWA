#!/usr/bin/env python3

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure
from dwa_custom_env import DifferentialDrive2D
import numpy as np
import math
import sys
import rospy

class rldwa:
	def __init__(self):
		self.models_dir = f"~/dwa_yk/models/apl_dwa.zip"
		# self.models_dir = f"~/dwa_yk/models/adwa.zip"
		self.env = DifferentialDrive2D()
		self.model = PPO.load(self.models_dir, env=self.env)

		# observation space
		self.obsSpace = np.array([0.0, 0.0], dtype=np.float64)

		# action space (head, obs, vel, N_predict)
		self.actSpace = np.array([0.0, 0.0, 0.0, 0.4], dtype=np.float64)
		
		# rospy init node
		rospy.init_node("rldwa")

		#rospy subscriber, observation space
		rospy.Subscriber("/odom", Odometry, self.obsspaceCallback)

		#rospy publisher, action space
		self.pub_action = rospy.Publisher("/apl_param", Float64MultiArray, queue_size=1)

	def modelPredictAction(self):
		action, _states = self.model.predict(self.obsSpace, deterministic=True)
		hd_weight = (float(action[0])+1.0)/2.0 #0~1
		obsd_weight = (float(action[1])+1.0)/2.0 #0~1
		vel_weight = (float(action[2])+1.0)/2.0 #0~1
		n_predict = (float(action[3])+1.5)*2/2.5 #0.4~2.5

		# (head, obs, vel, N_predict)
		actions_pub = Float64MultiArray()
		actions_pub.data = [hd_weight, obsd_weight, vel_weight, n_predict]
		sys.stdout.write('Command H: %.3f, Obs: %.3f, V: %.3f, N: %.3f\r' % (hd_weight, obsd_weight, vel_weight, n_predict))
		sys.stdout.flush()
		self.pub_action.publish(actions_pub)

	def obsspaceCallback(self, data):
		# twist: linear x and orientation z (right -, left +) 
		try:
			self.obsSpace[0] = (data.twist.twist.linear.x  - 0.15) / 0.3
			self.obsSpace[1] = -(data.twist.twist.angular.z  - math.pi/4.0) / (math.pi / 2.0)
			return
		except Exception as e:
			print(e)
			return 
	

if __name__ == '__main__':
	APL_DWA = rldwa()
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		APL_DWA.modelPredictAction()
		# print("spinning")
		rate.sleep()
