# Dynamic Window Approach (Local Planning) with moving obstacles
# Yaeohn Kim 2022 September

import math, time, random
# import pygame, math, time, random
# from pygame.locals import *
	
class dwa_pygame:
	def __init__(self):
		# pygame.init()
		
		# Constants and variables
		# Units here are in metres and radians using our standard coordinate frame
		self.BARRIERRADIUS = 0.1
		
		# Timestep delta to run control and simulation at
		self.dt = 0.1
		
		self.ROBOTRADIUS = 0.25
		self.W = 2 * self.ROBOTRADIUS # width of robot
		
		self.SAFEDIST = self.ROBOTRADIUS     # used in the cost function for avoiding obstacles
		
		self.WHEELR = 0.04
		self.MAXLIN_VEL = 1.0     #ms^(-1) max linear velocity
		self.MINLIN_VEL = 0.0      #ms^(-1) min linear velocity
		self.MAXROT_VEL = math.pi * 2 / 3     #rads^(-1) max angular velocity
		self.MINROT_VEL = -self.MAXROT_VEL      #rads^(-1) min angular velocity
		
		self.MAXLIN_ACC = 0.3 / self.dt #ms^(-2) max rate we can change speed of each wheel
		self.MAXROT_ACC = math.pi/ (2 * self.dt) #ms^(-2) max rate we can change speed of each wheel

		self.BARRIERVELOCITYRANGE = 0.1 #ms^(-1) human walking speed
		self.ENV_FLAG = True

		# The region we will fill with obstacles
		self.PLAYFIELDCORNERS = (-4.0, -3.0, 4.0, 3.0)



		# Starting pose of robot
		self.x = self.PLAYFIELDCORNERS[0] - 0.5
		self.y = 0.0
		self.theta = 0.0
		self.lin_vel = 0.0
		self.rot_vel = 0.0
		
		# Use for displaying a trail of the robot's positions
		self.locationhistory = []

		# Starting wheel velocities
		self.vL = 0.00
		self.vR = 0.00

		# self.STEPSAHEADTOPLAN = 10   
		# self.TAU = 1      # N_PREDICT

		# Barrier (obstacle) locations
		self.barriers = []
		self.targetbarrier = []
		
		self.ResetBarriers(5, True)
		# barrier contents are (bx, by, visibilitymask)
		# Generate some initial random barriers
		# for i in range(6):     # 10 human, 5 goal points moving
		#     # if i < 5:      # below vel 0.3
		#     #     (bx, by, vx, vy) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]), random.gauss(0.0, 0.2), random.gauss(0.0, 0.2))
		#     # else: # above 0.8 and limit
		#     #     (bx, by, vx, vy) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]), random.gauss(0.0, self.BARRIERVELOCITYRANGE), random.gauss(0.0, self.BARRIERVELOCITYRANGE))
		#     (bx, by, vx, vy) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]), random.gauss(0.0, self.BARRIERVELOCITYRANGE), random.gauss(0.0, self.BARRIERVELOCITYRANGE))
		#     barrier = [bx, by, vx, vy]
		#     self.barriers.append(barrier)
		
		# # static
		# (bx, by) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]))
		# self.targetbarrier = [bx, by]
		
		# self.targetindex = random.randint(0,len(self.barriers)-11)
		self.disttotarget = 0
		
		# Constants for graphics display
		# Transformation from metric world frame to graphics frame
		# k pixels per metre
		# Horizontal screen coordinate:     u = u0 + k * x
		# Vertical screen coordinate:       v = v0 - k * y

		# set the width and height of the screen (pixels)
		WIDTH = 1500
		HEIGHT = 1000

		size = [WIDTH, HEIGHT]
		self.black = (20,20,40)
		self.lightblue = (0,120,255)
		self.darkblue = (0,40,160)
		self.red = (255,100,0)
		self.white = (255,255,255)
		self.blue = (0,0,255)
		self.grey = (70,70,70)
		self.k = 160 # pixels per metre for graphics

		# Screen centre will correspond to (x, y) = (0, 0)
		self.u0 = WIDTH / 2
		self.v0 = HEIGHT / 2

		# Initialise Pygame display screen
		# self.screen = pygame.display.set_mode(size)
		# This makes the normal mouse pointer invisible in graphics window
		# pygame.mouse.set_visible(0)


		# Array for path choices use for graphics 
		self.pathstodraw = []
		self.locationhistory = []
		
		# reinforcement parameters
		self.done = False
		self.collision = False
		self.min_dist = 10
		self.success = False;
		self.traj = 0;
		self.vel = 0;
		self.duration = 0;

	def moveBarriers(self, dt):
		for (i, barrier) in enumerate(self.barriers):
			self.barriers[i][0] += self.barriers[i][2] * dt
			if (self.barriers[i][0] < self.PLAYFIELDCORNERS[0]):
				self.barriers[i][2] = -self.barriers[i][2]
			if (self.barriers[i][0] > self.PLAYFIELDCORNERS[2]):
				self.barriers[i][2] = -self.barriers[i][2]
			
			self.barriers[i][1] += self.barriers[i][3] * dt
			
			if (self.barriers[i][1] < self.PLAYFIELDCORNERS[1]):
				self.barriers[i][3] = -self.barriers[i][3]
			if (self.barriers[i][1] > self.PLAYFIELDCORNERS[3]):
				self.barriers[i][3] = -self.barriers[i][3]
	
	def ResetBarriers(self, max_num, envF=True):
		self.barriers = []
		self.ENV_FLAG = envF
		num = random.randint(4,max_num)
		if envF:    # dynamic true
			for i in range(num):     # 10 human, 5 goal points moving
				if i < round(num*0.6):      # low speed
					(bx, by, vx, vy) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]), random.gauss(0.0, 0.2), random.gauss(0.0, 0.2))
				else: # above 0.8 and limit
					(bx, by, vx, vy) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]), random.gauss(0.0, self.BARRIERVELOCITYRANGE), random.gauss(0.0, self.BARRIERVELOCITYRANGE))
				
				barrier = [bx, by, vx, vy]
				self.barriers.append(barrier)
		else: # static env
			for i in range(num): 
				# for static env
				(bx, by, vx, vy) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]), random.gauss(0.0, self.BARRIERVELOCITYRANGE), random.gauss(0.0, self.BARRIERVELOCITYRANGE))
			
				barrier = [bx, by, vx, vy]
				self.barriers.append(barrier)
				
		# static goal
		(bx, by) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]))
		self.targetbarrier = [bx, by]
					   
	# Function to predict new robot position based on current pose and velocity controls
	# Uses time N_predict time
	# Returns xnew, ynew, thetanew
	# Also returns the path during N_predict time. 
	def predictPosition(self, v, w, cur_x, cur_y, theta, N_predict):
			# for tau
		path = []   # list [(1,2), (2,3),...]
		xnew = cur_x
		ynew = cur_y
		thetanew = theta
		count = round(N_predict/self.dt)
		for t in range(1, count+2):
			(xnew, ynew, thetanew) = self.KinematicModel(xnew, ynew, thetanew, v, w, self.dt)
			path.append((xnew, ynew))
		return (xnew, ynew, thetanew, path)

	def KinematicModel(self, x_cur, y_cur, theta, v, w, dt):
		x_next = x_cur + v * math.cos(theta) * dt
		y_next = y_cur + v * math.sin(theta) * dt
		theta_next = theta + w*dt
		theta_next = self.glob_RAD(theta_next)
		return (x_next, y_next, theta_next)
	
	# Function to calculate the closest obstacle at a position (x, y)
	# Used during planning
	def calculateClosestObstacleDistance(self, x, y):
		closestdist = 4.0  
		# Calculate distance to closest obstacle
		for (i,barrier) in enumerate(self.barriers):
			# if (i != self.targetindex):
			dx = barrier[0] - x
			dy = barrier[1] - y
			d = math.sqrt(dx**2 + dy**2)
			# Distance between closest touching point of circular robot and circular barrier
			dist = d - self.BARRIERRADIUS - self.ROBOTRADIUS
			if (dist < closestdist):
				closestdist = dist
		
		if closestdist < 0.1: #on collision check true
			self.collision = True
		
		self.min_dist = closestdist
		return closestdist

	# Function to calculate heading difference in robot
	def calculateTargetHeadingDist(self, x, y, theta):
		diff_angle = math.atan2(self.targetbarrier[1]-y, self.targetbarrier[0]-x)
		diff_angle = self.glob_RAD(diff_angle)
		if diff_angle > theta:
			head = diff_angle - theta
		else:
			head = theta - diff_angle
		head = math.pi - head
		head *= 180/math.pi
		
		# has to be flipped, shorter the better!
		goal_dist = 100 - math.sqrt((self.targetbarrier[0]-x)**2 + (self.targetbarrier[1]-y)**2)
		return (head, goal_dist)

	# Function to calculate dynamic window
	def calculateDW(self):
		VL = max(self.MINLIN_VEL, self.lin_vel - self.MAXLIN_ACC* self.dt)
		VH = min(self.MAXLIN_VEL, self.lin_vel + self.MAXLIN_ACC*self.dt)
		WL = max(self.MINROT_VEL, self.rot_vel - self.MAXROT_ACC*self.dt)
		WH = min(self.MAXROT_VEL, self.rot_vel + self.MAXROT_ACC*self.dt)
		return (VL, VH, WL, WH)

	def searchSpace(self, N_PREDICT):
		# init datasets
		EVAL = []
		pathstodraw = []
		(VL, VH, WL, WH) = self.calculateDW()
		
		# decides the resolution of dwa lines
		dv = (VH - VL) / 6
		dw = (WH - WL) / 20
		for i in range(6):
			v = VL + dv*i
			if v > VH:
				v = VH
			elif i == 5:    # end of the index
				v = VH
				
			for j in range(20):
				# print(v,w)
				w = WL + dw*j
				if w > WH:
					w = WH
				elif i == 19:    # end of the index
					w = WH
				
				# generate a path
				(xnew, ynew, thetanew, path) = self.predictPosition(v, w, self.x, self.y, self.theta, N_PREDICT)
				
				# evaluation values (finding the v, w set that maximizes the rest summation)
				(heading, goal_dist) = self.calculateTargetHeadingDist(xnew, ynew, thetanew)
				obs_dist = self.calculateClosestObstacleDistance(xnew, ynew)
				velocity = abs(v)
				EVAL.append((v, w, heading, velocity, obs_dist, goal_dist))
				pathstodraw.append(path)
		
		return (EVAL, pathstodraw)
				
	def normalizeEval(self, EVAL, HEADWEIGHT, VELWEIGHT, OBSDWEIGHT, GOALWEIGHT):
		# v, w, heading, velocity, obs dist, goal dist
		newEVAL=[]
		sum_ele = [0, 0, 0, 0]
		bestBenefits = -10000
		index = 0
		# summing up
		for ev in EVAL:
			for i in range(len(sum_ele)):
				sum_ele[i] += ev[2+i]
				
		# normalizing
		cnt = 0
		for ev in EVAL:
			# newEVAL.append((ev[0], ev[1], ev[2]/sum_ele[0], ev[3]/sum_ele[1], ev[4]/sum_ele[2], ev[5]/sum_ele[3]))
			newEVAL = [ev[0], ev[1], ev[2]/sum_ele[0], ev[3]/sum_ele[1], ev[4]/sum_ele[2], ev[5]/sum_ele[3]]
			benefit = HEADWEIGHT * newEVAL[2] + VELWEIGHT * newEVAL[3] + OBSDWEIGHT * newEVAL[4] + GOALWEIGHT * newEVAL[5]
			if bestBenefits < benefit:
				bestBenefits = benefit
				index = cnt
			cnt += 1
			
		return index
	
	# Draw the barriers on the screen
	# def drawBarriers(self, barriers):
	# 	for (i,barrier) in enumerate (barriers):
	# 		# if (i == self.targetindex):
	# 		#     bcol = self.red
	# 		# else:
	# 		#     bcol = self.lightblue
	# 		bcol = self.lightblue
	# 		# pygame.draw.circle(self.screen, bcol, (int(self.u0 + self.k * barrier[0]), int(self.v0 - self.k * barrier[1])), int(self.k * self.BARRIERRADIUS), 0)
		
	# 	# draw static point
	# 	# pygame.draw.circle(self.screen, self.red, (int(self.u0 + self.k * self.targetbarrier[0]), int(self.v0 - self.k * self.targetbarrier[1])), int(self.k * self.BARRIERRADIUS), 0)
	
	def glob_RAD(self, theta):
		# keeping within range of 180 ~ -180
		# in radians
		if theta > math.pi:
			theta -= 2*math.pi
		elif theta < -math.pi:
			theta += 2*math.pi
		return theta
	
	def main_step(self, HEADWEIGHT=0.1, OBSDWEIGHT=0.3, N_PREDICT=2, VELWEIGHT = 0.2, GOALWEIGHT = 0.0):            
		# Eventlist = pygame.event.get()  # must have
		
		self.locationhistory.append((self.x, self.y)) # For display of trail
		#barrierscopy = copy.deepcopy(self.barriers) # Copy of barriers so we can predict their positions

		(EVAL, pathstodraw) = self.searchSpace(N_PREDICT) # EVALUATE
		
		optimal_index = self.normalizeEval(EVAL, HEADWEIGHT, VELWEIGHT, OBSDWEIGHT, GOALWEIGHT) # GET OPTIMAL CONTROL REF
		# print(EVAL[optimal_index][2], EVAL[optimal_index][4], EVAL[optimal_index][5])
		
		if len(EVAL) != 0:
			self.lin_vel = EVAL[optimal_index][0]   # update lin vel
			self.rot_vel = EVAL[optimal_index][1]   # update rot vel
		else:
			self.lin_vel = 0   # update lin vel
			self.rot_vel = 0   # update rot vel
		# Graphics
		self.screen.fill(self.black)
		# for loc in self.locationhistory:
			# pygame.draw.circle(self.screen, self.grey, (int(self.u0 + self.k * loc[0]), int(self.v0 - self.k * loc[1])), 3, 0)
		# self.drawBarriers(self.barriers)
		
		# Draw robot
		u = self.u0 + self.k * self.x
		v = self.v0 - self.k * self.y
		# pygame.draw.circle(self.screen, self.white, (int(u), int(v)), int(self.k * self.ROBOTRADIUS), 3)
		# Draw wheels as little blobs so you can see robot orientation
		# left wheel centre 
		wlx = self.x - (self.W/2.0) * math.sin(self.theta)
		wly = self.y + (self.W/2.0) * math.cos(self.theta)
		ulx = self.u0 + self.k * wlx
		vlx = self.v0 - self.k * wly
		WHEELBLOB = 0.04
		# pygame.draw.circle(self.screen, self.blue, (int(ulx), int(vlx)), int(self.k * WHEELBLOB))
		# right wheel centre 
		wrx = self.x + (self.W/2.0) * math.sin(self.theta)
		wry = self.y - (self.W/2.0) * math.cos(self.theta)
		urx = self.u0 + self.k * wrx
		vrx = self.v0 - self.k * wry
		# pygame.draw.circle(self.screen, self.blue, (int(urx), int(vrx)), int(self.k * WHEELBLOB))


		if (1):
				# Draw paths with smoothed lines
			for path in pathstodraw:
				if not path is None:
					for i in range(len(path)-1):
						startu = self.u0 + self.k * path[i][0]
						startv = self.v0 - self.k * path[i][1]
						endu = self.u0 + self.k * path[i+1][0]
						endv = self.v0 - self.k * path[i+1][1]
						# pygame.draw.aaline(self.screen, (0, 200, 0), [startu, startv], [endu, endv], 1)

		# Update display
		# pygame.display.flip()

		# Actually now move robot based on optimal reference
		(self.x, self.y, self.theta) = self.KinematicModel(self.x, self.y, self.theta, self.lin_vel, self.rot_vel, self.dt)
		self.createObs() # for RL
		
		if self.ENV_FLAG:
			self.moveBarriers(self.dt)
		
		# computing for rl parameters
		self.check_goal_reached()
		
		self.duration += self.dt        # counting traveled time
		self.traj += abs(self.lin_vel) * self.dt  # calculating moved traj
		
		if self.duration > 15:  # setting time limit
				self.done = True
				
		# Sleeping dt here runs simulation in real-time
		time.sleep(self.dt / 50)

	def check_goal_reached(self):
		# Wraparound: check if robot has reached target; if so reset it to the other side, randomise
		# target position and add some more barriers to go again
		self.disttotarget = math.sqrt((self.x - self.targetbarrier[0])**2 + (self.y - self.targetbarrier[1])**2)
		if (self.disttotarget < (self.BARRIERRADIUS + self.ROBOTRADIUS)):   # goal reached!
			# Add new barriers
			# for i in range(3):
			#         (bx, by) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]))
			#         (bx, by, vx, vy) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]), random.uniform(-self.BARRIERVELOCITYRANGE, self.BARRIERVELOCITYRANGE), random.uniform(-self.BARRIERVELOCITYRANGE, self.BARRIERVELOCITYRANGE))
			#         barrier = [bx, by, vx, vy]
			#         self.barriers.append(barrier)
			# self.targetindex = random.randint(0,len(self.barriers)-11)

			# new static position
			# (bx, by) = (random.uniform(self.PLAYFIELDCORNERS[0], self.PLAYFIELDCORNERS[2]), random.uniform(self.PLAYFIELDCORNERS[1], self.PLAYFIELDCORNERS[3]))
			# self.targetbarrier = [bx, by]
			# self.ResetBarriers(random.randint(4,15), random.randint(0,1))
			# Reset trail
			# self.locationhistory = []
			self.done=True
			
			if not self.collision:
				self.success = True

	def createObs(self):
		self.vL = 2*(self.lin_vel - self.rot_vel *self.W) / (2*self.WHEELR)
		self.vR = 2*(self.lin_vel + self.rot_vel *self.W) / (2*self.WHEELR)
	
	def reset_var(self):
		self.ResetBarriers(random.randint(4,15), random.randint(0,1))
		# Reset trail
		self.locationhistory = []
	
		self.lin_vel = 0
		self.rot_vel = 0
		self.done = False
		self.collision = False
		self.min_dist = 20
		self.success = False;
		self.traj = 0;
		self.duration = 0;
		
		# def static_scenario(self):
			
		
# temporarily check     
# env = dwa_pygame()
# for i in range(1,400):
#     env.main_step(0.3,0.1,2,0.3)
#     # print(env.lin_vel, env.rot_vel)
#     # time.sleep(env.dt/10)
# env.reset_var()
# for i in range(1,500):
#         env.main_step(0.3,0.01,2,0.4)
#         # print(env.lin_vel, env.rot_vel)
#         # time.sleep(env.dt/10)