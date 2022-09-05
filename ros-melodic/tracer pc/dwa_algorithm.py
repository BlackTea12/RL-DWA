#!/usr/bin/env python

# Dynamic Window Approach (Local Planning)
# For Tracer with Lidar
# Yaeohn Kim 2022 September

from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sys
import math
import rospy
class ros_dwa:
    def __init__(self):
        # -------------------- Constants and variables -------------------- #
        # Units here are in metres and radians using our standard coordinate frame
        self.BARRIERRADIUS = 0.2
        
        # Timestep delta to run control and simulation at
        self.dt = 0.1
        
        self.ROBOTRADIUS = 0.25
        self.W = 2 * self.ROBOTRADIUS # width of robot
        
        self.SAFEDIST = self.ROBOTRADIUS     # used in the cost function for avoiding obstacles
        
        self.WHEELR = 0.04
        self.MAXLIN_VEL = 0.3     #ms^(-1) max linear velocity
        self.MINLIN_VEL = 0.0      #ms^(-1) min linear velocity
        self.MAXROT_VEL = math.pi / 4     #rads^(-1) max angular velocity
        self.MINROT_VEL = -self.MAXROT_VEL      #rads^(-1) min angular velocity
        
        self.MAXLIN_ACC = 0.1 / self.dt #ms^(-2) max rate we can change speed of each wheel
        self.MAXROT_ACC = math.pi / (4 * self.dt) #ms^(-2) max rate we can change speed of each wheel

        self.BARRIERVELOCITYRANGE = 0.1 #ms^(-1) human walking speed
        self.VEL_INTV = 5
        self.ROT_INTV = 10
        # -------------------- END -------------------- #


        self.targetbarrier = (3,0)   # 6m ahead of initial position
        self.x = 0  # ideal initial vehicle position x
        self.y = 0  # ideal initial vehicle position y
        self.theta = 0  # ideal initial vehicle position theta
        self.barriers = []  # obstacles
        self.lin_vel = 0
        self.rot_vel = 0
        self.emergency_stop = False  # safety ensurance
        self.pathstodraw = []

        # -------------------- ROS related parameters -------------------- #
        self.node_name = "dwa_original"
        self.marker = Marker()
        self.marker.header.frame_id = "/dwa_path"
        self.marker.type = self.marker.POINTS
        self.marker.action = self.marker.ADD
        self.adaptive_param = [0.0, 0.0, 0.0, 1.0] # head, dist, vel, N_predict
        rospy.init_node(self.node_name)

        rospy.Subscriber("/realsense/obstacles", PoseArray, self.obsFetchCallback)
        rospy.Subscriber("/apl_param", Float64MultiArray, self.apldwaCallback)

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_visual_path = rospy.Publisher("/dwa_pathdraw", Marker, queue_size=1)
        # -------------------- END -------------------- #

        # rospy.spin()

    def set_visual(self):
        # self.marker.header.frame_id = "/dwa_pathdraw"
        # self.marker.type = self.marker.POINTS
        # self.marker.action = self.marker.ADD

        # marker scale
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.02

        # marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0

        # marker line points
        self.marker.points = []
        
        for point in self.pathstodraw:
            for ptup in point:  # one set of possible path in tuple (x, y)
                temp_point = Point()
                temp_point.x = -ptup[0]
                temp_point.y = ptup[1] - 0.08
                temp_point.z = 0.0
                self.marker.points.append(temp_point)
        
        self.pub_visual_path.publish(self.marker)
            
    def KinematicModel(self, x_cur, y_cur, theta, v, w, dt):
        x_next = x_cur + v * math.cos(theta) * dt
        y_next = y_cur + v * math.sin(theta) * dt
        theta_next = theta + w*dt
        theta_next = self.glob_RAD(theta_next)
        return (x_next, y_next, theta_next)
        
    def predictPosition(self, v, w, cur_x=0.0, cur_y=0.0, theta=0.0, N_predict=2.0):
        # for tau
        path = []   # list [(1,2), (2,3),...]
        xnew = cur_x
        ynew = cur_y
        thetanew = theta
        count = int(round(N_predict/self.dt))
        for t in range(1, count+2):
            (xnew, ynew, thetanew) = self.KinematicModel(xnew, ynew, thetanew, v, w, self.dt)
            path.append((xnew, ynew))
        return (xnew, ynew, thetanew, path)
    
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
        
        # if closestdist < 0.2:
        #     self.emergency_stop = True
        # else:
        #     self.emergency_stop = False
        # print(closestdist)
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
        dv = (VH - VL) / self.VEL_INTV
        dw = (WH - WL) / self.ROT_INTV
        for i in range(self.VEL_INTV):
            v = VL + dv*i
            if v > VH:
                v = VH
            elif i == self.VEL_INTV - 1:    # end of the index
                v = VH
                
            for j in range(self.ROT_INTV):
                # print(v,w)
                w = WL + dw*j
                if w > WH:
                    w = WH
                elif i == self.ROT_INTV - 1:    # end of the index
                    w = WH
                
                # generate a path
                (xnew, ynew, thetanew, path) = self.predictPosition(v, w, N_predict=N_PREDICT)
                
                # evaluation values (finding the v, w set that maximizes the rest summation)
                (heading, goal_dist) = self.calculateTargetHeadingDist(xnew, ynew, thetanew)
                obs_dist = self.calculateClosestObstacleDistance(xnew, ynew)
                velocity = abs(v)
                EVAL.append((v, w, heading, velocity, obs_dist, goal_dist))
                pathstodraw.append(path)

        return (EVAL, pathstodraw)
                
    def normalizeEval(self, EVAL, HEADWEIGHT, VELWEIGHT, OBSDWEIGHT, GOALWEIGHT):
        # v, w, heading, velocity, obs dist, goal dist
        newEVAL=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        sum_ele = [0.0, 0.0, 0.0, 0.0]
        bestBenefits = -10000
        index = 0
        # summing up
        for ev in EVAL:
            for i in range(len(sum_ele)):
                sum_ele[i] += ev[2+i]
                
        # normalizing
        cnt = 0
        for ev in EVAL:
            if sum_ele[0] == 0:
                newEVAL[2] = 0
            else:
                newEVAL[2] = ev[2]/sum_ele[0]

            if sum_ele[1] == 0:
                newEVAL[3] = 0
            else:
                newEVAL[3] = ev[3]/sum_ele[1]

            if sum_ele[2] == 0:
                newEVAL[4] = 0
            else:
                newEVAL[4] = ev[4]/sum_ele[2]

            if sum_ele[3] == 0:
                newEVAL[5] = 0
            else:
                newEVAL[5] = ev[5]/sum_ele[3]
            
            newEVAL[0] = ev[0]
            newEVAL[1] = ev[1]

            benefit = HEADWEIGHT * newEVAL[2] + VELWEIGHT * newEVAL[3] + OBSDWEIGHT * newEVAL[4] + GOALWEIGHT * newEVAL[5]
            if bestBenefits < benefit:
                bestBenefits = benefit
                index = cnt
            cnt += 1
            
        return index

    def glob_RAD(self, theta):
        # keeping within range of 180 ~ -180
        # in radians
        if theta > math.pi:
            theta -= 2*math.pi
        elif theta < -math.pi:
            theta += 2*math.pi
        return theta
    
    def main_loop(self, HEADWEIGHT = 0.1, OBSDWEIGHT = 0.3, N_PREDICT = 2, VELWEIGHT = 0.2, GOALWEIGHT = 0.0):                   
        (EVAL, self.pathstodraw) = self.searchSpace(N_PREDICT) # EVALUATE
        # self.set_visual(pathstodraw)

        optimal_index = self.normalizeEval(EVAL, HEADWEIGHT, VELWEIGHT, OBSDWEIGHT, GOALWEIGHT) # GET OPTIMAL CONTROL REF

        if len(EVAL) != 0:
            self.lin_vel = EVAL[optimal_index][0]   # update lin vel
            self.rot_vel = -EVAL[optimal_index][1]   # update rot vel
        else:
            self.lin_vel = 0   # update lin vel
            self.rot_vel = 0   # update rot vel
        
        if self.emergency_stop: 
            self.lin_vel = 0   # update lin vel
            self.rot_vel = 0   # update rot vel

        # to tracer actual command
        command = Twist()
        command.linear.x = self.lin_vel
        command.angular.z = self.rot_vel
        self.pub_cmd.publish(command)
        
        # (IDEAL MOVEMENT) Actually now move robot based on optimal reference
        # (self.x, self.y, self.theta) = self.KinematicModel(self.x, self.y, self.theta, self.lin_vel, self.rot_vel, self.dt)

    def obsFetchCallback(self, poses):
        # send as ros std_msgs
        try:
            self.emergency_stop = False
            self.barriers = []  # initialize
            if len(poses.poses) != 0: # obs exist
                for pose in poses.poses:
                    theta = math.pi/2.0 - math.atan2(pose.position.y+0.08, pose.position.x)
                    l = math.sqrt((pose.position.y+0.08)**2 + pose.position.x**2)
                    t_x = l*math.sin(math.pi/4.0-theta)
                    t_y = l*math.cos(math.pi/4.0-theta)
                    self.barriers.append([t_x, t_y, pose.position.z])  #x, y, depth
                    if pose.position.z < 0.2:   # depth, too close
                        self.emergency_stop = True
                    
            # print(self.barriers)
            sys.stdout.write('Command lin: %.3f, rot: %.3f\r' % (self.lin_vel, self.rot_vel))
            sys.stdout.flush()
            self.main_loop()
            return
        except Exception as e:
            print(e)
            return

    def apldwaCallback(self, params):
        try:
            if len(params.data) != 0:
                self.apl_param = [params.data[0], params.data[1], params.data[2], params.data[3]]
            pass
        except Exception as e:
            print(e)
            return

        
           
if __name__ == '__main__':
    dwa_ = ros_dwa()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        dwa_.set_visual()
        rate.sleep()
