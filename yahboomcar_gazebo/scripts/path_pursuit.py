#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
import csv
import os

Static_Velocity=0.5
Static_Angle= 0
class following_path:
    
    def __init__(self):
        self.current_pose = rospy.Subscriber('/pf/pose/odom', Odometry, self.callback_read_current_position, queue_size=1)
        self.Pose = []
        self.path_pose = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.callback_read_path, queue_size=1)
        self.path_info = []
        self.Goal = []
        self.navigation_input = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.my_laser_scan = rospy.Subscriber('scan',LaserScan,self.callback_read_laser_scan,queue_size=1)
        self.reach_goal = False
        self.MAX_VELOCITY = 1.2
        self.MIN_VELOCITY = 0
        self.max_angle = 0.8
        self.steering_velocity = 1
        self.jerk = 0.0
        self.acceleration = 0.0
        self.LOOKAHEAD_DISTANCE = 0.4
        self.Low_Speed_Mode = False
        self.IsVertical=False
        self.Porprotion = 0.91
        self.Porprotion_Min =0.85
	self.Ismounton = False
        self.IsLine=False
	self.P_Angle =0.9
	self.IsLook = False
        self.Is180 =False
        self.left_avg_length=1
        self.right_avg_length=1
        self.left_front_avg_length=1
        self.right_front_avg_length=1
        self.velocity_k=1
        self.IsBaffle = False
	self.IsZhui = False
###############################################################################################################################################################################     
    '''def humanCon(self, dist_array):
        goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 1.2)) & (dist_array > (self.LOOKAHEAD_DISTANCE+0.6)))[0]
	self.IsVertical=False
        for id in goal_array:
                #v1 = [path_points_x[id] - x, path_points_y[id] - y]
                #v2 = [math.cos(yaw), math.sin(yaw)]        
               # diff_angle_val = self.find_angle(v1,v2)
           diff_angle_val = path_points_w[id] - yaw
                #print("diff_angle_val Vertical"+str(diff_angle_val*57))
           if (abs(diff_angle_val) > np.pi/180*30) and (abs(diff_angle_val) < (np.pi/180*60)): # Check if the one that is the cloest to the lookahead direction
               self.IsVertical=True
	       print("vertical"+str(diff_angle_val*57))    
               break
           
        goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 1.2)) & (dist_array > (self.LOOKAHEAD_DISTANCE+0.6)))[0]
        LineCounter=0
        self.IsLine=False
        for id in goal_array:
                #v1 = [path_points_x[id] - x, path_points_y[id] - y]
                #v2 = [math.cos(yaw), math.sin(yaw)]        
               # diff_angle_val = self.find_angle(v1,v2)
            diff_angle_val = path_points_w[id] - yaw
                #print("diff_angle_val Vertical"+str(diff_angle_val*57))
            if (abs(diff_angle_val) >= np.pi/180*0) and (abs(diff_angle_val) < (np.pi/180*20)): # Check if the one that is the cloest to the lookahead direction
                self.IsVertical=True
                Linecounter += 1
                if LineCounter==5:
		    print("Line"+str(diff_angle_val*57))
                    self.IsLine=True
                    break
                    
	    #print(str(diff_angle*57))
	pre_goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 1.6)) & (dist_array > (self.LOOKAHEAD_DISTANCE +0.6)))[0]
	self.Ismounton = False
	for id in pre_goal_array:
            v1 = [path_points_x[id] - x, path_points_y[id] - y]
            v2 = [math.cos(yaw), math.sin(yaw)]
            diff_angle_val = self.find_angle(v1,v2)
	    count=0
            if (abs(diff_angle_val) > np.pi/6 )and( abs(diff_angle_val) < np.pi/4): # Check if the one that is the cloest to the lookahead direction
                count=count+1
	    if count>=5:
		self.Ismounton = True
		print("mounton")'''
######################################################################################################################
    def callback_read_laser_scan(self,data):
        #LaserScan
    	#std_msgs/Header header
    	#float32 angle_min
    	#float32 angle_max
    	#float32 angle_increment
    	#float32 time_increment
    	#float32 scan_time
    	#float32 range_min
    	#float32 range_max
    	#float32[] ranges
    	#float32[] intensities
        #diff_dist=np.zeros(29)
	data_scan=data
        
        data_scan.ranges = list(data.ranges)
        #for x in range(0,360):
           # if not(data.ranges[x]<=8&data.ranges[x]>=0)
	   #     data.ranges[x]
        #right_avg_length=np.mean(np.where(data.ranges[30:100]))   
#####   
        right_sum_length=0
	for x in range(40,100):
            if not(data.ranges[x]<=8 and data.ranges[x]>=0):
                data.ranges[x]=8 
	    right_sum_length=right_sum_length+data.ranges[x]
        self.right_avg_length=right_sum_length/60

#####
        right_front_sum_length=0
	for x in range(105,135):
            if not(data.ranges[x]<=8 and data.ranges[x]>=0):
                data.ranges[x]=8 
        tmp_list = data.ranges[105:135]
        tmp_list.sort()
        diff_dist=np.zeros(29)
        small_flag=0
	self.IsZhui=False
        for ii in range(29):
            diff_dist[ii]=data.ranges[ii+1]-data.ranges[ii]
            if diff_dist[ii]>=1.2:
                small_flag=ii
		self.IsZhui=True
        if small_flag<7:
            small_flag=7
        for x in range(small_flag): 
	    right_front_sum_length=right_front_sum_length+tmp_list[x]
        self.right_front_avg_length=right_front_sum_length/small_flag
	if self.IsZhui:
	    self.right_front_avg_length=self.right_front_avg_length-0.165
       # print("right_avg_length:"+str(self.right_avg_length))
       #print("right_avg_length:"+str(self.right_avg_length))tmp
#####
        
        left_sum_length=0
	for x in range(260,320):
            if not(data.ranges[x]<=8 and data.ranges[x]>=0):
                data.ranges[x]=8  
	    left_sum_length=left_sum_length+data.ranges[x]
        self.left_avg_length=left_sum_length/60

#####
	#print("left_avg_length:"+str(self.left_avg_length)+"     right_avg_length:"+str(self.right_avg_length))'''
        left_front_sum_length=0
	for x in range(225,255):
            if not(data.ranges[x]<=8 and data.ranges[x]>=0):
                data.ranges[x]=8 
        tmp_list = data.ranges[225:255]
        tmp_list.sort()
        diff_dist=np.zeros(29)
        small_flag=0
	self.IsZhui=False
        for ii in range(29):
            diff_dist[ii]=data.ranges[ii+1]-data.ranges[ii]
            if diff_dist[ii]>=1.2:
                small_flag=ii
		self.IsZhui=True
        if small_flag<7:
            small_flag=7
        for x in range(small_flag): 
	    left_front_sum_length=left_front_sum_length+tmp_list[x]
        self.left_front_avg_length=left_front_sum_length/small_flag
	if self.IsZhui:
	    self.left_front_avg_length=self.left_front_avg_length-0.165
        #print("right_front_avg_length:"+str(self.right_front_avg_length))
        #print("left_front_avg_length:"+str(self.left_front_avg_length))
#####
	for x in range(155,205):
	    if not(data.ranges[x]<=8 and data.ranges[x]>=0):
                data.ranges[x]=8
	temp_data_ranges=data.ranges
	for ii in range(60):
	    faraway=0.672
	    if Static_Velocity>0.3 and Static_Velocity<0.5:
		faraway=2.24*Static_Velocity
	    if Static_Velocity>=0.5:
		faraway=1.12
	    if temp_data_ranges[ii]<faraway:
		self.IsBaffle = True
#####
	for x in range(90,270):
            if not(data.ranges[x]<=8 and data.ranges[x]>=0):
                data.ranges[x]=8
	'''temp_data_ranges=data.ranges
	b_count=0
	for ii in range(180):
	    if temp_data_ranges[ii+1]<temp_data_ranges[ii]:
		baffle_index=ii
		for i in range(12):
                    if temp_data_ranges[baffle_index]>temp_data_ranges[baffle_index+i]:
		        b_count=b_count+1
	            if b_count>10 and temp_data_ranges[baffle_index+i]>temp_data_ranges[baffle_index+5]:
		        self.IsZhui=True
		        print("IsZhui")'''

###############################################################################################################################################################################
    def callback_read_path(self, data):
        # Organize the pose message and only ask for (x,y) and orientation
        # Read the Real time pose message and load them into path_info
        self.path_info = []
        path_array = data.poses
        for path_pose in path_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_qx = path_pose.pose.orientation.x
            path_qy = path_pose.pose.orientation.y
            path_qz = path_pose.pose.orientation.z
            path_qw = path_pose.pose.orientation.w
            path_quaternion = (path_qx, path_qy, path_qz, path_qw)
            path_euler = euler_from_quaternion(path_quaternion)
            path_yaw = path_euler[2]
            self.path_info.append([float(path_x), float(path_y), float(path_yaw)])
        self.Goal = list(self.path_info[-1]) # Set the last pose of the global path as goal location
###############################################################################################################################################################################
    def callback_read_current_position(self, data):
	global Static_Angle
	global Static_Velocity
        if self.reach_goal: # Stop updating the information.
            self.path_info = []
            self.Pose = []
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0

        if  len(self.path_info) >= 1:
            #print(str(len(self.path_info)))
            # Read the path information to path_point list
            path_points_x = [float(point[0]) for point in self.path_info]
            path_points_y = [float(point[1]) for point in self.path_info]
            path_points_w = [float(point[2]) for point in self.path_info]

            # Read the current pose of the car from particle filter
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w

            # Convert the quaternion angle to eular angle
            quaternion = (qx,qy,qz,qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.Pose = [float(x), float(y), float(yaw)]

            if self.dist(self.Goal, self.Pose) < 1.0:
                self.Low_Speed_Mode = False
                if self.dist(self.Goal, self.Pose) < 0.3:
                    self.reach_goal = True
                    print('Goal Reached!')
                else:
                    print('Low Speed Mode ON!')
            else:
                self.Low_Speed_Mode = False

            # 2. Find the path point closest to the vehichle tat is >= 1 lookahead distance from vehicle's current location.
            dist_array = np.zeros(min(len(path_points_x), len(path_points_y), len(path_points_w)))
            #if 0==len(dist_array):
			#	dist_array = np.zeros(1)
            #if not len(path_points_x) == len(path_points_y):
                #print(str(len(path_points_x))+"   "+str(len(path_points_y))+'    '+str(len(dist_array)))



            #try:
            for i in range(len(dist_array)):
            	#if i==len(path_points_x)-1:
            	#	break
                dist_array[i] = self.dist((path_points_x[i], path_points_y[i]), (x,y))
	    #except IndexError:	
                #pass
                #print(str(len(path_points_x))+"   "+str(len(path_points_y))+'    '+str(len(dist_array)))
            
            goal = np.argmin(dist_array) # Assume the closet point as the goal point at first
            goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 0.3)) & (dist_array > (self.LOOKAHEAD_DISTANCE - 0.3)))[0]
            for id in goal_array:
                v1 = [path_points_x[id] - x, path_points_y[id] - y]
                v2 = [math.cos(yaw), math.sin(yaw)]
                diff_angle = self.find_angle(v1,v2)
                if abs(diff_angle) < np.pi/4: # Check if the one that is the cloest to the lookahead direction
                    goal = id
		    self.IsLook=True
                    break
            
            L = dist_array[goal]
            # 3. Transform the goal point to vehicle coordinates. 
            glob_x = path_points_x[goal] - x 
            glob_y = path_points_y[goal] - y 
            goal_x_veh_coord = glob_x*np.cos(yaw) + glob_y*np.sin(yaw)
            goal_y_veh_coord = glob_y*np.cos(yaw) - glob_x*np.sin(yaw)

            # 4. Calculate the curvature = 1/r = 2x/l^2
            # The curvature is transformed into steering wheel angle by the vehicle on board controller.
            # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
            diff_angle = path_points_w[goal] - yaw # Find the turning angle


 ########################################################################          
            goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 1.2)) & (dist_array > (self.LOOKAHEAD_DISTANCE+0.7)))[0]
	    self.IsVertical=False
            for id in goal_array:
                #v1 = [path_points_x[id] - x, path_points_y[id] - y]
                #v2 = [math.cos(yaw), math.sin(yaw)]        
               # diff_angle_val = self.find_angle(v1,v2)
               diff_angle_val = path_points_w[id] - yaw
                #print("diff_angle_val Vertical"+str(diff_angle_val*57))
               if (abs(diff_angle_val) > np.pi/180*30) and (abs(diff_angle_val) < (np.pi/180*60)): # Check if the one that is the cloest to the lookahead direction
                   self.IsVertical=True
	           print("vertical"+str(diff_angle_val*57))    
                   break
 #####################          
            goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 1.2)) & (dist_array > (self.LOOKAHEAD_DISTANCE+0.6)))[0]
            LineCounter=0
            self.IsLine=False
            for id in goal_array:
                if id % 6 == 1:
                #v1 = [path_points_x[id] - x, path_points_y[id] - y]
                #v2 = [math.cos(yaw), math.sin(yaw)]        
               # diff_angle_val = self.find_angle(v1,v2)
                    diff_angle_val = path_points_w[id] - yaw
                #print("diff_angle_val Vertical"+str(diff_angle_val*57))
                    if (abs(diff_angle_val) >= np.pi/180*0) and (abs(diff_angle_val) < (np.pi/180*10)): # Check if the one that is the cloest to the lookahead direction
                        LineCounter += 1
                        if LineCounter==6:
		            print("Line"+str(diff_angle_val*57))
                            self.IsLine=True
                            break
 #####################                   
	    #print(str(diff_angle*57))
            mountonCounter=0
	    pre_goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 1.1)) & (dist_array > (self.LOOKAHEAD_DISTANCE +0.5)))[0]
	    self.Ismounton = False
	    for id in goal_array:
                if id % 2 == 1:
                #v1 = [path_points_x[id] - x, path_points_y[id] - y]
                #v2 = [math.cos(yaw), math.sin(yaw)]        
               # diff_angle_val = self.find_angle(v1,v2)
                    diff_angle_val = path_points_w[id] - yaw
                    #diff_angle_val_1 = path_points_w[id] - yaw
                #print("diff_angle_val Vertical"+str(diff_angle_val*57))
                    if (abs(diff_angle_val) >= np.pi/180*15) and (abs(diff_angle_val) < (np.pi/180*27)): # Check if the one that is the cloest to the lookahead direction
                        mountonCounter += 1
                        if mountonCounter==2:
		            print("mounton"+str(diff_angle_val*57))
                            self.Ismounton=True
                            break
 #####################                   
	    #print(str(diff_angle*57))
            Counter180=0
	    pre_goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 0.7)) & (dist_array > (self.LOOKAHEAD_DISTANCE +0.1)))[0]
	    self.Is180 = False
	    for id in goal_array:
                if id % 2 == 1:
                #v1 = [path_points_x[id] - x, path_points_y[id] - y]
                #v2 = [math.cos(yaw), math.sin(yaw)]        
               # diff_angle_val = self.find_angle(v1,v2)
                    diff_angle_val = path_points_w[id] - yaw
                    #diff_angle_val_1 = path_points_w[id] - yaw
                #print("diff_angle_val Vertical"+str(diff_angle_val*57))
                    if (abs(diff_angle_val) >= np.pi/180*60) and (abs(diff_angle_val) < (np.pi/180*180)): # Check if the one that is the cloest to the lookahead direction
                        Counter180 += 1
                        if Counter180==8:
		            print("180:"+str(diff_angle_val*57))
                            self.Is180=True
                            break 
######################################################################################

            r = L/(2*math.sin(diff_angle)) # Calculate the turning radius
            angle = 2 * math.atan(0.4/r) # Find the wheel turning radius
############
	    if (self.right_front_avg_length)<0.38 and (self.left_front_avg_length)>1.1:
                angle=0.66
                self.velocity_k=0.75
                print ("avoid right  big accident")
            elif(self.right_front_avg_length)<0.4 and (self.left_front_avg_length)>0.75:
                angle=0.32
                self.velocity_k=0.95
                print ("avoid right small accident")
            elif(self.right_front_avg_length)<0.4 and (self.left_front_avg_length)>0.75:
                angle=0.12
                self.velocity_k=0.95
                print ("avoid right small small accident")
            elif (self.left_front_avg_length)<0.4 and (self.right_front_avg_length)>1.1:
                angle=-0.66
                self.velocity_k=0.75
                print ("avoid left big accident")
            elif (self.left_front_avg_length)<0.4 and (self.right_front_avg_length)>0.75:
                angle=-0.32
                self.velocity_k=0.95
                print ("avoid left small accident")
            elif (self.left_front_avg_length)<0.4 and (self.right_front_avg_length)>0.75:
                angle=-0.12
                self.velocity_k=0.95
                print ("avoid left small small accident")
############
	    if   self.Is180 :
                angle=angle*1
            #elif self.IsVertical and self.Ismounton:
                #angle=angle*1
            elif self.IsVertical:
                angle=angle*1
            elif self.Ismounton:
	        angle=angle/1
            elif self.IsLine:
	        angle=0
                if (self.left_avg_length-self.right_avg_length)>0.6:
                    angle=0.4
                    print("adjust to right")
                elif  (self.right_avg_length-self.left_avg_length)>0.6:
                    angle=-0.4
                    print("adjust to left")
	    '''if self.left_avg_length<0.3 and self.right_avg_length>1.1:
		if abs(angle)>0.35:
		    angle=-0.35
		else:
		   angle=angle/1.4
	    if self.left_avg_length>1.1 and self.right_avg_length<0.3:
		if abs(angle)>0.35:
		    angle=0.35
		else:
		   angle=angle/1.4'''
            #if not self.IsLine:
            '''if (self.right_front_avg_length)<0.4 and (self.left_front_avg_length)>1.1:
                angle=0.7
                self.velocity_k=0.7
                print ("avoid right  big accident")
            elif(self.right_front_avg_length)<0.45 and (self.left_front_avg_length)>0.75:
                angle=0.35
                self.velocity_k=0.9
                print ("avoid right small accident")
            elif(self.right_front_avg_length)<0.50 and (self.left_front_avg_length)>0.75:
                angle=0.15
                self.velocity_k=0.95
                print ("avoid right small small accident")
            elif (self.left_front_avg_length)<0.4 and (self.right_front_avg_length)>1.1:
                angle=-0.7
                self.velocity_k=0.7
                print ("avoid left big accident")
            elif (self.left_front_avg_length)<0.45 and (self.right_front_avg_length)>0.75:
                angle=-0.35
                self.velocity_k=0.9
                print ("avoid left small accident")
            elif (self.left_front_avg_length)<0.50 and (self.right_front_avg_length)>0.75:
                angle=-0.15
                self.velocity_k=0.95
                print ("avoid left small small accident")'''
                #else:
                 #   angle=0
############
            angle = np.clip(angle, -self.max_angle+0.2, self.max_angle-0.2) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
            angle = (0 if abs(angle) < 0.08 else angle)
	    angle = (1-self.P_Angle)*Static_Angle + angle*self.P_Angle
            VELOCITY = self.speed_control(angle, self.MIN_VELOCITY, self.MAX_VELOCITY)
            Static_Velocity = VELOCITY
	    Static_Angle = angle
            #print("Static_Velocity = " + str(Static_Velocity))
            #if self.IsLine:
            #    angle=0
            # Write the Velocity and angle data into the ackermann message
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = VELOCITY
            ackermann_control.drive.steering_angle = angle
            ackermann_control.drive.steering_angle_velocity = self.steering_velocity   
        else:
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0
        
        self.navigation_input.publish(ackermann_control)
    
    # Computes the Euclidean distance between two 2D points p1 and p2
    def dist(self, p1, p2):
	try:
		return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
	except:
		return 0.5
###############################################################################################################################################################################
    # Compute the angle between car direction and goal direction
    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1, v2)
        sin_ang = LA.norm(np.cross(v1, v2))
        return np.arctan2(sin_ang, cos_ang) 
###############################################################################################################################################################################
    # Control the speed of the car within the speed limit
    def speed_control(self, angle, MIN_VELOCITY, MAX_VELOCITY):
        # Assume the speed change linearly with respect to yaw angle
        global Static_Velocity
        if self.Low_Speed_Mode:
            Velocity = 0.4
        else:
            k = (MIN_VELOCITY - MAX_VELOCITY)/self.max_angle + 0.4
            Velocity = k * abs(angle) + MAX_VELOCITY
	if self.reach_goal:
	    return 0
	if self.Ismounton:
	    Velocity = Velocity / 1.09
            #self.Ismounton=False
	if self.IsLine:
            Velocity=-0.4*abs(angle)+MAX_VELOCITY
        if self.Is180:
            Velocity = Velocity / 1.17
	if self.IsVertical:
	    Velocity= Velocity/1.28
	if self.IsBaffle:
	    Velocity = Velocity /1.5
        Velocity =Velocity*self.velocity_k
        self.velocity_k=1
            #self.IsLine=False
	#return Velocity
        #return Velocity*self.Porprotion+self.Static_Velocity*(1-self.Porprotion)
	if Velocity<0.25:
	    Velocity = 0.25
	if Velocity>Static_Velocity+MAX_VELOCITY/3:
            return Velocity*self.Porprotion_Min+Static_Velocity*(1-self.Porprotion_Min)
        elif Velocity>Static_Velocity:
            return Velocity*self.Porprotion+Static_Velocity*(1-self.Porprotion)
        else:
            return Velocity*self.Porprotion_Min+Static_Velocity*(1-self.Porprotion_Min)
        #return Velocity>Static_Velocity ? Velocity*self.Proportion+Static_Velocity*(1-self.Porprotion) : Velocity
###############################################################################################################################################################################
    
if __name__ == "__main__":

    rospy.init_node("pursuit_path")
    following_path()
    rospy.spin()
