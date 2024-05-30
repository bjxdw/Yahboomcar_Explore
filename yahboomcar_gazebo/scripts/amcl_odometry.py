#!/usr/bin/env python
#coding=utf-8
'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
import tf

class OdometryNode:
    # Set publishers
    #pub_odom = rospy.Publisher('/my_amcl/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.pub_odom = rospy.Publisher('/my_amcl/odom', Odometry, queue_size=1)
        self.listener = tf.TransformListener()  #定义listener  一旦定义listener  ，他就开始接受信息，并且可以缓冲10S.
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        #self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        #rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        '''# Find the index of the racecar
        try:
            arrayIndex = msg.name.index('racecar::base_footprint')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rospy.Time.now()'''
        #listener = tf.TransformListener()  #定义listener  一旦定义listener  ，他就开始接受信息，并且可以缓冲10S.
        while not rospy.is_shutdown(): 
            try:
                (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            #angular = 4 * math.atan2(trans[1], trans[0])
            #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            #print(angular)
            #print(linear)
            print(trans)
            print(rot)

    def timer_callback(self, event):
        self.last_recieved_stamp = rospy.Time.now()
        if self.last_recieved_stamp is None:
            return
        if not rospy.is_shutdown(): 
            try:
                (self.last_received_pose,self.last_received_twist) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return
        #print(self.last_received_pose)
        #print(self.last_received_twist)
        data = "pose: "+str(self.last_received_pose) + ","+"twist: "+ str(self.last_received_twist)
	#rospy.loginfo(data)
        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'map'
        cmd.child_frame_id = 'base_link' # This used to be odom
        
        cmd.pose.pose.position.x = self.last_received_pose[0]
        cmd.pose.pose.position.y = self.last_received_pose[1]
        cmd.pose.pose.position.z = self.last_received_pose[2]

        cmd.pose.pose.orientation.x = self.last_received_twist[0]
        cmd.pose.pose.orientation.y = self.last_received_twist[1]
        cmd.pose.pose.orientation.z = self.last_received_twist[2]
        cmd.pose.pose.orientation.w = self.last_received_twist[3]

        print(cmd)
        self.pub_odom.publish(cmd)

        '''tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)'''

# Start the node
if __name__ == '__main__':

    rospy.init_node("amcl_odometry_node")
    
    node = OdometryNode()
    rospy.spin()
