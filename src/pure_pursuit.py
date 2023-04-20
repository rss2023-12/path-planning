#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    
    def __init__(self):
        self.current_pose = None
        self.init_sub = rospy.Subscriber('/initialpose', PosewithCovarianceStamped, self.init_callback, queue_size=1)
        
        self.odom_topic       = rospy.get_param("~odom_topic", "/odom")
        self.lookahead        = 0.5
        self.speed            = 10
        self.wheelbase_length = 0.325
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.pose_sub = rospy.Subscriber(self.odom_topic, PoseArray, self.update_pose_callback, queue_size=1)
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)


    def initialize_callback(self,msg):
        
        angle = tf.transformations.euler_from_quaternion(msg.poses.orientation)
        self.current_pose = (msg.poses.position.x msg.poses.position.y angle)
    def update_pose_callback(self, msg):
        '''
        literally just update the car's pose
        '''
        if self.current_pose is None:
            return
        angle = tf.transformations.euler_from_quaternion(msg.poses.orientation)
        self.current_pose = (msg.poses.position.x, msg.poses.position.y, angle)
        rospy.logerr(msg)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        L_1 = self.lookahead
        L = self.wheelbase_length
        '''
        drive_cmd = AckermannDriveStamped()
        np.linalg.norm(self.trajectory.points, axis=1)
        
        rospy.logerr(self.trajectory.distances)
        rospy.logerr(self.trajectory.points)
        eta = self
        drive_cmd.drive.steering_angle = -np.tan(2*L*np.sin(eta)/(L_1));
        drive_cmd.drive.steering_angle_velocity = 1 #Needs to be set
        drive_cmd.drive.speed = self.speed
        L_1 = self.lookahead
        L = self.wheelbase_length
        
        '''
        while True:
            x_car, y_car, theta_car = self.current_pose
            rospy.logerr(self.current_pose)
            x_car = float(x_car)
            y_car = float(y_car)
            theta_car = float(theta_car)
            #####################################################################
            # find last and next points in trajectory to define line to intersect
            #####################################################################
            t_points = self.trajectory.points
    
            distances_from_car = np.array([((point[0]-x_car)**2+ (point[1]-y_car)**2)**0.5 for point in self.trajectory.points])
            
            rospy.logerr("Distances from car")
            rospy.logerr(distances_from_car)
            min_dist = np.amin(distances_from_car)
            min_dist_ind = np.where(distances_from_car == min_dist)
            min_dist_ind = int(min_dist_ind[0])
            first_point = t_points[min_dist_ind]
            # check if this is point is in front of the car or behind it
            # the second point should be the opposite so that both points
            # surround the car
            
            car_dir = np.array([np.cos(theta_car), np.sin(theta_car)])              # vector of car's orientation
            path_angle = np.arctan2((first_point[1]-y_car), (first_point[0]-x_car))
            path_dir = np.array([np.cos(path_angle), np.sin(path_angle)])           # vector from car to closest point
            in_front = np.dot(car_dir, path_dir) > 0            # positive dot product indicative of car facing towards closest point found
            second_point_ind = min_dist_ind +  1 if not in_front else min_dist_ind - 1
            if len(t_points) == second_point_ind:
                drive_cmd = AckermannDriveStamped()
                drive_cmd.drive.speed = 0
                self.drive_pub.publish(drive_cmd)
                rospy.logerr("stopping")
                return
            second_point = t_points[second_point_ind]
    
            #####################################################################
            # Define line to intersect and find point of intersection with car
            # radius.
            #####################################################################
            x_offset, y_offset = -x_car, -y_car   # for easier math
            p1_x, p2_x, p1_y, p2_y = first_point[0] + x_offset, second_point[0] + x_offset, first_point[1] + y_offset, second_point[1] + y_offset
    
            a = L_1
            m = (p2_y - p1_y)/(p2_x-p1_x)
            c = p1_y - m*p1_x
            D = (2*m*c)**2-4*(1+m**2)*(c**2-a**2)
            res1 = -((2*m*c)**2 + np.sqrt(D))/(2*(1 + m**2))
            res2 = -((2*m*c)**2 - np.sqrt(D))/(2*(1 + m**2))
            x1_int, x2_int = res1 - x_offset, res2 - x_offset   # actual x coordinates of intersections
            y1_int, y2_int = m*x1_int + c - y_offset, m*x2_int + c - y_offset
            
            #####################################################################
            # choose a point to follow (the one most closely aligned) and 
            # calculate required eta value to get there
            #####################################################################
            
            # check intersection point 1 to see if it is ahead
            int1_angle = np.atan2((y1_int - y_car), (x1_int - x_car))
            int1_dir = np.array([np.cos(int1_angle), np.sin(int1_angle)])
            ahead = np.dot(car_dir, int1_dir) > 0
            pursue_point = np.array([x1_int, y1_int]) if ahead else np.array([x2_int, y2_int])
    
            v_ref = np.array([1, 0])    # reference vector which is horizontal in the world frame
            pursue_angle = np.atan2((pursue_point[1] - y_car), (pursue_point[2] - x_car))
            pursue_dir = np.array([np.cos(pursue_angle), np.sin(pursue_angle)])
            eta = np.acos(np.dot(v_ref, pursue_dir)/(a))        # a is L_1 which is the lookahead radius
    
            #####################################################################
            # compose drive command
            #####################################################################
    
            drive_cmd = AckermannDriveStamped()
            #rospy.logerr(self.trajectory.distances)
            #rospy.logerr(self.trajectory.points)
    
            drive_cmd.drive.steering_angle = np.atan2(2*L*np.sin(eta), L_1)
            drive_cmd.drive.steering_angle_velocity = 0
            drive_cmd.drive.speed = self.speed
            self.drive_pub.publish(drive_cmd)


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
