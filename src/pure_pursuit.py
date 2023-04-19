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
from shapely.geometry import Point, LineString

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 0.5
        self.speed            = 10
        self.wheelbase_length = 0.325
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, PoseArray, self.update_pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.current_pose = (0, 0, 0)


    def update_pose_callback(self, msg):
        '''
        literally just update the car's pose
        '''
        angle = tf.transformations.euler_from_quaternion(msg.poses.orientation)
        self.current_pose = (msg.poses.position.x, msg.poses.position.y, angle)


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
        
        x_car, y_car, theta_car = self.current_pose

        #####################################################################
        # find last and next points in trajectory to define line to intersect
        #####################################################################
        t_points = self.trajectory.points

        distances_from_car = np.array([((point[0]-x_car)**2, (point[1]-y_car)**2, (point[2]-theta_car)**2)**0.5 for point in self.trajectory.points])
        
        min_dist = np.amin(distances_from_car)
        min_dist_ind = np.where(distances_from_car == min_dist)
        first_point = t_points[min_dist_ind]
        # check if this is point is in front of the car or behind it
        # the second point should be the opposite so that both points
        # surround the car
        
        car_dir = np.array([np.cos(theta_car), np.sin(theta_car)])              # vector of car's orientation
        path_angle = np.atan2((first_point[1]-y_car), (first_point[0]-x_car))
        path_dir = np.array([np.cos(path_angle), np.sin(path_angle)])           # vector from car to closest point
        in_front = np.dot(car_dir, path_dir) > 0            # positive dot product indicative of car facing towards closest point found
        second_point_ind = min_dist_ind +  1 if not in_front else min_dist_ind - 1
        second_point = t_points[second_point_ind]

        #####################################################################
        # Define line to intersect and find point of intersection with car
        # radius.
        #####################################################################
        

        
        px, py = Point
        drive_cmd = AckermannDriveStamped()
        #rospy.logerr(self.trajectory.distances)
        #rospy.logerr(self.trajectory.points)
        eta = 1             

        drive_cmd.drive.steering_angle = np.atan2(2*L*np.sin(eta), L_1)
        drive_cmd.drive.steering_angle_velocity = 0
        drive_cmd.drive.speed = self.speed
        self.drive_pub.publish(drive_cmd)


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
