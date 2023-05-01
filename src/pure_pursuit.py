#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf
import math


from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    
    def __init__(self):
        self.init = False
        self.odom_topic       = rospy.get_param("~odom_topic", "/pf/pose/odom")
        self.lookahead        = rospy.get_param("/lookahead",1.5) # 0.4
        self.speed            =1.25
        self.wheelbase_length = 0.325
        
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.update_pose_callback, queue_size=1)
        
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        #self.error_pub = rospy.Publisher("/drive_error",Float32, queue_size = 1)

    def update_pose_callback(self, msg):
        '''
        literally just update the car's pose
        '''
        angle = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.current_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, angle])
        #rospy.logerr(self.current_pose)
        L = self.wheelbase_length
        
        drive_cmd = AckermannDriveStamped()
        
        if not self.init:
            return
        
        #Get trajectory
        t_points = self.trajectory.points
        
        # Get Distances       
        x_car, y_car, theta_car = self.current_pose
        x_car = float(x_car)
        y_car = float(y_car)
        theta_car = float(theta_car)
        #####################################################################
        # find last and next points in trajectory to define line to intersect
        #####################################################################
        distances_from_car = np.array([((point[0]-x_car)**2+ (point[1]-y_car)**2)**0.5 for point in self.trajectory.points])
        
        min_dist_ind = np.argmin(distances_from_car)
        first_point = t_points[min_dist_ind]
        '''
        car_dir = np.array([np.cos(theta_car), np.sin(theta_car)])              # vector of car's orientation
        path_angle = np.arctan2((first_point[1]-y_car), (first_point[0]-x_car))
        path_dir = np.array([np.cos(path_angle), np.sin(path_angle)])           # vector from car to closest point
        in_front = np.dot(car_dir, path_dir) > 0            # positive dot product indicative of car facing towards closest point found
        second_point_ind = min_dist_ind +  1 if not in_front else min_dist_ind - 1
        if len(t_points) == second_point_ind:
            drive_cmd.drive.speed = 0
            self.drive_pub.publish(drive_cmd)
            rospy.logerr("stopping end")
            return
        '''
        second_point_ind = min_dist_ind +  1
        second_point = t_points[second_point_ind]
        
        min_dist = self.shortest_distance_getter(first_point, second_point)
        #self.error_pub.publish(min_dist)
        rospy.logerr(min_dist)
        intersect_val = None
        
        intersect = self.circ_intersector(first_point,second_point)
        '''
        for i in range(min_dist_ind, len(t_points)-1):
            intersect = self.circ_intersector(t_points[i], t_points[i+1])
        '''
        if not (intersect is None): 
            intersect_val = intersect
        
        
        if intersect_val is None: 
            #drive_cmd.drive.speed = 0
            #self.drive_pub.publish(drive_cmd)
            rospy.logerr("Slow down, bad intersect")
            
        else:
            V = np.subtract(intersect_val, np.array([x_car, y_car]))
        
    
            angle = np.arctan2(V[1], V[0]) - self.current_pose[2] #angle wrt int
    
            distance = math.sqrt(V[0]**2 + V[1]**2) #distance wrt intersect
    
            delta = math.atan(2*L*np.sin(angle)/distance) #pure pursuit
    
            drive_cmd.drive.steering_angle = delta 
            drive_cmd.drive.speed = self.speed 
            self.drive_pub.publish(drive_cmd) 
    
        return
        
        
        

    def shortest_distance_getter(self,p1,p2):
        """
        Given a start and end point, we create a line connecting the two and find the shortest distance to a point on that line
        """
        x_car, y_car = self.current_pose[0], self.current_pose[1]

        p1x, p1y = p1[0], p1[1]
        p2x, p2y = p2[0], p2[1]

        x12, y12 = (p2x-p1x, p2y-p1y)
        x1c, y1c = (x_car-p1x, y_car-p1y)        
        x2c, y2c = (x_car-p2x, y_car-p2y)

        #Dot product of the line and the car distance is negative if point 1 is closest
        if (x12*x1c+y12*y1c) < 0: 
            return math.sqrt(x1c**2 + y1c**2)
        #Dot product of the line and the car distance is positive if point 2 is closest
        if (x12*x2c+y12*y2c) > 0: 
            return math.sqrt(x2c**2 + y2c**2)

        #Solve for intermediate distance with the area of a triangle where a = 0.5bh
        A = np.array([[x_car, y_car, 1],[p1x, p1y, 1], [p2x, p2y, 1]]) #determinant is 2*A
        A = abs(np.linalg.det(A))

        b = math.sqrt((p2x - p1x)**2 + (p2y - p1y)**2)

        h = A / b

        return h


    def normalize(self, V):
        return np.multiply(V,np.array([1/math.sqrt(np.dot(V, V)), 1/math.sqrt(np.dot(V, V))])) #Unit vector

    def circ_intersector(self, p1, p2):

        """
        We must find where the line segment intersects with a circle centered at the car with r = look ahead distance
        """
        p1 = np.array(p1)
        p2 = np.array(p2)
        #This is the stuff from the guide
        Q = np.array([self.current_pose[0],self.current_pose[1]]) #Circle center
        r = self.lookahead #Radius
        V = np.subtract(p2,p1) #Vector along line segment

        #Solving quadratic
        a = np.dot(V,V)
        b = 2 * np.dot(V, np.subtract(p1,Q))
        c = np.dot(p1,p1) + np.dot(Q,Q) - 2 * np.dot(p1,Q) - r**2

        discriminant = b**2 - 4*a*c
        if discriminant < 0:             
            #rospy.logerr("bad discriminant")
            return None

        t_1 = (-b+math.sqrt(discriminant))/(2*a)
        t_2 = (-b-math.sqrt(discriminant))/(2*a)

        if not ((0 <= t_1 <= 1) or (0 <= t_2 <= 1)): 
            return None

        elif not 0 <= t_1 <= 1: #If only this t2 point
            return np.add(p1, np.multiply(np.array([t_2, t_2]),V))

        elif not 0 <= t_2 <= 1: #If only this t1 point
            return np.add(p1, np.multiply(np.array([t_1,t_1]), V))

        else: 
            soln1 = np.add(p1, np.multiply(np.array([t_1,t_1]), V)) #Solution 1
            soln2 = np.add(p1, np.multiply(np.array([t_2,t_2]), V)) #Solution 2
            V1 = np.subtract(soln1, Q)  #Vector from car to first circle point
                           
            V1 = self.normalize(V1)
            
            V2 = np.subtract(soln2, Q) #Vector from car to second circle point

            V2 = self.normalize(V2)
            car_norm = np.array([np.cos(self.current_pose[2]), np.sin(self.current_pose[2])])

            #These dot products help us figure out which soln to turn to
            d1 = np.dot(car_norm, V1) 
            d2 = np.dot(car_norm, V2)

            # return soln1 if d1 > d2 else soln2 , returning solution with smallest ngle diff.
            
            return soln1 if d1 ==max(d1,d2) else soln2 
        
        
        
        
        
    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.init = True
        
        return
        


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
