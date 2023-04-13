#!/usr/bin/env python

import rospy
import numpy as np
import scipy.signal
import math
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

#
# TODO (ideas)
# Varied Resolution = resolution of graph is different than resolution of map
# Shortest Path Chain Reduction = reduce the chain of nodes if they are in a straight line
# Dynamic Subgraph Node Grouping = veried resolution in graph (higher around start, lower otherwise) 
# (!) Consider Bot Orientation in building the trajectory
#

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        

        # states
        self.to_update_graph = False
        self.map = None
        self.lowres_map = None # downsampled version of map as an OccupancyGrid
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.graph = None # as a set of vacant cells in lowres_map
        self.goal = None # as a PoseStamped in map
        self.start = None # as a PoseStamped in map
        
        while True:
            self.main()

    def main(self):
        if self.to_update_graph and self.map and self.goal and self.start:
            self.to_update_graph = False

            # create graph from map, start, and goal
            self.graph = self.build_graph(self.lowres_map)

            # TODO (for Varied Resolution) convert start and goal to cell indices in graph of different resolution
            start = self.xy_to_uv(self.start.position.x, self.start.position.y)
            goal = self.xy_to_uv(self.goal.position.x, self.goal.position.y)

            # plan path
            #parent = self.find_shortest_path(self.graph, start, goal)
            # TODO (for Shortest Path Chain Reduction) collapse parent chain
            #self.plan_path(self.graph, start, goal, parent)


    def map_cb(self, msg):
        """
        Callback for map. Updates the new map
        """
        self.to_update_graph = True
        self.map = msg
        self.lowres_map = self.make_lowres_map(msg)
        print('new map made', self.lowres_map.info)

    def odom_cb(self, msg):
        """
        Callback for odometry. Updates the current pose to a cell index in the graph
        """
        self.to_update_graph = True
        self.start = msg.pose


    def goal_cb(self, msg):
        """
        Callback for goal pose. Updates the new goal pose to a cell index in the graph
        """
        self.to_update_graph = True
        self.goal = msg.pose



    def plan_path(self, graph, start, goal, parent):
        """
        Creates a LineTrajectory object and publishes it to the /trajectory/current topic.
        """
        # form PoseArray from parent values
        self.trajectory = LineTrajectory("/planned_trajectory")
        p = goal
        while p != start:
            # TODO (for Varied Resolution) convert p to a PoseStamped in map
            self.trajectory.addPoint(self.uv_to_xy(p[0], p[1]))
            p = parent[p]

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    def uv_to_xy(self,u,v):
        '''
        inputs must not be np array
        '''

        u *= self.map.info.resolution
        v *= self.map.info.resolution
        
        quaternion = [
                self.map.info.origin.orientation.x,
                self.map.info.origin.orientation.y,
                self.map.info.origin.orientation.z,
                self.map.info.origin.orientation.w
                ]

        translation = [
                self.map.info.origin.position.x,
                self.map.info.origin.position.y,
                self.map.info.origin.position.z,
                0
                ]
        
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
        a = np.array([u,v,0,0]).reshape(4,1)
        xy = np.matmul(rotation_matrix, a) + np.array(translation).reshape(4,1)

        return  xy.reshape(1,4)[0][:2]


    def xy_to_uv(self, x,y):
        '''
        inputs must not be np array
        '''

        quaternion = [
                self.map.info.origin.orientation.x,
                self.map.info.origin.orientation.y,
                self.map.info.origin.orientation.z,
                self.map.info.origin.orientation.w
                ]

        translation = [
                self.map.info.origin.position.x,
                self.map.info.origin.position.y,
                self.map.info.origin.position.z,
                0
                ]
 
        a = np.array([x,y,0,0]).reshape(4,1)
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
        b = np.linalg.inv(rotation_matrix)
        uv = np.matmul(b, a - np.array(translation).reshape(4,1) ) / self.map.info.resolution
        
        return uv.reshape(1,4)[0][:2]


    def find_shortest_path(self, graph, start, goal):
        """
        Finds the shortest path from start to goal in the graph.
        Returns a set of parent values for each node in the graph.
        """
        # TODO: UPDATE PARENT POINTERS

        #queue for holding nodes to look at, seen for seen nodes
        queue = { (start, self. dist_between_points(start, goal)), }
        seen = {start,}
        parents = {start: None}  
        
        height  = graph.info.height
        width  = graph.info.width

        #edge relation for each node
        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1, -1), (1, -1), (-1, 1)]
    
        while queue:

            #pop off the queue, get node values i and j
            node, _ = queue.pop()
            i,j = node

            #base end, compute path back to start and return
            if (i,j) == goal:
                path = []
                p = parents[(i,j)]
                while p:
                    path.append(p)
                    p = parents[p]
                return path[::-1]


            #for each neighbor, check if in bounds and not seen before, then add to seen. If not dark, add to the queue
            for x,y in neighbors:
                _x, _y = i+x, j+y
                if height > _x  >=0 and width > _y >= 0 and (_x,_y) not in seen:
                    seen.add( (_x,_y) )
                    if graph.data[_x][_y] < 30:
                       queue.add( ((_x, _y), self.dist_between_points((_x,_y), goal)) )
                       parents[(_x, _y)] = (x,y)

            seen.add(node)


        return None

    def dist_between_points(self, A, B):
        return ((1.0*A[0]-B[0])**2 + (1.0*A[1]-B[1])**2)**0.5
    


    def make_lowres_map(self, map):
        """
        Downsamples the map by a factor "stride".
        """
        og_map_data = np.array(map.data).reshape(map.info.width, map.info.height)

        # params
        filt = np.ones((7, 7)) # 0.05m x 3 = 15 cm buffer on each side
        stride = 2

        # convolve
        stride_conv = lambda arr, arr2, s: scipy.signal.convolve2d(arr, arr2[::-1, ::-1], mode='valid')[::s, ::s]
        new_map_data = stride_conv(og_map_data, filt, stride)

        new_map = OccupancyGrid()
        new_map.data = new_map_data.flatten()
        new_map.info = map.info
        new_map.info.width = new_map_data.shape[0]
        new_map.info.height = new_map_data.shape[1]
        new_map.info.resolution = map.info.resolution * stride

        return new_map
    

    def build_graph(self, map):
        """
        Builds a graph of vacant cells from the map data.
        """
        graph = set()
        for i in range(map.info.height):
            for j in range(map.info.width):
                if map.data[i*map.info.width + j] == 0:
                    graph.add((i,j))
        return graph
  

    def map_thicken(self,OG):
        '''
        Creates a graph out of map data. First adds a thickening, finds all open space, then downsamples into dict graph representation"
        '''

        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1, -1), (1, -1), (-1, 1)]
        height  = OG.info.height
        width  = OG.info.width

        graph = {}


        borders = set()
        for i in range(height):
            for j in range(width):
                if OG.data[i*width + j] == 0:
                    for x,y in neighbors:
                        _x, _y = i+x, j+y
                        if height > _x  >=0 and width > _y >= 0 and OG.data[_x*width + _y] != 0:
                            borders.add( (i,j) )
                            break

        wall_thickness = 0.5
        pix_thickness = int(1/OG.info.resolution*wall_thickness)
        coords = []
        for x in range(pix_thickness):
            coords += [(x,pix_thickness-x),(-x,pix_thickness-x),(-x,-(pix_thickness-x)),(x,-(pix_thickness-x))]
            
        print('borders found')

        bad = set()
        #add thicken circles
        for i,j in borders:
            for x,y in coords:
                _x, _y = i+x, j+y
                bad.add( (_x, _y) )

        print('new borders made')

        #downsample graph by val
        meters_per_pixel_ideal = 0.2
        down_sample = int(np.rint(meters_per_pixel_ideal/OG.info.resolution))
        x = 0
        y = 0
        while x < height:
            while y < width:
                valid_node  = True
                for i in range(x, x+down_sample):
                    for j in range(y, y+down_sample):
                        if (i,j) in bad or (height > i  >=0 and width > j >= 0 and OG.data[i*width + j] != 0):
                            valid_node  = False
                            break
                    if not valid_node:
                        break
                if valid_node:
                    _x = (1.0*down_sample*(2*x+1) - 1)/2 
                    _y = (1.0*down_sample*(2*y+1) - 1)/2
                    graph[(_x, _y)] = None
                            
                y += down_sample
            y = 0
            x  += down_sample

        return graph, (height, width)
            
if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
