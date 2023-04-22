#!/usr/bin/env python

import rospy
import numpy as np
import tf
from functools import wraps
import scipy.signal
from heapdict import heapdict
import math
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
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
def timeit(func):
    def timeit_wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        total_time = end_time - start_time
        print('\nFunction ' +  func.__name__ + " Took " + str(total_time) + ' seconds')
        return result
    return timeit_wrapper

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
        self.stride = 1 #default of stride
        self.graph = None # as a set of vacant cells in lowres_map
        self.goal = None # as a PoseStamped in map
        self.start = None # as a PoseStamped in map
        
        while True:
            self.main()

    def main(self):
        if self.to_update_graph and self.graph and self.goal and self.start:
            self.to_update_graph = False

            # TODO (for Varied Resolution) convert start and goal to cell indices in graph of different resolution

            start = self.xy_to_uv(self.start.position.x, self.start.position.y)
            goal = self.xy_to_uv(self.goal.position.x, self.goal.position.y)
         
            
            start = self.find_closest_node(start)[::-1]
            goal = self.find_closest_node(goal)[::-1]
            
            # plan path with greedy search
            parent_path = self.greedy_search(self.graph, start, goal)
            self.plan_path(parent_path, '/greedy_trajectory')

            #plan path with DFS
            parent_path = self.DFS(self.graph, start, goal)
            self.plan_path(parent_path, '/DFS_trajectory')

            # plan path with BFS
            parent_path = self.BFS(self.graph, start, goal)
            self.plan_path(parent_path)

            # TODO (for Shortest Path Chain Reduction) collapse parent chain
                

    @timeit
    def map_cb(self, msg):
        """
        Callback for map. Updates the new map
        """
        if False:
            self.map = msg
            print(msg.info)
            #self.lowres_map = self.make_lowres_map(msg)
            self.graph = self.map_thicken(msg)
            #self.build_graph(self.lowres_map)
            print('new map made')
            #print(self.lowres_map.info)
            print(len(self.graph))
            return
        self.map = msg
        print(msg.info)
        self.lowres_map = self.make_lowres_map(msg)
        #with open('map_info.npy', 'wb') as f:
        #   np.save(f, np.array(msg.data))
        self.graph = self.build_graph(self.lowres_map)
        print('new map made')
        print(self.lowres_map.info)
        print(len(self.graph))

    def odom_cb(self, msg):
        """
        Callback for odometry. Updates the current pose to a cell index in the graph
        """
        self.start = msg.pose.pose

    def goal_cb(self, msg):
        """
        Callback for goal pose. Updates the new goal pose to a cell index in the graph
        """
        self.to_update_graph = True
        self.goal = msg.pose
        print('goal found')

    def plan_path(self, parent_path, viz_name = "/planned_trajectory"):
        """
        Creates a LineTrajectory object and publishes it to the /trajectory/current topic.
        """
        # form PoseArray from parent values
        self.trajectory = LineTrajectory(viz_name)
        for (i,j) in parent_path:
            u = (1.0*self.stride*(2*i+1)-1)/2
            v = (1.0*self.stride*(2*j+1)-1)/2
            
            # TODO (for Varied Resolution) convert p to a PoseStamped in map
            p = Point()
            p.x, p.y = self.uv_to_xy(v, u)
            self.trajectory.addPoint(p)

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

    def find_closest_node(self, (u,v)):
        '''
        finds the closest pixel node in the downsample version
        '''
        _u  = int(np.rint( ((2.0*u+1)/self.stride - 1)/2 ))
        _v = int(np.rint( ((2.0*v+1)/self.stride - 1)/2 ))

        return (_u, _v)

    @timeit
    def BFS(self, graph, start, goal):

        q = [start]
        seen = {start,}
        parents = {start: None} 
        closest = (start, self.dist_between_points(start, goal))


        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1, -1), (1, -1), (-1, 1)]

        while q:
            node = q.pop(0)
            seen.add(node)
            _ = self.dist_between_points(node, goal)
            if _ < closest[1]:
                closest = (node, _)
            i,j = node

            if node == goal:
                break

            #for each neighbor, check if not seen, then add to seen. If reachable, add to the queue
            for x,y in neighbors:
                _x, _y = i+x, j+y
                if (_x,_y) not in seen:
                    seen.add( (_x,_y) )
                    if (_x,_y) in graph:
                        q.append( (_x, _y) )
                        parents[(_x, _y)] = (i,j)
        
        #base end, compute path back to start and return
        path = [closest[0]]
        p = parents[closest[0]]
        while p:
            path.append(p)
            p = parents[p]
        return path[::-1]
    
    @timeit
    def DFS(self, graph, start, goal):

        q = [start]
        seen = {start,}
        parents = {start: None} 
        closest = (start, self.dist_between_points(start, goal))


        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1, -1), (1, -1), (-1, 1)]

        while q:
            node = q.pop(-1)
            seen.add(node)
            _ = self.dist_between_points(node, goal)
            if _ < closest[1]:
                closest = (node, _)
            i,j = node

            if node == goal:
                break

            #for each neighbor, check if not seen, then add to seen. If reachable, add to the queue
            for x,y in neighbors:
                _x, _y = i+x, j+y
                if (_x,_y) not in seen:
                    seen.add( (_x,_y) )
                    if (_x,_y) in graph:
                        q.append( (_x, _y) )
                        parents[(_x, _y)] = (i,j)
        
        #base end, compute path back to start and return
        path = [closest[0]]
        p = parents[closest[0]]
        while p:
            path.append(p)
            p = parents[p]
        return path[::-1]


    @timeit
    def greedy_search(self, graph, start, goal):
        """
        Finds the shortest path from start to goal in the graph.
        Returns a set of parent values for each node in the graph.
        """
        # TODO: UPDATE PARENT POINTERS

        #queue for holding nodes to look at, seen for seen nodes
        q = heapdict()
        q[start] = self.dist_between_points(start, goal)

        seen = {start,}
        parents = {start: None} 

        closest = (start, self.dist_between_points(start, goal))

        
        #edge relation for each node
        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1, -1), (1, -1), (-1, 1)]
    
        while q:
            
            #pop off the queue, get node values i and j
            node, _ = q.popitem()
            if _ < closest[1]:
                closest = (node, _)
            i,j = node

            if node == goal:
                break

            #for each neighbor, check if not seen, then add to seen. If reachable, add to the queue
            for x,y in neighbors:
                _x, _y = i+x, j+y
                if (_x,_y) not in seen:
                    seen.add( (_x,_y) )
                    if (_x,_y) in graph:
                        q[(_x, _y)] =  self.dist_between_points((_x,_y), goal)
                        parents[(_x, _y)] = (i,j)

            seen.add(node)


        #base end, compute path back to start and return
        path = [closest[0]]
        p = parents[closest[0]]
        while p:
            path.append(p)
            p = parents[p]
        return path[::-1]


    def dist_between_points(self, A, B):
        return ((1.0*A[0]-B[0])**2 + (1.0*A[1]-B[1])**2)**0.5
    

    def make_lowres_map(self, _map):
        """
        Downsamples the map by a factor "stride".
        """
        og_map_data = np.array(_map.data).reshape(_map.info.height, _map.info.width)

        # params
        filt = np.ones((15,15)) # 0.05m x 3 = 15 cm buffer on each side

        
        stride = 3
        self.stride = stride

        # convolve
        stride_conv = lambda arr, arr2, s: scipy.signal.convolve2d(arr, arr2, mode='same', boundary='fill')[::s, ::s]
        new_map_data = stride_conv(og_map_data, filt, stride)

        new_map = OccupancyGrid()
        new_map.data = new_map_data.flatten()
        new_map.info = MapMetaData()
        new_map.info.height = new_map_data.shape[0]
        new_map.info.width = new_map_data.shape[1]
        new_map.info.resolution = _map.info.resolution * stride

        return new_map
    

    def build_graph(self, _map):
        """
        Builds a graph of vacant cells from the map data.
        """
        graph = set()
        for i in range(_map.info.height):
            for j in range(_map.info.width):
                if _map.data[i*_map.info.width + j] == 0:
                    graph.add((i,j))
        return graph
  
    @timeit
    def map_thicken(self,OG):
        '''
        Creates a graph out of map data. First adds a thickening, finds all open space, then downsamples into dict graph representation"
        '''
        height  = OG.info.height
        width  = OG.info.width

        graph = set()

        og_map_data = np.array(OG.data).reshape(height, width)

        # params
        filt = np.ones((1,1)) # 0.05m x 3 = 15 cm buffer on each side

        # convolve
        stride_conv = lambda arr, arr2: scipy.signal.convolve2d(arr, arr2, mode='same', boundary='fill')
        new_map_data = stride_conv(og_map_data, filt)

        open_nodes = set()
        for i in range(height):
            for j in range(width):
                #if OG.data[i*width+j] == 0:
                if new_map_data[i][j] == 0:
                    open_nodes.add( (i,j) )
                if new_map_data[i][j] == 0 != OG.data[i*width+j]:
                    print('h', (i,j))

        print('Good nodes found')

        #downsample graph by val
        meters_per_pixel_ideal = 0.2
        down_sample = 2#int(np.rint(meters_per_pixel_ideal/OG.info.resolution))
        self.stride = down_sample
        x = 0
        y = 0

        while x < height:
            while y < width:
                valid_node  = True
                for i in range(x, x+down_sample):
                    for j in range(y, y+down_sample):
                        if (i,j) not in open_nodes:
                            valid_node  = False
                            break
                    if not valid_node:
                        break
                if valid_node:
                    _x = x/down_sample 
                    _y = y/down_sample
                    graph.add( (_x, _y) ) 
                y += down_sample
            y = 0
            x  += down_sample

        return graph
            
if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
