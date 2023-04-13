#!/usr/bin/env python

import rospy
import numpy as np
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
        self.map = None # as an OccupancyGrid
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.graph = {
            "data": None,
            "info": {
                "resolution": 0.1, # meters per cell
            }
        } # as an OccupacyGrid
        self.goal = None # as a PoseStamped in map
        self.start = None # as a PoseStamped in map
        
        while True:
            self.main()

    def main(self):
        if self.to_update_graph and self.map and self.goal and self.start:
            self.to_update_graph = False

            # create graph from map, start, and goal
            self.graph = self.map # TODO (for Varied Resolution) update resolution, height, width

            # TODO (for Varied Resolution) convert start and goal to cell indices in graph of different resolution
            start = (self.start.position.x, self.start.position.y)
            goal = (self.goal.position.x, self.goal.position.y)

            # plan path
            parent = self.find_shortest_path(self.graph, start, goal)
            # TODO (for Shortest Path Chain Reduction) collapse parent chain
            self.plan_path(self.graph, start, goal, parent)


    def map_cb(self, msg):
        """
        Callback for map. Updates the new map
        """
        self.to_update_graph = True
        self.map = self.map_thicken(msg)


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
            self.trajectory.addPoint(p)
            p = parent[p]

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


    def find_shortest_path(self, graph, start, goal):
        """
        Finds the shortest path from start to goal in the graph.
        Returns a set of parent values for each node in the graph.
        """
        # TODO: UPDATE PARENT POINTERS

        #queue for holding nodes to look at, seen for seen nodes
        queue = {start,}
        seen = {start,}
        parents = {start: None}
        dists = {start: self.dist_between_points(start, goal) }     
        
        height  = graph.info.height
        width  = graph.info.width

        #edge relation for each node
        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1, -1), (1, -1), (-1, 1)]
    
        while queue:

            #pop off the queue, get node values i and j
            i,j = queue.pop()

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
                       queue.add((_x, _y))
                       dists[(_x,_y)] = self.dist_between_points((_x,_y), goal)
                       parents[(_x, _y)] = (x,y)

            seen.add(node)


        return None

    def dist_between_points(self, A, B):
        return ((1.0*A[0]-B[0])**2 + (1.0*A[1]-B[1])**2)**0.5


    def map_thicken(self,OG):
        dark = {}
        for i in OG.info.height:
            for j in OG.info.width:
                if OG.data[i][j] > 30:
                    dark.add((i,j))


        wall_thickness = 0.5
        pix_thickness = math.ciel(1/OG.info.resolution*wall_thickness)
        coords = []
        for x in range(pix_thickness):
            coords += [(x,pix_thickness-x),(-x,pix_thickness-x),(-x,-(pix_thickness-x)),(x,-(pix_thickness-x))]
            
        for i,j in dark:
            for x,y in coords:
                if OG.info.height > i+x >= 0 and OG.info.width > j+y >= 0:
                    OG.data[i+x][j+y] = 100
        return OG
            
if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
