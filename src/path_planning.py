#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

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

        # params
        self.graph_resolution = 0.1 # meters per cell

        # states
        self.to_update_graph = False
        self.map = None
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.graph = None # as a 2d numpy array
        self.goal = None # as an index into 2d numpy array
        self.start = None # as an index into 2d numpy array

        while True:
            self.main()

    def main(self):
        if self.to_update_graph and self.map and self.goal and self.start:
            self.to_update_graph = False

            # create graph from map, start, and goal
            self.graph = self.map # TODO rewrite if you want a graph of a different resolution

            # plan path
            parent = self.find_shortest_path(self.graph, self.start, self.goal)
            self.plan_path(self.graph, self.start, self.goal, parent)


    def map_cb(self, msg):
        """
        Callback for map. Updates the new map
        """
        self.to_update_graph = True
        # TODO update map


    def odom_cb(self, msg):
        """
        Callback for odometry. Updates the current pose to a cell index in the graph
        """
        self.to_update_graph = True
        # TODO convert pose to cell index


    def goal_cb(self, msg): # msg is a PoseStamped
        """
        Callback for goal pose. Updates the new goal pose to a cell index in the graph
        """
        self.to_update_graph = True
        # TODO convert pose to cell index



    def plan_path(self, graph, start, goal, parent):
        """
        Creates a LineTrajectory object and publishes it to the /trajectory/current topic.
        """
        # TODO form a new trajectory from the parent values

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    
    def find_shortest_path(self, graph, start, goal):
        """
        Finds the shortest path from start to goal in the graph.
        Returns a set of parent values for each node in the graph.
        """
        # TODO
        parent = {}
        return parent


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
