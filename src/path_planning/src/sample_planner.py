import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, PolygonStamped, Point
import time, os
from utils import LineTrajectory
from visualization_msgs.msg import Marker, MarkerArray
import path_planner
import graph_build as gb

class SamplePlanner(PathPlanner)
    def __init__(self):
        self.goal_sample_rate = 20
        self.drive_dist = 2
        self.obstacle_buffer = 0.2
        super(SamplePlanner, self).__init__()
    def find_path(self):
        #initializing graph with start node
        g = gb.Graph()
        sn = gb.Node(self.init_pose.x, self.init_pose.y, parent = None)
        g.add_node(sn)
        reached_goal = False
        i = 0
        #sets a limit on run time
        while i < 10**10 and not reached_goal:
            #goal biasing
            if i%self.goal_sample_rate == 0:
                sample_point = (self.goal.x, self.goal.y, 0)
            else:
                sample_point = self.sample_map()
                #get neighbor
            neighbor = self.nearest_neighbor(sample_point, graph)
            #get end point
            end_point = self.steer(neighbor.state, sample_point, self.drive_dist)
            #checking for obstacles
            if check_obstacle(neighbor.state, end_point, self.obstacle_buffer):
                #adding the edge
                en = gb.Node(end_point[0], end_point[1], parent = neighbor)
                g.add_edge(neighbor, en, node_dist(en, neighbor))
                #checking for completion
                if self.at_goal(end_point):
                    reached_goal = True
            i+=1
        #return path if one is found
        if reached_goal:
            path = extract_path(g, en)
            return path
        return None
    def sample_map(self):
        return #tuple (x, y, theta)
    def nearest_neighbor(self, sample_point, graph):
        return #node
    def steer(self, start_point, end_point, dist):
        return #point tuple (x, y, theta)
    def check_obstacle(self, start_point, end_point, buffer):
        return #boolean true if no obstacles within a buffer width rectangle
    def at_goal(self, point, margin):
        return #boolean if you're at the goal within a margin
    def node_dist(self, node1, node2):
        return #euclidean distance between two nodes
    def extract_path(self, graph, goal_node):
        return #trajectory traced back from goal_node
