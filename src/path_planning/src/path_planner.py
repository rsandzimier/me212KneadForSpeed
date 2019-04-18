#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, PolygonStamped, Point
import time, os
from utils import LineTrajectory
from visualization_msgs.msg import Marker, MarkerArray

class PathPlanner(object):
    """ Listens for points published by RViz and uses them to build a trajectory.
    """
    def __init__(self):

        # Get parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.goal_topic = rospy.get_param("~nav_goal")
        self.init_pose_topic = rospy.get_param("~initial_pose")
        self.trajectory_topic = rospy.get_param("~trajectory_topic")
        self.traj_pts_topic = rospy.get_param("~trajectory_points_topic")

        # Initialize class variables
        self.trajectory = LineTrajectory("/built_trajectory")
        self.map_info = None
        self.map_data = None
        self.init_pose = None
        self.goal = None
        self.start = None
        self.end = None
        self.data_points = []
        self.count = 0

        # Initialize subscribers and publishers
        self.map_subscriber = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=10)
        self.initial_pose_subscriber = rospy.Subscriber(self.init_pose_topic, PoseWithCovarianceStamped, self.init_pose_callback, queue_size=10)
        self.nav_goal_subscriber = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=10)
        self.trajectory_publisher = rospy.Publisher(self.trajectory_topic, PolygonStamped, queue_size=10)
        
    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data)
        self.map_data = self.map_data.reshape(self.map_info.height, self.map_info.width)

        x_origin = self.map_info.origin.position.x
        y_origin = self.map_info.origin.position.y
        scale = self.map_info.resolution

        max_width = x_origin + self.map_info.width * scale
        max_height = y_origin + self.map_info.height * scale
        self.obstacleList = np.argwhere(self.map_data > 10)
        self.obstacleList = self.obstacleList.tolist()
        for obstacle in self.obstacleList:
            temp_x = obstacle[0]
            temp_y = obstacle[1]
            obstacle[0] = temp_y * scale + x_origin
            obstacle[1] = temp_x * scale + y_origin

    def init_pose_callback(self, msg):
        point = Point()
        point.x = msg.pose.pose.position.x
        point.y = msg.pose.pose.position.y
        self.mark_point(self.start_publisher, (0,1,0), [point])
        self.initial_pose = (point.x, point.y)
        if self.nav_goal:
            self.make_path()

    def goal_callback(self, msg):
        point = Point()
        point.x = msg.pose.position.x
        point.y = msg.pose.position.y
        self.mark_point(self.end_publisher, (1,0,0), [point])
        self.nav_goal = (point.x, point.y)
        if self.initial_pose:
            self.find_path()

    def mark_point(self, publisher, data):
        mark_pt = Marker()
        mark_pt.header.frame_id = "/map"
        mark_pt.header.stamp = rospy.Time.now()
        mark_pt.type  = mark_pt.SPHERE_LIST
        mark_pt.action = mark_pt.ADD
        mark_pt.scale.x = .5
        mark_pt.scale.y = .5
        mark_pt.scale.z= .5
        mark_pt.color.a =1.0
        mark_pt.color.r=color_tup[0]
        mark_pt.color.g = color_tup[1]
        mark_pt.color.b = color_tup[2]
        mark_pt.points = data
        subscriber.publish(mark_pt)

    def find_remaining_distance(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_path(self):
        """
        Finds eather search planning or sample planning path in respective subclasses. 
        """
        raise NotImplementedError("implement in subclass")

if __name__=="__main__":
    rospy.init_node("path_planner")
    pf = PathPlanner()
    rospy.spin()
