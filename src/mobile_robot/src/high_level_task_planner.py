#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, PolygonStamped, Point
import time, os
from utils import LineTrajectory
from visualization_msgs.msg import Marker, MarkerArray


class HighLevelTaskPlanner(object):


    def __init__(self):
        self.trajectory_delta = LineTrajectory("/delta_trajectory")
        self.trajectory_backup = LineTrajectory("/backup_trajectory")
        self.trajectory_waiter = LineTrajectory("/waiter_trajectory")
        self.trajectory_finish = LineTrajectory("/finish_trajectory")

        self.delta_pts = [(1.93, 0.0), (1.94, 1.13)]
        self.backup_pts = [(1.94, 1.13), (1.93, 0.8)]
        self.waiter_pts = [(1.93, 0.8), (1.94, 1.13), (1.14, 1.95)]
        self.finish_pts = [(1.14, 1.95), (0.73, 1.04),(1.07, 0.29),(0.10, 0.271)]

        self.delta_speed = 1
        self.backup_speed = -1
        self.waiter_speed = 1
        self.finish_speed = 1

        '''
        Insert appropriate subscribers/publishers here

        '''
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PolygonStamped, queue_size=10)
        self.trajectory_points = rospy.Publisher("/traj_pts", Marker, queue_size=20)

        self.create_trajectory(self.trajectory_delta, self.delta_pts, self.delta_speed)
        self.create_trajectory(self.trajectory_backup, self.backup_pts, self.backup_speed)
        self.create_trajectory(self.trajectory_waiter, self.waiter_pts, self.waiter_speed)
        self.create_trajectory(self.trajectory_finish, self.finish_pts, self.finish_speed)

        self.current_traj_index = 0
        self.trajectories = [self.trajectory_delta, self.trajectory_backup, self.trajectory_waiter, self.trajectory_finish]


    def publish_trajectory(self):
        traj = self.trajectories[self.current_traj_index]
        self.traj_pub.publish(traj.toPolygon())
        self.current_traj_index += 1

    def create_trajectory(self, traj, pts, speed):
        for x, y in pts:
            p = Point()
            p.x = x
            p.y = y
            traj.addPoint(p)
            traj.speed_profile.append(speed)

    def clicked_pose(self, point):
        self.publish_trajectory()
        print "yay"

    

if __name__=="__main__":
    rospy.init_node("high_level_task_planner")
    hltp = HighLevelTaskPlanner()
    rospy.spin()
