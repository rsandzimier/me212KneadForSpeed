#!/usr/bin/env python

import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, PolygonStamped, Point
import time, os
from utils import LineTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, Float32
from collections import deque


class HighLevelTaskPlanner(object):


    def __init__(self):
        self.trajectory_delta = LineTrajectory("/delta_trajectory")
        self.trajectory_backup = LineTrajectory("/backup_trajectory")
        self.trajectory_waiter = LineTrajectory("/waiter_trajectory")
        self.trajectory_right_finish = LineTrajectory("/right_finish_trajectory")
        self.trajectory_waiter_backup = LineTrajectory("/waiter_backup_trajectory")
        self.trajectory_left_finish = LineTrajectory("/left_finish_trajectory")
        self.trajectory_stop = LineTrajectory("/stop_trajectory")

        # list of (x, y)
        self.delta_pts = [(1.93, 0.0), (2.06, 1.80)]
        self.backup_pts = [(2.01, 1.80), (1.93, 0.7)]
        self.waiter_pts = [(1.93, 0.7), (1.75, 1.15),(1.38, 1.42), (1.10, 2.16), (.70, 2.00), (.69, 1.45)]        
        self.waiter_backup_pts = [(.69, 1.45), (.95, 2.07)]
        self.right_finish_pts = [(0.45, 1.95),(0.4,0.96),(0.74,0.95),(1.1,0.76), (0.96, 0.3),(-0.1, .551)]

        self.left_finish_pts = [(.70, 2.00), (0.78, 1.04), (1.17, 0.50), (0.70, 0.43), (-0.1, .551)]

        self.stop_pts = []

        self.past_bobs = deque([],15)

        self.path = None


        #(0.9, 0.8),
        self.delta_speed = 1
        self.backup_speed = -1
        self.waiter_speed = 1
        self.waiter_backup_speed = -1
        self.right_finish_speed = 1
        self.left_finish_speed = 1

        self.stop_speed = 0.0

        '''
        Insert appropriate subscribers/publishers here

        '''
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=10)
        rospy.Subscriber("finished_mobile_traj", Bool, self.finished_cb)
        rospy.Subscriber("waiter_pose", Float32, self.bob_cb)
        self.traj_pub = rospy.Publisher("/trajectory/current", PolygonStamped, queue_size=10)
        self.trajectory_points = rospy.Publisher("/traj_pts", Marker, queue_size=20)
        self.mobile_arrived_pub = rospy.Publisher("/mobile_arrived", Bool, queue_size=10)
        self.publish_bob_pub = rospy.Publisher("/publish_bob", Bool, queue_size=10)

        self.listener = tf.TransformListener()



        self.create_trajectory(self.trajectory_delta, self.delta_pts, self.delta_speed)
        self.create_trajectory(self.trajectory_backup, self.backup_pts, self.backup_speed)
        self.create_trajectory(self.trajectory_waiter, self.waiter_pts, self.waiter_speed)
        self.create_trajectory(self.trajectory_waiter_backup, self.waiter_backup_pts, self.waiter_backup_speed)
        self.create_trajectory(self.trajectory_right_finish, self.right_finish_pts, self.right_finish_speed)
        self.create_trajectory(self.trajectory_left_finish, self.left_finish_pts, self.left_finish_speed)

        self.create_trajectory(self.trajectory_stop, self.stop_pts, self.stop_speed)


        self.current_traj_index = 0
        self.trajectories = [self.trajectory_delta, self.trajectory_backup, self.trajectory_waiter, \
                             self.trajectory_waiter_backup, self.trajectory_right_finish]

        rospy.sleep(0.2)
        self.planner()
                             
    def planner(self):
        #print "on step", self.current_traj_index
        if self.current_traj_index == 0:
            # publish delta trajectory
            self.traj_pub.publish(self.trajectory_delta.toPolygon())
            # self.mobile_arrived_pub.publish(True)
            self.current_traj_index += 1
        elif self.current_traj_index == 1:
            #  wait for delta_robot confirmation then publish backup
            rospy.wait_for_message("clicked_point", PointStamped)
            # rospy.wait_for_message("pizza_loaded", Bool)
            self.traj_pub.publish(self.trajectory_backup.toPolygon())
            self.current_traj_index += 1
        elif self.current_traj_index == 2:
            # immediately publish trajectory to waiter
            self.traj_pub.publish(self.trajectory_waiter.toPolygon())
            self.current_traj_index += 1

        elif self.current_traj_index == 3:
            # immediately publish trajectory to move backwards
            self.traj_pub.publish(self.trajectory_waiter_backup.toPolygon())
            self.current_traj_index += 1

        elif self.current_traj_index == 4:
            # wait for waiter information
            # either publish left of right
            self.publish_bob_pub.publish(True)
            while self.path is None:
                pass
            while self.current_traj_index == 4:
                if self.path == "right":
                    # print "trying to go",self.path
                    self.traj_pub.publish(self.trajectory_right_finish.toPolygon())
                elif self.path == "left":
                    # print "trying to go",self.path
                    self.traj_pub.publish(self.trajectory_left_finish.toPolygon())
                else:
                    pass
                    #print "self.path is not right or left", self.path

                try:
                    trans, rot = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    print trans[1]
                    if trans[1] < 1.70:
                        self.current_traj_index += 1
                except tf.LookupException:
                    print("error in transformation")
                rospy.sleep(0.03)

    def create_trajectory(self, traj, pts, speed):
        for x, y in pts:
            p = Point()
            p.x = x
            p.y = y
            traj.addPoint(p)
            traj.speed_profile.append(speed)

    def clicked_pose(self, point):
        print "yay"

    def finished_cb(self, msg):
        if msg.data == True:
            self.traj_pub.publish(self.trajectory_stop.toPolygon())
            self.planner()

    def bob_cb(self, msg):
        self.past_bobs.append((msg.data, rospy.get_rostime().to_sec()))
        if len(self.past_bobs) < 15:# or self.path is not None:
            return
        x = 0.5*(self.past_bobs[0][0] + self.past_bobs[-1][0])
        v = (self.past_bobs[-1][0] - self.past_bobs[0][0])/(self.past_bobs[-1][1] - self.past_bobs[0][1])
        last_path = self.path
        if abs(v) < .02:
            if x > 0.45:
                self.path = "right"
            else:
                self.path = "left"
        else:
            if v > 0:
                self.path = "right"
            else:
                self.path = "left"
        if last_path!=self.path:
            print "changed path",last_path,"to",self.path
        #print "path chosen", self.path, x, v


if __name__=="__main__":
    rospy.init_node("high_level_task_planner")
    hltp = HighLevelTaskPlanner()
    rospy.spin()
