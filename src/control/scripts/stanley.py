#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2, degrees
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class stanley_controller:
    def __init__(self):
        rospy.init_node('stanley_controller', anonymous=True)

        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd_0', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.K = 0.1  # Gain for Stanley control
        self.min_lfd = 5  # Minimum look ahead distance
        self.max_lfd = 30  # Maximum look ahead distance
        self.vehicle_length = 4.47  # Length of the vehicle
        if self.vehicle_length is None:
            print("you need to change values at line 38 , self.vehicle_length ")
            exit()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                print('Waiting global path data')

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6

                steering = self.stanley_control(self.status_msg, self.global_path)
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                self.ctrl_cmd_msg.steering = steering
                if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def odom_callback(self, msg):
        self.is_odom = True
        self.current_position = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')
        current_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def stanley_control(self, ego_status, global_path):
        # Find the nearest point on the path
        nearest_point = global_path.poses[self.current_waypoint].pose.position
        dx = nearest_point.x - ego_status.position.x
        dy = nearest_point.y - ego_status.position.y
        cte = dy * cos(self.vehicle_yaw) - dx * sin(self.vehicle_yaw)

        # Find the heading error
        yaw_path = atan2(dy, dx)
        yaw_diff = yaw_path - self.vehicle_yaw

        # Ensure the error is between -pi and pi
        while(yaw_diff >= -pi and yaw_diff <= pi):
            if yaw_diff > pi:
                yaw_diff -= 2 * pi
            elif yaw_diff < -pi:
                yaw_diff += 2 * pi

        # Calculate the steering angle using Stanley control
        steering = yaw_diff + atan2(self.K * cte, ego_status.velocity.x)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = global_path.poses[i+box].pose.position.x
                y = global_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            #곡률 기반 속도 계획
            v_max = sqrt(r*9.8*self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        stanley_controller_instance = stanley_controller()
    except rospy.ROSInterruptException:
        pass

