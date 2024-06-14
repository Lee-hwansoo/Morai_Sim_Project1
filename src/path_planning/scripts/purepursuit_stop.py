#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        # rospy.Subscriber("/local_path", Path, self.path_callback)
        
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd_0',CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 4.470
        self.lfd = 4.5
        self.min_lfd = 3
        self.max_lfd = 25
        self.lfd_gain = 0.81
        self.target_velocity = 60

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        while not self.is_global_path:
            print('Waiting global path data')
            time.sleep(0.5)
        self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6
                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else:
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    
                output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)
                # 도착 지점에 도달했는지 확인
                if self.is_arrived(self.status_msg, self.global_path, 12.0):
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 100.0
                else:
                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0.0
                        print("here1!")
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -output
                        print("here2!")

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            else:
                print(f"self.is_path (/lattice_path) : {self.is_path}")
                print(f"self.is_status (/Ego_topic)  : {self.is_status}")
                print(f"self.is_odom (/odom)         : {self.is_odom}")

            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg    

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

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

    def calc_pure_pursuit(self):
        self.lfd = max(self.min_lfd, min(self.max_lfd, self.status_msg.velocity.x * self.lfd_gain))

        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * sin(theta) * self.vehicle_length), self.lfd)

        return steering
    
    def is_arrived(self, ego_status, global_path, dist):
        # 현재 위치와 마지막 웨이포인트의 거리를 계산하여 도착 여부 확인
        x = global_path.poses[-1].pose.position.x
        y = global_path.poses[-1].pose.position.y
        distance_to_last_waypoint = sqrt((ego_status.position.x - x) ** 2 +
                                         (ego_status.position.y - y) ** 2)
        return distance_to_last_waypoint <= dist

class pidControl:
    def __init__(self):
        self.p_gain = 0.8
        self.i_gain = 0.01
        self.d_gain = 0.08
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__(self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(len(global_path.poses)):
            if i < point_num or i > len(global_path.poses) - point_num - 1:
                out_vel_plan.append(self.car_max_speed)
                continue
            
            # 곡률 기반 속도 계산
            x_list, y_list = [], []
            for j in range(i - point_num, i + point_num + 1):
                if 0 <= j < len(global_path.poses):
                    x = global_path.poses[j].pose.position.x
                    y = global_path.poses[j].pose.position.y
                    x_list.append([-2*x, -2*y, 1])
                    y_list.append(-x*x - y*y)

            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_transpose = x_matrix.T

            try:
                a_matrix = np.linalg.inv(x_transpose.dot(x_matrix)).dot(x_transpose).dot(y_matrix)
                a, b, c = a_matrix[0], a_matrix[1], a_matrix[2]
                R = np.sqrt(a*a + b*b - c)
                v_max = np.sqrt(R * 9.81 * self.road_friction)
                out_vel_plan.append(min(v_max, self.car_max_speed))
            except np.linalg.LinAlgError:
                out_vel_plan.append(self.car_max_speed)

        # 마지막 접근을 위해 속도 조정
        final_approach_index = max(0, len(global_path.poses) - 20)
        for i in range(final_approach_index, len(global_path.poses)):
            out_vel_plan[i] = max(0, out_vel_plan[i] - (self.car_max_speed / 10) * (i - final_approach_index + 1))

        return out_vel_plan


if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass

