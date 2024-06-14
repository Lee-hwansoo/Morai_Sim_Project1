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

# Node Execution Order
# 1. Declare subscribers and publishers.
# 2. Set Look Ahead Distance value proportional to speed.
# 3. Create coordinate transformation matrix.
# 4. Calculate steering angle.
# 5. Generate PID controller.
# 6. Compute curvature of the road.
# 7. Plan speed based on curvature.
# 8. Publish control input message.

class stanley :
    def __init__(self):
        rospy.init_node('stanley', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)

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
        self.current_position = Point()

        self.vehicle_length = 4.47
        if self.vehicle_length is None:
            print("you need to change values at line 51 , self.vehicle_length ")
            exit()
        self.target_velocity = 40

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True:
                prev_time = time.time()

                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6

                steering = self.calc_stanley()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else :
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (8) 제어입력 메세지 Publish
                print(steering)
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicle Status Subscriber
        self.is_status=True
        self.status_msg=msg

    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True

    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')
        current_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_stanley(self):
        # 오차 계산
        cte = self.calc_cross_track_error()

        # 차량 방향과 경로 방향의 차이 각도 계산
        yaw_error = atan2(self.waypoints[self.current_waypoint][1] - self.current_position.y,
                          self.waypoints[self.current_waypoint][0] - self.current_position.x) - self.current_yaw

        # Stanley 제어 계산
        steering = yaw_error + atan2(self.vehicle_length * cte, self.status_msg.velocity.x)
        return steering

    def calc_cross_track_error(self):
        # 현재 차량 위치를 기준으로 가장 가까운 경로 상의 점 찾기
        min_dist = float('inf')
        for i, (x, y) in enumerate(self.waypoints):
            dist = sqrt((self.current_position.x - x) ** 2 + (self.current_position.y - y) ** 2)
            if dist < min_dist:
                min_dist = dist
                self.current_waypoint = i

        # 차량 위치와 가장 가까운 경로 상의 점과의 거리 계산
        x0, y0 = self.waypoints[self.current_waypoint]
        x1, y1 = self.waypoints[(self.current_waypoint + 1) % len(self.waypoints)]
        cte = ((y1 - y0) * self.current_position.x - (x1 - x0) * self.current_position.y + x1 * y0 - y1 * x0) / \
              sqrt((y1 - y0) ** 2 + (x1 - x0) ** 2)
        return cte

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.025
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
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

            #TODO: (6) 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            #TODO: (7) 곡률 기반 속도 계획
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
        test_track=stanley()
    except rospy.ROSInterruptException:
        pass
