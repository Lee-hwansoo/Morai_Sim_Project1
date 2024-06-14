#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
from math import pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from morai_msgs.msg import EgoVehicleStatus
from tf.transformations import quaternion_from_euler

class odometry:
    def __init__(self):
        rospy.init_node('odometry')
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        rospy.loginfo("Odometry Initialized")

        self.init = False
        while not rospy.is_shutdown():
            rospy.spin()

    def ego_callback(self, _data):
        if not self.init:
            self.init = True
            self.initial_pose = _data

        diff_x = -(_data.position.y - self.initial_pose.position.y)
        diff_y = _data.position.x - self.initial_pose.position.x
        diff_heading = _data.heading - self.initial_pose.heading
        br = tf.TransformBroadcaster()
        br.sendTransform((diff_x, diff_y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, (diff_heading)/180*pi),
                        rospy.Time.now(),
                        "base_link",
                        "odom")
        orientation = tf.transformations.quaternion_from_euler(0, 0, (diff_heading)/180*pi)
        odometry_data = Odometry()
        odometry_data.header.stamp = rospy.Time.now()
        odometry_data.header.frame_id = "map"
        odometry_data.child_frame_id = "base_link"
        odometry_data.pose.pose.position.x = diff_x
        odometry_data.pose.pose.position.y = diff_y
        odometry_data.pose.pose.orientation.x = orientation[0]
        odometry_data.pose.pose.orientation.y = orientation[1]
        odometry_data.pose.pose.orientation.z = orientation[2]
        odometry_data.pose.pose.orientation.w = orientation[3]
        self.odom_pub.publish(odometry_data)

if __name__ == '__main__':
    try:
        new_class = odometry()
    except rospy.ROSInterruptException:
        pass

