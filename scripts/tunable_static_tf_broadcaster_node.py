#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from dynamic_reconfigure.server import Server
from tunable_static_tf_broadcaster.cfg import TfConfig
from time import sleep

class TunableStaticTFBroadcaster:
    def __init__(self):
        self.__pub_rate = rospy.get_param('~rate', 10) #Hz
        #self.__rate = rospy.Rate(self.__pub_rate)
        self.__tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.__static_transformStamped = geometry_msgs.msg.TransformStamped()
        self.__static_transformStamped.header.frame_id = rospy.get_param('~header_frame', "world")
        self.__static_transformStamped.child_frame_id = rospy.get_param('~child_frame', "base_link")
        self.__static_transformStamped.transform.translation.x = 0.1
        self.__static_transformStamped.transform.translation.y = 0.1
        self.__static_transformStamped.transform.translation.z = 0.1
        self.__tf_roll = 0.0
        self.__tf_pitch = 0.0
        self.__tf_yaw = 0.0
        self.__tf_quaternion = tf.transformations.quaternion_from_euler(self.__tf_roll, self.__tf_pitch, self.__tf_yaw)
        self.__static_transformStamped.transform.rotation.x = self.__tf_quaternion[0]
        self.__static_transformStamped.transform.rotation.y = self.__tf_quaternion[1]
        self.__static_transformStamped.transform.rotation.z = self.__tf_quaternion[2]
        self.__static_transformStamped.transform.rotation.w = self.__tf_quaternion[3]
        self.__srv = Server(TfConfig, self.__reconfigure_callback)

    def __reconfigure_callback(self, config, level):
        self.__static_transformStamped.transform.translation.x = config.tf_x
        self.__static_transformStamped.transform.translation.y = config.tf_y
        self.__static_transformStamped.transform.translation.z = config.tf_z
        self.__tf_roll = config.tf_roll
        self.__tf_pitch = config.tf_pitch
        self.__tf_yaw = config.tf_yaw
        self.__tf_quaternion = tf.transformations.quaternion_from_euler(self.__tf_roll, self.__tf_pitch, self.__tf_yaw)
        self.__static_transformStamped.transform.rotation.x = self.__tf_quaternion[0]
        self.__static_transformStamped.transform.rotation.y = self.__tf_quaternion[1]
        self.__static_transformStamped.transform.rotation.z = self.__tf_quaternion[2]
        self.__static_transformStamped.transform.rotation.w = self.__tf_quaternion[3]
        return config

    def run(self):
        while not rospy.is_shutdown():
            self.__static_transformStamped.header.stamp = rospy.Time.now()
            self.__tf_broadcaster.sendTransform(self.__static_transformStamped)
            #self.__rate.sleep()
            sleep(1/self.__pub_rate)


def main():
    rospy.init_node('tunable_static_tf_broadcaster')
    broadcaster = TunableStaticTFBroadcaster()
    broadcaster.run()

if __name__ == '__main__':
    main()
