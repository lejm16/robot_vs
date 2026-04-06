#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def blue_controller():
    rospy.init_node('blue_controller_node')

    # 蓝车的控制话题
    pub = rospy.Publisher('/robot_blue/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    vel_msg = Twist()
    # 让蓝车直线走
    vel_msg.linear.x = 0.3
    vel_msg.angular.z = 0.0

    rospy.loginfo("蓝车开始移动...")
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        blue_controller()
    except rospy.ROSInterruptException:
        pass