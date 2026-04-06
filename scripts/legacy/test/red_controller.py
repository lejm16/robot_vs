#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def red_controller():
    rospy.init_node('red_controller_node')

    # 红车的控制话题：带 namespace
    pub = rospy.Publisher('/robot_red/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    vel_msg = Twist()
    # 让红车原地转圈
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = 0.3

    rospy.loginfo("红车开始移动...")
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        red_controller()
    except rospy.ROSInterruptException:
        pass