#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from command_parser import CommandParser

class RedRobot:
    def __init__(self):
        rospy.init_node("red_robot")
        self.ns = "robot_red"  # 红方命名空间
        
        self.parser = CommandParser(self.ns)

        self.rate = rospy.Rate(10)  # 频率
        rospy.loginfo("红方robot启动完成")

    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()



if __name__ == '__main__':
    try:
        robot = RedRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass