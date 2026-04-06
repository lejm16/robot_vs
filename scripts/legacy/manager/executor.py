#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from robot_vs.msg import RobotCommand

class Executor:
    def __init__(self, namespace):
        self.ns = namespace
        self.is_arrived = False
        
        # 发布导航目标点
        self.goal_pub = rospy.Publisher(
            "/{}/move_base_simple/goal".format(self.ns),
            PoseStamped,
            queue_size=10
        )
        
        # 发布自定义行为指令
        self.command_pub = rospy.Publisher(
            "/{}/robot_command".format(self.ns),
            RobotCommand,
            queue_size=10
        )

        # 发布底盘速度，供AMCL初始化时触发微小运动
        self.cmd_vel_pub = rospy.Publisher(
            "/{}/cmd_vel".format(self.ns),
            Twist,
            queue_size=10
        )

        # 订阅导航结果，用于记录是否到达目标
        self.move_base_result_sub = rospy.Subscriber(
            "/{}/move_base/result".format(self.ns),
            MoveBaseActionResult,
            self.move_base_result_callback
        )
        
        rospy.loginfo("[{}] 执行模块初始化完成".format(self.ns))

    def publish_nav_goal(self, goal):
        """发布导航目标点到move_base"""

        self.is_arrived = False
        self.goal_pub.publish(goal)
        # rospy.loginfo("[{}] 发布导航目标: ({}, {})".format(self.ns,goal.pose.position.x,goal.pose.position.y))

    def publish_robot_command(self, cmd):
        """发布自定义行为指令"""

        
        self.command_pub.publish(cmd)
        # rospy.loginfo("[{}] 发布行为指令: 攻击={}, 模式={}".format(self.ns,cmd.attack,cmd.mode))

    def move_base_result_callback(self, msg):
        # GoalStatus.SUCCEEDED = 3
        self.is_arrived = (msg.status.status == 3)

    def publish_cmd_vel(self, cmd_vel):
        """发布底盘速度指令。"""
        self.cmd_vel_pub.publish(cmd_vel)