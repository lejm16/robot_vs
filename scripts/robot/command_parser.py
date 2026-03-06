#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from robot_vs.msg import RobotCommand

class CommandParser:
    def __init__(self, namespace):
        self.ns = namespace  # 命名空间：robot_red/robot_blue
        # 订阅Manager的指令
        self.cmd_sub = rospy.Subscriber(
            "/{}/robot_command".format(self.ns),
            RobotCommand,
            self.cmd_callback
        )
        rospy.loginfo("[{}] 指令解析节点启动".format(self.ns))

    def cmd_callback(self, msg):
        """解析指令:导航目标已由move_base处理,这里只处理攻击/行为模式"""
        # 1. 处理攻击指令
        if msg.attack:
            rospy.loginfo("I am car, [{}] 收到攻击指令！".format(self.ns))
            # 调用attack_executor执行攻击（预留接口）
            # self.execute_attack()
        # 2. 处理行为模式
        mode_map = {0: "待机",1: "巡逻", 2: "攻击", 3: "撤退"}
        rospy.loginfo("I am car, [{}] 当前模式：{}".format(self.ns,mode_map[msg.mode]))

    def execute_attack(self):
        """攻击逻辑（预留，可扩展：控制机械臂/发射装置等）"""
        pass
