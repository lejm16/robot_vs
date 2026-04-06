#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from perception import Perception
from decision import DecisionEngine
from executor import Executor

class BlueTeamManager:
    def __init__(self):
        # 初始化节点，指定命名空间
        rospy.init_node("blue_team_manager")
        # 蓝方两台机器人的命名空间列表
        self.robot_ns_list = ["robot_blue", "robot_blue2"]  
        
        # 为每台机器人初始化 感知/决策/执行 模块
        self.perception_dict = {}
        self.decision_dict = {}
        self.executor_dict = {}
        for ns in self.robot_ns_list:
            self.perception_dict[ns] = Perception(ns)
            self.decision_dict[ns] = DecisionEngine(ns)
            self.executor_dict[ns] = Executor(ns)

        self.rate = rospy.Rate(10)  # 决策频率
        rospy.loginfo("蓝方TeamManager启动完成，管理机器人：%s", self.robot_ns_list)

    def run(self):
        """主循环：为每台机器人独立执行 感知→决策→执行"""
        while not rospy.is_shutdown():
            for ns in self.robot_ns_list:
                # 1. 感知：获取当前机器人的位置/图像数据
                pose = self.perception_dict[ns].get_current_pose()
                image = self.perception_dict[ns].get_current_image()

                # 2. 决策：根据感知数据生成指令（可扩展为多机协同决策）
                nav_goal, robot_cmd = self.decision_dict[ns].make_decision(pose, image)

                # 3. 执行：发布指令给当前机器人
                if nav_goal:
                    self.executor_dict[ns].publish_nav_goal(nav_goal)
                if robot_cmd:
                    self.executor_dict[ns].publish_robot_command(robot_cmd)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        manager = BlueTeamManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass