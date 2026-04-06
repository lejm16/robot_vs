#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from perception import Perception
from decision import DecisionEngine
from executor import Executor

class RedTeamManager:
    def __init__(self):
        # 初始化节点，指定命名空间
        rospy.init_node("red_team_manager")
        # 红方两台机器人的命名空间列表
        self.robot_ns_list = ["robot_blue","robot_blue2"]  #tempp!!!!!
        
        # 为每台机器人初始化 感知/执行 模块
        self.perception_dict = {}
        self.executor_dict = {}
        for ns in self.robot_ns_list:
            self.perception_dict[ns] = Perception(ns)
            self.executor_dict[ns] = Executor(ns)

        # 团队级决策器：直接读取perception/executor共享状态
        self.decision_engine = DecisionEngine(
            self.robot_ns_list,
            self.perception_dict,
            self.executor_dict
        )

        self.rate = rospy.Rate(10)  # 决策频率
        self.amcl_ready = False

        rospy.loginfo("红方TeamManager启动完成")

    def run(self):
        """主循环：团队决策 -> 分车执行"""
        while not rospy.is_shutdown():
            if not self.amcl_ready:
                vel_actions, all_done = self.decision_engine.make_amcl_convergence_actions()
                for ns in self.robot_ns_list:
                    cmd_vel = vel_actions.get(ns)
                    if cmd_vel:
                        self.executor_dict[ns].publish_cmd_vel(cmd_vel)

                if all_done:
                    self.amcl_ready = True
                    rospy.loginfo("AMCL收敛阶段完成，切换到常规决策")

                self.rate.sleep()
                continue

            # 1. 决策：直接读取共享状态做协同决策
            team_actions = self.decision_engine.make_team_decision()

            # 2. 执行：按机器人分发指令
            for ns in self.robot_ns_list:
                nav_goal, robot_cmd = team_actions.get(ns, (None, None))
                if nav_goal:
                    self.executor_dict[ns].publish_nav_goal(nav_goal)
                if robot_cmd:
                    self.executor_dict[ns].publish_robot_command(robot_cmd)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        manager = RedTeamManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass