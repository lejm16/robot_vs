#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from robot_vs.msg import RobotCommand
from tf.transformations import euler_from_quaternion


class AMCLConvergencePolicy(object):
    """通过里程计累计偏航角，控制机器人原地转满一圈。"""

    def __init__(self, robot_ns_list, angular_speed=0.6, target_turn=2.0 * math.pi):
        self.angular_speed = max(0.1, abs(angular_speed))
        self.target_turn = target_turn
        self.total_yaw = {ns: 0.0 for ns in robot_ns_list}
        self.last_yaw = {ns: None for ns in robot_ns_list}
        self.is_done = {ns: False for ns in robot_ns_list}

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_odom(odom):
        q = odom.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        return yaw

    def step(self, ns, odom):
        """返回当前应发布的cmd_vel，以及该机器人是否完成收敛动作。"""
        if self.is_done[ns]:
            return Twist(), True

        cmd = Twist()
        cmd.angular.z = self.angular_speed

        if odom is None:
            return cmd, False

        yaw = self._yaw_from_odom(odom)
        if self.last_yaw[ns] is None:
            self.last_yaw[ns] = yaw
            return cmd, False

        dyaw = self._normalize_angle(yaw - self.last_yaw[ns])
        self.total_yaw[ns] += abs(dyaw)
        self.last_yaw[ns] = yaw

        if self.total_yaw[ns] >= self.target_turn:
            self.is_done[ns] = True
            rospy.loginfo("[%s] AMCL收敛旋转完成，累计角度 %.2f rad", ns, self.total_yaw[ns])
            return Twist(), True

        return cmd, False

class PatrolPolicy(object):
    """简单巡逻策略：距离到点 -> 停留 -> 切下一个目标。"""

    def __init__(self, robot_ns_list):
        self.patrol_points = [(-0.889, 1.6), (-2.59, 0.767), (-0.156, 0.2)]
        self.arrive_radius = 0.15
        self.arrive_hold_s = 3.0

        point_count = len(self.patrol_points)
        # 每台机器人当前目标点索引
        self.current_idx = {}
        # 到点后的停留开始时间(None表示尚未开始停留)
        self.hold_start_time = {}
        # 已下发过的目标点索引，用于避免每个循环都重复发goal
        self.last_goal_idx = {}
        for i, ns in enumerate(robot_ns_list):
            self.current_idx[ns] = i % point_count
            self.hold_start_time[ns] = None
            self.last_goal_idx[ns] = None

    def _build_nav_goal(self, target_x, target_y):
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = "map"
        nav_goal.header.stamp = rospy.Time.now()
        nav_goal.pose.position.x = target_x
        nav_goal.pose.position.y = target_y
        nav_goal.pose.orientation.w = 1.0
        return nav_goal

    def _build_robot_command(self, target_x, target_y):
        robot_cmd = RobotCommand()
        robot_cmd.mode = 1
        robot_cmd.attack = False
        robot_cmd.goal_x = target_x
        robot_cmd.goal_y = target_y
        return robot_cmd

    def decide(self, ns, pose, is_arrived=False):
        """返回当前机器人(nav_goal, robot_cmd)。nav_goal可能为None(无须重发)。"""
        if pose is None:
            return None, None

        idx = self.current_idx[ns]
        target_x, target_y = self.patrol_points[idx]
        now = rospy.Time.now()
        dist = math.hypot(pose.position.x - target_x, pose.position.y - target_y)

        # 到点判定采用双条件：几何距离或move_base成功状态。
        reached = (dist <= self.arrive_radius) or bool(is_arrived)

        # 进入到点状态后开始停留计时，达到停留时间再切到下一点
        if reached:
            if self.hold_start_time[ns] is None:
                self.hold_start_time[ns] = now
                rospy.loginfo("[%s] reached waypoint %d, hold %.1fs (dist=%.3f, status=%s)",
                              ns, idx, self.arrive_hold_s, dist, str(bool(is_arrived)))
            elif (now - self.hold_start_time[ns]).to_sec() >= self.arrive_hold_s:
                idx = (idx + 1) % len(self.patrol_points)
                self.current_idx[ns] = idx
                self.hold_start_time[ns] = None
                self.last_goal_idx[ns] = None
                target_x, target_y = self.patrol_points[idx]
                rospy.loginfo("[%s] switch to waypoint %d (%.2f, %.2f)", ns, idx, target_x, target_y)
        else:
            self.hold_start_time[ns] = None

        nav_goal = None
        if self.last_goal_idx[ns] != self.current_idx[ns]:
            nav_goal = self._build_nav_goal(target_x, target_y)
            self.last_goal_idx[ns] = self.current_idx[ns]

        robot_cmd = self._build_robot_command(target_x, target_y)

        return nav_goal, robot_cmd


class DecisionEngine:
    def __init__(self, robot_ns_list, perception_dict, executor_dict):
        rospy.loginfo("决策模块初始化完成")
        self.robot_ns_list = list(robot_ns_list)
        self.perception_dict = perception_dict
        self.executor_dict = executor_dict
        self.patrol_policy = PatrolPolicy(self.robot_ns_list)
        self.amcl_convergence_policy = AMCLConvergencePolicy(self.robot_ns_list)

    def make_amcl_convergence_actions(self):
        """AMCL收敛阶段决策：输出每台机器人的速度指令。"""
        vel_actions = {}
        all_done = True

        for ns in self.robot_ns_list:
            perception = self.perception_dict.get(ns)
            odom = perception.current_odom if perception else None
            cmd_vel, done = self.amcl_convergence_policy.step(ns, odom)
            vel_actions[ns] = cmd_vel
            all_done = all_done and done

        return vel_actions, all_done

    def make_team_decision(self):
        """团队决策：直接读取共享状态，输出每台机器人(nav_goal, robot_cmd)。"""
        actions = {}
        for ns in self.robot_ns_list:
            perception = self.perception_dict.get(ns)
            executor = self.executor_dict.get(ns)

            pose = perception.current_pose if perception else None
            image = perception.current_image if perception else None
            # odom = perception.current_odom if perception else None
            is_arrived = executor.is_arrived if executor else False

            nav_goal, robot_cmd = self._make_single_robot_decision(ns, pose, image, is_arrived)
            actions[ns] = (nav_goal, robot_cmd)

        return actions

    def _make_single_robot_decision(self, ns, pose, image, is_arrived):
        """单车决策子过程：当前先保留巡逻逻辑，后续可替换为协同策略。"""
        _ = image
        return self.patrol_policy.decide(ns, pose, is_arrived=is_arrived)