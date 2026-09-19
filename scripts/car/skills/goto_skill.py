#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist

from skills.base_skill import BaseSkill, RUNNING, SUCCESS, FAILED
from skills.steering import AvoidanceSteering


class GoToSkill(BaseSkill):
    """导航技能：默认交给 move_base 规划路径（会绕开墙体和障碍）。

    私有参数（写在 config/car/*.yaml 里）：
        ~use_move_base (bool, 默认 true)
            true  → 向 /<ns>/move_base_simple/goal 发目标，等 /<ns>/move_base/result；
                    能绕障，依赖 map_server + amcl + scan + tf
            false → 退化成“原地转向 + 直行”的纯 cmd_vel 驾驶。没有任何避障能力，
                    只在 move_base 没起来时做调试用。
        ~nav_wait_s (float, 默认 5.0)
            move_base 在这段时间里既没返回结果、车也没挪动，就判定失败并打印排查提示。
        ~nav_speed / ~nav_angular_speed
            仅 use_move_base=false 的直行模式生效。
    """

    # move_base(actionlib) 的 GoalStatus 状态码
    _STATUS_SUCCEEDED = 3
    _STATUS_FAILED = (4, 5, 8, 9)  # ABORTED / REJECTED / RECALLED / LOST

    def __init__(self, skill_manager, frame_id="map"):
        super(GoToSkill, self).__init__(skill_manager)
        self.frame_id = str(frame_id)
        self._use_move_base = bool(rospy.get_param("~use_move_base", True))
        self._nav_wait_s = float(rospy.get_param("~nav_wait_s", 5.0))

        self._target_x = 0.0
        self._target_y = 0.0
        self._start_time = None
        self._timeout = 20.0
        self._start_pose = None

        # 仅纯 cmd_vel 直行模式使用
        self._arrival_tolerance = 0.3
        self._speed = float(rospy.get_param("~nav_speed", 0.4))
        self._angular_speed = float(rospy.get_param("~nav_angular_speed", 0.8))
        self._angle_tolerance = 0.15
        self._obstacle_stop_distance = float(rospy.get_param("~nav_obstacle_stop", 0.5))
        self._has_printed = False
        self._steering = None
        # 本次任务实际使用的模式（配置 + 是否处于 move_base 降级期）
        self._active_use_move_base = True

    def start(self, task):
        task = task or {}
        if isinstance(task, dict):
            self._target_x = float(task.get('target_x', 0.0))
            self._target_y = float(task.get('target_y', 0.0))
            self._timeout = float(task.get('timeout', 20.0)) or 20.0
        else:
            self._target_x = float(task.target_x)
            self._target_y = float(task.target_y)
            self._timeout = float(getattr(task, 'timeout', 20.0)) or 20.0

        self._start_time = rospy.Time.now().to_sec()
        self._has_printed = False
        self._start_pose = self.skill_manager.get_current_pose()
        self._target_x, self._target_y = self._adjust_target(self._target_x, self._target_y)

        use_move_base = self._use_move_base and not self.skill_manager.nav_fallback_active()
        self._active_use_move_base = use_move_base
        if use_move_base:
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.frame_id
            goal.pose.position.x = self._target_x
            goal.pose.position.y = self._target_y
            goal.pose.position.z = 0.0
            goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.skill_manager.reset_nav_status()
            self.skill_manager.publish_nav_goal(goal)
            rospy.loginfo(
                "[%s] GoToSkill(move_base) start: target=(%.2f, %.2f) timeout=%.1fs",
                self.skill_manager.ns, self._target_x, self._target_y, self._timeout,
            )
        else:
            self.skill_manager.publish_nav_cancel()
            # 直行模式统一用带绕障的控制律（详见 skills/steering.py）
            self._steering = AvoidanceSteering(
                self.skill_manager,
                speed=self._speed,
                max_angular=self._angular_speed,
                align_gain=float(rospy.get_param("~nav_align_gain", 1.6)),
                lookahead=float(rospy.get_param("~nav_lookahead", 1.1)),
                front_stop=self._obstacle_stop_distance,
                target_sector_deg=25.0,
            )
            self._steering.reset()
            rospy.loginfo(
                "[%s] GoToSkill(cmd_vel 直行) start: target=(%.2f, %.2f)",
                self.skill_manager.ns, self._target_x, self._target_y,
            )

    def _adjust_target(self, target_x, target_y):
        """把目标点夹到地图范围内，避免把目标点发到地图外。"""
        map_info = self.skill_manager.get_map_info()
        if not map_info:
            return target_x, target_y

        margin = 0.35
        min_x = map_info['origin_x'] + margin
        max_x = map_info['origin_x'] + map_info['width'] * map_info['resolution'] - margin
        min_y = map_info['origin_y'] + margin
        max_y = map_info['origin_y'] + map_info['height'] * map_info['resolution'] - margin

        clamped_x = max(min_x, min(float(target_x), max_x))
        clamped_y = max(min_y, min(float(target_y), max_y))
        if abs(clamped_x - target_x) > 1e-6 or abs(clamped_y - target_y) > 1e-6:
            rospy.logwarn(
                "[%s] GoToSkill: 目标点 (%.2f, %.2f) 超出地图范围，夹到 (%.2f, %.2f)",
                self.skill_manager.ns, target_x, target_y, clamped_x, clamped_y,
            )

        # 落在障碍格上的目标点 move_base 一定会失败，就近挪到可通行的地方
        return self.skill_manager.nearest_free_point(clamped_x, clamped_y)

    def update(self):
        elapsed = rospy.Time.now().to_sec() - self._start_time
        if self._active_use_move_base:
            return self._update_move_base(elapsed)
        return self._update_direct(elapsed)

    # ------------------------------------------------------------------
    # move_base 模式
    # ------------------------------------------------------------------
    def _update_move_base(self, elapsed):
        status = self.skill_manager.nav_status_code

        if status == self._STATUS_SUCCEEDED:
            rospy.loginfo("[%s] GoToSkill: move_base 到达目标", self.skill_manager.ns)
            self.skill_manager.publish_stop_velocity()
            return SUCCESS

        if status in self._STATUS_FAILED:
            rospy.logwarn(
                "[%s] GoToSkill: move_base 规划/执行失败 (status=%s)，放弃该目标点",
                self.skill_manager.ns, status,
            )
            self.skill_manager.publish_stop_velocity()
            self.skill_manager.activate_nav_fallback()
            return FAILED

        if elapsed > self._timeout:
            rospy.logwarn(
                "[%s] GoToSkill: move_base 超时 (%.1fs > %.1fs)",
                self.skill_manager.ns, elapsed, self._timeout,
            )
            self.skill_manager.publish_stop_velocity()
            return FAILED

        # status 仍为 -1：既没有结果，也没有进展，多半是定位/话题链断了
        if elapsed > self._nav_wait_s and not self._has_moved():
            rospy.logerr(
                "[%s] GoToSkill: %.1fs 内 move_base 没有返回结果、车也没有移动。"
                "请依次检查：map_server 是否发出 /map、amcl 是否发出 /%s/amcl_pose、"
                "/%s/scan 是否有数据、tf 是否连通 "
                "(map -> %s/odom -> %s/base_footprint)。"
                "本车接下来会自动改用自带激光绕障直行。",
                self.skill_manager.ns, elapsed, self.skill_manager.ns,
                self.skill_manager.ns, self.skill_manager.ns, self.skill_manager.ns,
            )
            self.skill_manager.activate_nav_fallback()
            return FAILED

        return RUNNING

    def _has_moved(self):
        pose = self.skill_manager.get_current_pose()
        if pose is None or self._start_pose is None:
            return False
        dx = pose.position.x - self._start_pose.position.x
        dy = pose.position.y - self._start_pose.position.y
        return math.hypot(dx, dy) > 0.15

    # ------------------------------------------------------------------
    # 纯 cmd_vel 直行模式（无避障，仅调试用）
    # ------------------------------------------------------------------
    def _update_direct(self, elapsed):
        if elapsed > self._timeout:
            rospy.logwarn("[GoToSkill] 直行超时 (%.1fs > %.1fs)", elapsed, self._timeout)
            self.skill_manager.publish_stop_velocity()
            return FAILED

        pose = self.skill_manager.get_current_pose()
        if pose is None:
            if not self._has_printed:
                rospy.logwarn(
                    "[%s] GoToSkill: 还没有位姿（/%s/odom 与 /%s/amcl_pose 都没数据）",
                    self.skill_manager.ns, self.skill_manager.ns, self.skill_manager.ns,
                )
                self._has_printed = True
            return RUNNING

        dx = self._target_x - pose.position.x
        dy = self._target_y - pose.position.y
        distance = math.hypot(dx, dy)

        if distance < self._arrival_tolerance:
            rospy.loginfo("[GoToSkill] arrived at target (dist=%.2f)", distance)
            self.skill_manager.publish_stop_velocity()
            return SUCCESS

        desired_yaw = math.atan2(dy, dx)
        current_yaw = self.skill_manager.get_current_yaw()
        if current_yaw is None:
            return RUNNING

        if self._steering is None:
            self._steering = AvoidanceSteering(
                self.skill_manager,
                speed=self._speed,
                max_angular=self._angular_speed,
                align_gain=float(rospy.get_param("~nav_align_gain", 1.6)),
                lookahead=float(rospy.get_param("~nav_lookahead", 1.1)),
                front_stop=self._obstacle_stop_distance,
                target_sector_deg=25.0,
            )

        linear, angular, mode = self._steering.compute(
            current_yaw, desired_yaw, distance, rospy.Time.now().to_sec())
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.skill_manager.publish_cmd_vel(cmd)
        rospy.loginfo_throttle(
            2.0, "[%s] GoToSkill(直行)[%s] dist=%.2f lin=%.2f ang=%.2f",
            self.skill_manager.ns, mode, distance, linear, angular,
        )
        return RUNNING

    def stop(self):
        rospy.loginfo("[GoToSkill] stop")
        self.skill_manager.publish_stop_velocity()
        self.skill_manager.publish_nav_cancel()
