#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from skills.base_skill import BaseSkill, RUNNING, FAILED
from skills.steering import AvoidanceSteering


class AttackSkill(BaseSkill):
    """追击目标，对准后开火。

    控制方式（纯 cmd_vel，不依赖 move_base）：
      1) **比例转向**：angular.z = clamp(align_gain * err)。
         旧版是“误差 > 容差就用 ±0.7 rad/s 硬打”，而容差只有 0.02 rad，
         每帧却要转过 0.07 rad（10 Hz 主循环），车会在目标朝向两侧无限来回摆，
         既进不了前进分支、也满足不了开火条件 —— 表现就是原地抖、不追也不打。
      2) 误差大时原地对准，误差小时“边走边修”，兼顾转向和推进。
      3) **激光保护**：车头扇区内障碍近于 obstacle_stop_distance 时停止前进，
         改朝两侧更空的一侧转，保证不撞墙。
      4) 只有朝向误差足够小、且距离在射程内才开火；开火角还会按
         “弹道半宽 / 距离”收紧，避免远距离空放浪费弹药。
         子弹是否命中由裁判判定（含遮挡检查，打不穿墙）。

    私有参数（写在 config/car/*.yaml，或用 ~attack_* 覆盖）：
        ~attack_speed             追击线速度，默认 0.35 m/s
        ~attack_max_angular       最大角速度，默认 1.2 rad/s
        ~attack_align_gain        转向比例增益，默认 1.6
        ~attack_yaw_tolerance     判定“已经对准”的误差，默认 0.12 rad
        ~attack_fire_angle        开火允许的最大朝向误差，默认 0.06 rad
        ~attack_fire_range        超过该距离不开火，默认 3.0 m
        ~attack_stop_distance     离目标多近就停下开火，默认 0.6 m
        ~attack_obstacle_stop     前方障碍停车距离，默认 0.5 m
    """

    def __init__(self, skill_manager):
        super(AttackSkill, self).__init__(skill_manager)
        self.target_x = 0.0
        self.target_y = 0.0

        self.yaw_tolerance = 0.12
        self.align_gain = 1.6
        self.max_angular_speed = 1.2
        self.attack_speed = 0.35
        self.fire_angle = 0.06
        self.fire_range = 3.0
        self.hit_half_width = 0.1
        self.arrival_tolerance = 0.6
        self.fire_cooldown_s = 2.0
        self.pose_lost_timeout_s = 1.5
        self.obstacle_stop_distance = 0.5

        self._start_ts = None
        self._last_pose_ts = None
        self._steering = None

    def start(self, params=None):
        params = params or {}
        self.target_x = float(params.get("target_x", 0.0))
        self.target_y = float(params.get("target_y", 0.0))

        self.attack_speed = float(rospy.get_param("~attack_speed", params.get("attack_speed", 0.35)))
        self.max_angular_speed = abs(float(rospy.get_param("~attack_max_angular", 1.2)))
        self.align_gain = float(rospy.get_param("~attack_align_gain", 1.6))
        self.yaw_tolerance = float(rospy.get_param(
            "~attack_yaw_tolerance", params.get("yaw_tolerance", 0.12)))
        self.fire_angle = abs(float(rospy.get_param(
            "~attack_fire_angle", params.get("fire_angle", 0.06))))
        self.fire_range = float(rospy.get_param(
            "~attack_fire_range", params.get("fire_range", 3.0)))
        self.hit_half_width = float(rospy.get_param("~attack_hit_half_width", 0.1))
        self.arrival_tolerance = float(rospy.get_param(
            "~attack_stop_distance", params.get("arrival_tolerance", 0.6)))
        self.fire_cooldown_s = float(rospy.get_param(
            "~attack_fire_cooldown", params.get("fire_cooldown_s", 2.0)))
        self.pose_lost_timeout_s = float(params.get("pose_lost_timeout_s", 1.5))
        self.obstacle_stop_distance = float(rospy.get_param("~attack_obstacle_stop", 0.5))

        self._start_ts = rospy.Time.now().to_sec()
        self._last_pose_ts = None

        # 追击用统一的“比例转向 + 绕障”控制律（详见 skills/steering.py）
        self._steering = AvoidanceSteering(
            self.skill_manager,
            speed=self.attack_speed,
            max_angular=self.max_angular_speed,
            align_gain=self.align_gain,
            lookahead=float(rospy.get_param("~attack_lookahead", 1.1)),
            front_stop=self.obstacle_stop_distance,
            target_sector_deg=25.0,
        )
        self._steering.reset()
        self._status = RUNNING

        rospy.loginfo(
            "[%s] AttackSkill start: target=(%.2f, %.2f) speed=%.2f gain=%.1f fire_angle=%.3f",
            self.skill_manager.ns, self.target_x, self.target_y,
            self.attack_speed, self.align_gain, self.fire_angle,
        )

    def update(self):
        pose = self.skill_manager.get_current_pose()
        now = rospy.Time.now().to_sec()

        if pose is None:
            if self._last_pose_ts is None:
                self._last_pose_ts = self._start_ts if self._start_ts is not None else now
            if (now - self._last_pose_ts) > self.pose_lost_timeout_s:
                rospy.logwarn(
                    "[%s] AttackSkill failed: pose lost for %.2fs",
                    self.skill_manager.ns, now - self._last_pose_ts,
                )
                self.skill_manager.publish_stop_velocity()
                self._status = FAILED
                return self._status
            self._status = RUNNING
            return self._status

        self._last_pose_ts = now

        dx = self.target_x - pose.position.x
        dy = self.target_y - pose.position.y
        distance = math.hypot(dx, dy)

        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        desired_yaw = math.atan2(dy, dx)
        err = self._normalize_angle(desired_yaw - yaw)

        cmd = Twist()

        # ---- 1) 已经贴到目标附近：停下来对准并开火 ----
        if distance <= self.arrival_tolerance:
            if abs(err) > self.yaw_tolerance:
                cmd.angular.z = self._clamp(self.align_gain * err, self.max_angular_speed * 0.6)
                self.skill_manager.publish_cmd_vel(cmd)
            else:
                self.skill_manager.publish_stop_velocity()
            self._try_fire(pose, yaw, err, distance, now)
            self._status = RUNNING
            return self._status

        # ---- 2) 追击：统一的比例转向 + 绕障控制律 ----
        linear, angular, mode = self._steering.compute(yaw, desired_yaw, distance, now)
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.skill_manager.publish_cmd_vel(cmd)
        rospy.loginfo_throttle(
            2.0, "[%s] AttackSkill[%s] dist=%.2f err=%.2f lin=%.2f ang=%.2f",
            self.skill_manager.ns, mode, distance, err, linear, angular,
        )

        self._try_fire(pose, yaw, err, distance, now)
        self._status = RUNNING
        return self._status

    def _try_fire(self, pose, yaw, err, distance, now):
        """对准、在射程内、且前方没有遮挡时才开火。"""
        if distance > self.fire_range:
            return

        # 用激光沿“目标方位”做一次遮挡预判：比目标还近说明中间隔着东西，别浪费弹药
        clearance = self.skill_manager.min_range_in_sector(math.degrees(err), 20.0)
        if clearance is not None and clearance < distance - 0.25:
            return

        allowed = self.fire_angle
        if distance > 0.1 and self.hit_half_width > 0.0:
            # 留 20% 余量：裁判判定是“到弹道中心线的垂距 < hit_width”，
            # 贴着边界开枪很容易刚好落在线外，白费一发弹药。
            allowed = min(allowed, 0.8 * self.hit_half_width / distance)
        if abs(err) > allowed:
            return

        if not self.skill_manager.can_fire(self.fire_cooldown_s):
            return

        self.skill_manager.publish_fire_event(
            x=pose.position.x, y=pose.position.y, yaw=yaw,
        )
        self.skill_manager.note_fired()
        rospy.loginfo(
            "[%s] AttackSkill fire_event: pose=(%.2f, %.2f) yaw=%.2f dist=%.2f err=%.3f",
            self.skill_manager.ns, pose.position.x, pose.position.y, yaw, distance, err,
        )

    def stop(self):
        self.skill_manager.publish_stop_velocity()
        rospy.loginfo("[%s] AttackSkill stopped", self.skill_manager.ns)

    @staticmethod
    def _clamp(value, limit):
        limit = abs(float(limit))
        return max(-limit, min(float(value), limit))

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))
