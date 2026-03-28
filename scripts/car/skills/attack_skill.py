#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from skills.base_skill import BaseSkill, RUNNING, SUCCESS


class AttackSkill(BaseSkill):
    """朝向目标并模拟攻击延时，完成后返回成功。"""

    def __init__(self, skill_manager):
        super(AttackSkill, self).__init__(skill_manager)
        self.target_x = 0.0
        self.target_y = 0.0
        self.start_time = None
        self.attack_delay_s = 1.5

    def start(self, params=None):
        params = params or {}
        self.target_x = float(params.get("target_x", 0.0))
        self.target_y = float(params.get("target_y", 0.0))
        self.attack_delay_s = float(params.get("attack_delay_s", 1.5))#delay default
        self.start_time = rospy.Time.now().to_sec()
        self._status = RUNNING

        rospy.loginfo(
            "[%s] AttackSkill start: target=(%.2f, %.2f) delay=%.2fs",
            self.skill_manager.ns, self.target_x, self.target_y, self.attack_delay_s,
        )

    def update(self):
        pose = self.skill_manager.get_current_pose()
        now = rospy.Time.now().to_sec()

        cmd = Twist()
        if pose is not None:
            q = pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            desired_yaw = math.atan2(self.target_y - pose.position.y, self.target_x - pose.position.x)
            err = self._normalize_angle(desired_yaw - yaw)

            if abs(err) > 0.2:
                cmd.angular.z = 0.7 if err > 0.0 else -0.7
                self.skill_manager.publish_cmd_vel(cmd)
                self._status = RUNNING
                return self._status

        self.skill_manager.publish_stop_velocity()
        rospy.loginfo(
            "[%s] AttackSkill: aligned with target, (shooting) waiting for attack delay (%.2fs elapsed)",
            self.skill_manager.ns, now - self.start_time,
        )

        if self.start_time is None:
            self.start_time = now

        if (now - self.start_time) >= self.attack_delay_s:
            self._status = SUCCESS
        else:
            self._status = RUNNING
        return self._status

    def stop(self):
        self.skill_manager.publish_stop_velocity()

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))
