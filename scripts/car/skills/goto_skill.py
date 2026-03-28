#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

from skills.base_skill import BaseSkill, RUNNING, SUCCESS, FAILED


class GoToSkill(BaseSkill):
    """通过 move_base_simple/goal 导航到指定 (x, y) 目标。

    当 move_base 返回 SUCCEEDED（status == 3）时结束为 SUCCESS。
    当 move_base 返回失败状态时结束为 FAILED。
    """

    # move_base 的 GoalStatus 状态码
    _STATUS_SUCCEEDED = 3
    _STATUS_FAILED = {4, 5, 8, 9}  # ABORTED、REJECTED、LOST 等

    def __init__(self, skill_manager, frame_id="map"):
        super(GoToSkill, self).__init__(skill_manager)
        self.target_x = 0.0
        self.target_y = 0.0
        self.frame_id = str(frame_id)

    def start(self, params=None):
        params = params or {}
        self.target_x = float(params.get("target_x", 0.0))
        self.target_y = float(params.get("target_y", 0.0))

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = self.target_x
        goal.pose.position.y = self.target_y
        goal.pose.position.z = 0.0
        goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.skill_manager.reset_nav_status()
        self.skill_manager.publish_nav_goal(goal)
        self._status = RUNNING

        rospy.loginfo(
            "[%s] GoToSkill start: target=(%.2f, %.2f)",
            self.skill_manager.ns, self.target_x, self.target_y,
        )

    def update(self):
        nav_status = self.skill_manager.nav_status_code

        if nav_status == self._STATUS_SUCCEEDED:
            self._status = SUCCESS
        elif nav_status in self._STATUS_FAILED:
            rospy.logwarn(
                "[%s] GoToSkill failed: move_base status=%d",
                self.skill_manager.ns, nav_status,
            )
            self._status = FAILED
        else:
            self._status = RUNNING

        return self._status

    def stop(self):
        # 通过发布零速度进行取消；move_base 将进入空闲。
        self.skill_manager.publish_stop_velocity()
