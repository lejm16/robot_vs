#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from skills.base_skill import BaseSkill, SUCCESS


class StopSkill(BaseSkill):
    """通过发布零速度立即停止机器人。

    在一个 tick 后返回 SUCCESS，表示停止指令已发送。
    当 task_engine 的当前任务仍为 STOP 时，会持续调用 update()，
    因此每个 tick 都会重复发布零速度指令。
    """

    def start(self, params=None):
        self.skill_manager.publish_stop_velocity()
        rospy.loginfo("[%s] StopSkill start", self.skill_manager.ns)

    def update(self):
        self.skill_manager.publish_stop_velocity()
        self._status = SUCCESS
        return self._status

    def stop(self):
        pass
