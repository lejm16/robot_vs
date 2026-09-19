#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""共用的“朝目标点行驶 + 简单绕障”控制律（纯 cmd_vel）。

用于两类场景：攻击追击，以及 move_base 不可用时的 GOTO 回退。
不依赖 map / amcl / move_base，只依赖 /<ns>/scan 和当前位姿。

行为：
  1) 车头正前方过近 → 只转向不前进（保命）；
  2) 朝目标的射线在 lookahead 内被挡 → 进入绕行：把“绕行航向”固定在
     目标方位 ±90°（选两侧更空的一侧），沿这条固定航向直行，绕过去；
  3) 目标方向重新通畅、或绕行航向已对准 → 退出绕行，恢复追击；
  4) 正常时比例转向 + 前进，误差大时先原地对准。

关键点：绕行航向必须固定在**世界坐标系**里。如果每帧都用“当前目标方位 ±90°”
去追，目标方位会随车头一起旋转，车会陷入无限自转——这正是“车只在原地转、
不往前走”的典型成因。
"""

import math

import rospy


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AvoidanceSteering(object):
    """带简单绕障的追踪控制律。每次 compute() 返回 (linear_x, angular_z, mode)。"""

    def __init__(self, skill_manager, speed=0.35, max_angular=1.2, align_gain=1.6,
                 lookahead=1.1, front_stop=0.45, target_sector_deg=25.0,
                 detour_exit_deg=30.0):
        self.skill_manager = skill_manager
        self.speed = abs(float(speed))
        self.max_angular = abs(float(max_angular))
        self.align_gain = float(align_gain)
        self.lookahead = abs(float(lookahead))
        self.front_stop = abs(float(front_stop))
        self.target_sector_deg = abs(float(target_sector_deg))
        self.detour_exit_deg = abs(float(detour_exit_deg))

        self._detour_yaw = None
        self._warned_no_scan = False

    def reset(self):
        self._detour_yaw = None
        self._warned_no_scan = False

    def compute(self, yaw, desired_yaw, distance, now=None):
        """根据当前朝向与目标方位给出速度指令。"""
        err = normalize_angle(desired_yaw - yaw)

        front = self.skill_manager.min_range_in_sector(0.0, 30.0)
        if front is None and not self._warned_no_scan:
            self._warned_no_scan = True
            rospy.logwarn(
                "[%s] 读不到 /%s/scan，避障保护失效（仍会继续走）",
                self.skill_manager.ns, self.skill_manager.ns,
            )

        # 1) 正前方贴脸：只转向
        if front is not None and front < self.front_stop:
            self._detour_yaw = None
            return (0.0, self._clamp(self.align_gain * err), "front-guard")

        blocked = self._path_blocked(err, distance)

        # 2) 绕行中：沿固定的绕行航向直行
        if self._detour_yaw is not None:
            detour_err = normalize_angle(self._detour_yaw - yaw)
            if (not blocked) and abs(detour_err) < math.radians(self.detour_exit_deg):
                self._detour_yaw = None
            else:
                linear = self.speed * 0.8 if abs(detour_err) < 0.5 else 0.0
                return (linear, self._clamp(self.align_gain * detour_err), "detour")

        # 3) 需要绕行：把航向固定下来（世界系）
        if blocked:
            side = self._choose_side(err)
            self._detour_yaw = desired_yaw + side * math.pi / 2.0
            detour_err = normalize_angle(self._detour_yaw - yaw)
            return (0.0, self._clamp(self.align_gain * detour_err), "detour-start")

        # 4) 正常追击
        if abs(err) > 0.6:
            return (0.0, self._clamp(self.align_gain * err), "align")
        linear = self.speed if abs(err) <= 0.12 else self.speed * 0.5
        return (linear, self._clamp(self.align_gain * err), "pursue")

    # ------------------------------------------------------------------
    def _path_blocked(self, err, distance):
        """朝目标的射线是否在 lookahead 内被挡住。"""
        if distance <= 0.0:
            return False
        clearance = self.skill_manager.min_range_in_sector(
            math.degrees(err), self.target_sector_deg)
        if clearance is None:
            return False
        limit = min(max(distance - 0.25, 0.15), self.lookahead)
        return clearance < limit

    def _choose_side(self, err):
        """选更空的一侧绕行：+1 = 左，-1 = 右。"""
        deg = math.degrees(err)
        center = self.skill_manager.min_range_in_sector(deg, self.target_sector_deg)
        left = self.skill_manager.min_range_in_sector(deg + 75.0, 45.0)
        right = self.skill_manager.min_range_in_sector(deg - 75.0, 45.0)
        left_clear = center if left is None else left
        right_clear = center if right is None else right
        return 1.0 if left_clear >= right_clear else -1.0

    def _clamp(self, value):
        return max(-self.max_angular, min(float(value), self.max_angular))
