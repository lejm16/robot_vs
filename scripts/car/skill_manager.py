#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import threading

import actionlib  # 预留给后续 action client 集成使用。
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from robot_vs.msg import BattleMacroState
from robot_vs.msg import FireEvent
from robot_vs.msg import RobotState

from skills.base_skill import RUNNING, FAILED


class SkillManager(object):
    """管理小车技能所需的 ROS 发布器与订阅器。

    职责：
    - 向 /<ns>/move_base_simple/goal 发布导航目标
    - 向 /<ns>/cmd_vel 发布速度指令
    - 订阅 /<ns>/move_base/result 跟踪导航结果
    - 提供技能对象的工厂创建方法
    """

    def __init__(self, ns):
        self.ns = str(ns)
        self._lock = threading.RLock()

        self.nav_status_code = -1  # -1 表示尚未收到导航结果
        # 位姿来源：amcl_pose（地图系，优先）与 odom（里程计，兜底）
        # 之前 odom 只在第一次被采用（_latest_pose is None 时），一旦 amcl 没起来，
        # 上报的位姿就永久冻结在出生点，裁判那边所有车都在 (0,0)，永远打不中。
        self._latest_pose = None        # 当前生效的位姿（兼容旧引用）
        self._amcl_pose = None
        self._amcl_pose_stamp = None
        self._odom_pose = None
        self._odom_pose_stamp = None
        self._pose_source_timeout_s = float(rospy.get_param("~pose_source_timeout_s", 2.0))
        self._pose_warn_stamp = 0.0
        self._latest_twist = Twist()
        self._map_info = None
        self._map_data = None
        self._occ_threshold = 50
        self._latest_scan = None
        self._latest_scan_stamp = None
        self._scan_timeout_s = float(rospy.get_param("~scan_timeout_s", 1.0))
        self._nav_fallback_until = 0.0
        self._last_fire_ts = None

        self.active_skill = None
        self.active_action = "NONE"

        self._feedback = {
            "task_id": 0,
            "current_action": "NONE",
            "task_status": "IDLE",
            "mode": 0,
            "reason": "",
        }

        self.team = int(rospy.get_param("~team", 0))
        self.default_hp = float(rospy.get_param("~default_hp", 100.0))
        self.default_ammo = float(rospy.get_param("~default_ammo", 50.0))
        self.hp = float(self.default_hp)
        self.ammo = float(self.default_ammo)
        self.is_alive = True

        self._goal_pub = rospy.Publisher(
            "/{}/move_base_simple/goal".format(self.ns),
            PoseStamped,
            queue_size=1,
        )
        self._cancel_pub = rospy.Publisher(
            "/{}/move_base/cancel".format(self.ns),
            GoalID,
            queue_size=1,
        )
        self._cmd_vel_pub = rospy.Publisher(
            "/{}/cmd_vel".format(self.ns),
            Twist,
            queue_size=1,
        )

        # 死亡锁存：保证“死亡处理”只触发一次
        self._dead_latched = False

        # 死亡后持续发布 stop 的定时器（默认 None，死亡时创建）
        self._dead_stop_timer = None
        self._dead_stop_hz = float(rospy.get_param("~dead_stop_hz", 20.0))
        self._state_pub = rospy.Publisher(
            "/{}/robot_state".format(self.ns),
            RobotState,
            queue_size=10,
        )
        self._fire_event_pub = rospy.Publisher(
            "/{}/fire_event".format(self.ns),
            FireEvent,
            queue_size=10,
        )

        self._odom_sub = rospy.Subscriber(
            "/{}/odom".format(self.ns),
            Odometry,
            self._odom_cb,
            queue_size=10,
        )
        self._amcl_sub = rospy.Subscriber(
            "/{}/amcl_pose".format(self.ns),
            PoseWithCovarianceStamped,
            self._amcl_pose_cb,
            queue_size=10,
        )
        self._nav_result_sub = rospy.Subscriber(
            "/{}/move_base/result".format(self.ns),
            MoveBaseActionResult,
            self._nav_result_cb,
            queue_size=10,
        )
        self._macro_state_sub = rospy.Subscriber(
            "/referee/macro_state",
            BattleMacroState,
            self._macro_state_cb,
            queue_size=10,
        )
        self._map_sub = rospy.Subscriber(
            "/map",
            OccupancyGrid,
            self._map_cb,
            queue_size=1,
        )
        self._scan_sub = rospy.Subscriber(
            "/{}/scan".format(self.ns),
            LaserScan,
            self._scan_cb,
            queue_size=1,
        )

        self._state_timer = rospy.Timer(rospy.Duration(0.1), self._publish_robot_state)

        rospy.loginfo("[%s] SkillManager initialised", self.ns)

    # ------------------------------------------------------------------
    # 发布器辅助方法
    # ------------------------------------------------------------------

    def publish_nav_cancel(self):
        """取消 move_base 的所有目标。"""
        try:
            self._cancel_pub.publish(GoalID())  # 空 GoalID = cancel all
        except Exception as exc:
            rospy.logwarn("[%s] publish_nav_cancel failed: %s", self.ns, exc)

    def _dead_stop_tick(self, _event):
        """
        死亡后持续执行：反复发布 0 速度，防止 move_base/其他节点残留输出把车带跑。
        """
        # 只要不存活就持续压制
        with self._lock:
            alive = bool(self.is_alive)
        if alive:
            return
        self.publish_stop_velocity()

    def _enter_dead_state(self):
        """
        死亡瞬间触发一次：cancel move_base + stop 当前技能 + 启动持续 stop 定时器
        """
        rospy.logwarn("[%s] detected DEAD -> cancel move_base and STOP,无问题只做提示其死亡", self.ns)

        # 1) 先取消导航，避免 move_base 继续输出 cmd_vel
        self.publish_nav_cancel()

        # 2) 立刻发一次 stop
        self.publish_stop_velocity()

        # 3) 停掉当前技能（避免技能继续 update 发指令/发 fire_event）
        self.stop_active_skill()

        # 4) 强制状态显示为 STOP（可选，但有助于日志与 RobotState）
        self.active_action = "STOP"
        try:
            self.active_skill = self.make_skill("STOP", {})
            self.active_skill.start({})
        except Exception as exc:
            rospy.logwarn("[%s] start StopSkill on death failed: %s", self.ns, exc)
            self.active_skill = None

        # 5) 启动“死亡持续 stop”定时器（只启动一次）
        if self._dead_stop_timer is None:
            period = 1.0 / max(1.0, float(self._dead_stop_hz))
            self._dead_stop_timer = rospy.Timer(rospy.Duration(period), self._dead_stop_tick)

    def publish_nav_goal(self, goal):
        """向 move_base_simple 发布 PoseStamped 目标。死亡后禁止导航。"""
        with self._lock:
            alive = bool(self.is_alive)
        if not alive:
            return
        self._goal_pub.publish(goal)

    def publish_stop_velocity(self):
        """发布零速度 Twist，使机器人立即停止。"""
        self._cmd_vel_pub.publish(Twist())

    def publish_cmd_vel(self, cmd_vel):
        with self._lock:
            alive = bool(self.is_alive)
        if not alive:
            self._cmd_vel_pub.publish(Twist())
            return
        self._cmd_vel_pub.publish(cmd_vel)

    def cancel_nav_goal(self):
        """取消当前 move_base 目标，并重置本地导航状态。"""
        self.publish_nav_cancel()
        self.reset_nav_status()

    def publish_fire_event(self, x, y, yaw):
        with self._lock:
            alive = bool(self.is_alive)
            ammo = float(self.ammo)
        if (not alive) or ammo <= 0.0:
            return

        msg = FireEvent()
        msg.shooter_ns = self.ns
        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(yaw)
        self._fire_event_pub.publish(msg)

    def can_fire(self, cooldown_s):
        """开火冷却判断。

        冷却状态必须挂在 SkillManager（车一级）上：技能对象会随着任务切换被重建，
        如果把 `_last_fire_ts` 放在技能里，敌人每移动一点就换任务、冷却被重置，
        弹药会在几秒内被打光。
        """
        with self._lock:
            if self._last_fire_ts is None:
                return True
            return (rospy.Time.now().to_sec() - float(self._last_fire_ts)) >= float(cooldown_s)

    def note_fired(self):
        with self._lock:
            self._last_fire_ts = rospy.Time.now().to_sec()

    # ------------------------------------------------------------------
    # 导航状态
    # ------------------------------------------------------------------

    def reset_nav_status(self):
        """发送新目标前清空上一次导航结果。"""
        self.nav_status_code = -1

    def _nav_result_cb(self, msg):
        self.nav_status_code = msg.status.status

    def _map_cb(self, msg):
        with self._lock:
            self._map_info = {
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution
            }
            self._map_data = list(msg.data)
            self._occ_threshold = int(rospy.get_param("~map_occ_threshold", 50))

    def is_blocked(self, x, y):
        """(x, y) 是否不可通行（占据格或未知格）。

        取值约定与 map_server 的 OccupancyGrid 一致：-1 未知、0~100 占据概率。
        地图还没收到时返回 False（不阻拦）。
        """
        with self._lock:
            info = self._map_info
            data = self._map_data
            threshold = self._occ_threshold
        if not info or data is None:
            return False

        resolution = float(info['resolution'])
        mx = int((float(x) - float(info['origin_x'])) / resolution)
        my = int((float(y) - float(info['origin_y'])) / resolution)
        if mx < 0 or my < 0 or mx >= int(info['width']) or my >= int(info['height']):
            return True
        value = int(data[my * int(info['width']) + mx])
        if value < 0:
            return True          # 未知区域按不可通行处理（与裁判 block_unknown 一致）
        return value >= threshold

    def nearest_free_point(self, x, y, max_radius=1.5, step=0.1):
        """目标点若落在障碍里，就近找一个可通行的替代点（螺旋搜索）。

        找不到就原样返回；地图不可用时也原样返回。
        """
        try:
            x = float(x)
            y = float(y)
        except (TypeError, ValueError):
            return x, y

        with self._lock:
            if self._map_info is None or self._map_data is None:
                return x, y

        if not self.is_blocked(x, y):
            return x, y

        rings = max(1, int(float(max_radius) / float(step)))
        for ring in range(1, rings + 1):
            radius = ring * float(step)
            for index in range(16):
                angle = index * math.pi / 8.0
                candidate_x = x + radius * math.cos(angle)
                candidate_y = y + radius * math.sin(angle)
                if not self.is_blocked(candidate_x, candidate_y):
                    rospy.loginfo(
                        "[%s] 目标点 (%.2f, %.2f) 落在障碍/未知区域，改到 (%.2f, %.2f)",
                        self.ns, x, y, candidate_x, candidate_y,
                    )
                    return candidate_x, candidate_y
        return x, y

    def _scan_cb(self, msg):
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._latest_scan = msg
            self._latest_scan_stamp = self._msg_stamp(msg, now)

    def get_scan(self):
        """返回最近一帧激光；超过 scan_timeout_s 没更新则视为不可用。"""
        with self._lock:
            if self._latest_scan is None or self._latest_scan_stamp is None:
                return None
            if rospy.Time.now().to_sec() - float(self._latest_scan_stamp) > self._scan_timeout_s:
                return None
            return self._latest_scan

    def min_range_in_sector(self, center_deg, half_width_deg):
        """指定扇区（以车头为 0°，逆时针为正）内的最近障碍距离。

        返回 None 表示该方向上没有可用读数（无激光或全是无效值），
        调用方应把它当作“未知”，而不是“没有障碍”。
        """
        scan = self.get_scan()
        if scan is None:
            return None

        center = math.radians(float(center_deg))
        half_width = math.radians(abs(float(half_width_deg)))
        best = None
        angle = float(scan.angle_min)
        for value in scan.ranges:
            delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
            angle += float(scan.angle_increment)
            if abs(delta) > half_width:
                continue
            try:
                distance = float(value)
            except (TypeError, ValueError):
                continue
            if distance != distance:  # NaN
                continue
            if distance < float(scan.range_min) or distance > float(scan.range_max):
                continue
            if best is None or distance < best:
                best = distance
        return best

    def _odom_cb(self, msg):
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._latest_twist = msg.twist.twist
            self._odom_pose = msg.pose.pose
            self._odom_pose_stamp = self._msg_stamp(msg, now)

    def _amcl_pose_cb(self, msg):
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._amcl_pose = msg.pose.pose
            self._amcl_pose_stamp = self._msg_stamp(msg, now)

    @staticmethod
    def _msg_stamp(msg, fallback):
        """取消息头时间戳，取不到或为 0 时用当前时间。"""
        try:
            stamp = msg.header.stamp.to_sec()
        except Exception:
            return fallback
        return stamp if stamp > 0.0 else fallback

    def _resolve_pose(self):
        """选出当前可用的位姿：amcl 新鲜就用 amcl，否则退回 odom。

        调用方需已持有 self._lock（get_current_pose / _publish_robot_state 都持锁）。
        """
        now = rospy.Time.now().to_sec()
        if (self._amcl_pose is not None
                and self._amcl_pose_stamp is not None
                and (now - self._amcl_pose_stamp) <= self._pose_source_timeout_s):
            return self._amcl_pose

        if self._odom_pose is not None:
            if now - self._pose_warn_stamp > 10.0:
                self._pose_warn_stamp = now
                rospy.logwarn(
                    "[%s] 没有可用的 amcl_pose，先用 odom 位姿参与决策与裁判判定。"
                    "请检查 map_server(/map)、amcl(/%s/amcl_pose)、/%s/scan 与 tf "
                    "(map -> %s/odom -> %s/base_footprint)。",
                    self.ns, self.ns, self.ns, self.ns, self.ns,
                )
            return self._odom_pose

        return None

    def _extract_self_macro_state(self, team_state):
        if team_state is None:
            return None

        robot_ns = getattr(team_state, "robot_ns", [])
        hp = getattr(team_state, "hp", [])
        ammo = getattr(team_state, "ammo", [])
        alive = getattr(team_state, "alive", [])

        size = min(len(robot_ns), len(hp), len(ammo), len(alive))
        for idx in range(size):
            if str(robot_ns[idx]).strip().strip("/") == self.ns.strip().strip("/"):
                return float(hp[idx]), float(ammo[idx]), bool(alive[idx])
        return None

    def _macro_state_cb(self, msg):
        state = self._extract_self_macro_state(msg.red)
        if state is None:
            state = self._extract_self_macro_state(msg.blue)
        if state is None:
            return

        hp, ammo, alive = state
        hp = max(0.0, float(hp))
        ammo = max(0.0, float(ammo))
        new_alive = bool(alive and hp > 0.0)

        just_died = False
        came_back = False
        with self._lock:
            prev_alive = bool(self.is_alive)

            self.hp = hp
            self.ammo = ammo
            self.is_alive = new_alive

            # 仅在“从活->死”的瞬间触发一次
            if prev_alive and (not new_alive) and (not self._dead_latched):
                self._dead_latched = True
                just_died = True
            # 复活（比如裁判自动复位开下一局）：解锁死亡锁存，否则第二局再死一次
            # 就不会再执行 cancel + 持续刹车了
            elif (not prev_alive) and new_alive and self._dead_latched:
                self._dead_latched = False
                came_back = True

        # 注意：不要在锁内做 stop/cancel（避免潜在死锁）
        if just_died:
            self._enter_dead_state()
        elif came_back:
            rospy.loginfo("[%s] 检测到复活（裁判复位），清除死亡锁存", self.ns)

    # ------------------------------------------------------------------
    # 技能生命周期与工厂
    # ------------------------------------------------------------------

    def make_skill(self, action_type, task):
        """根据 *action_type* 创建对应技能实例。

        参数：
            action_type (str): 例如 "GOTO"、"STOP"
            task (dict): 来自 TaskDispatcher 的完整任务字典

        返回：
            BaseSkill 子类实例；若动作未知则回退为 StopSkill。
        """
        # 在此处导入以避免模块加载阶段出现循环依赖。
        from skills.goto_skill import GoToSkill
        from skills.stop_skill import StopSkill
        from skills.attack_skill import AttackSkill
        from skills.rotate_skill import RotateSkill
        from skills.retreat_skill import RetreatSkill   # 新增

        action = str(action_type).upper()
        if action == "GOTO":
            return GoToSkill(self)
        elif action == "STOP":
            return StopSkill(self)
        elif action == "ATTACK":
            return AttackSkill(self)
        elif action == "ROTATE":
            return RotateSkill(self)
        elif action == "RETREAT":           # 新增
            return RetreatSkill(self)       # 新增
        else:
            rospy.logwarn(
                "[%s] SkillManager: unknown action_type '%s', defaulting to StopSkill",
                self.ns, action_type,
            )
            return StopSkill(self)

    def switch_skill(self, action_type, task):
        with self._lock:
            alive = bool(self.is_alive)

        if not alive:
            # 死亡后：任何任务都强制变成 STOP，但不要 return
            action_type = "STOP"
            task = task or {}
            self.publish_nav_cancel()

        self.stop_active_skill()
        self.active_action = str(action_type).upper()
        self.active_skill = self.make_skill(action_type, task)
        try:
            self.active_skill.start(task)
        except Exception as exc:
            rospy.logwarn("[%s] skill.start failed: %s", self.ns, exc)
            self.active_skill = self.make_skill("STOP", task)
            self.active_action = "STOP"
            try:
                self.active_skill.start(task)
            except Exception as stop_exc:
                rospy.logwarn("[%s] fallback StopSkill start failed: %s", self.ns, stop_exc)

    def update_active_skill(self):
        if self.active_skill is None:
            return RUNNING
        try:
            return self.active_skill.update()
        except Exception as exc:
            rospy.logwarn("[%s] skill.update failed: %s", self.ns, exc)
            return FAILED

    def stop_active_skill(self):
        if self.active_skill is None:
            return
        try:
            self.active_skill.stop()
        except Exception as exc:
            rospy.logwarn("[%s] skill.stop failed: %s", self.ns, exc)
        self.active_skill = None

    def set_task_feedback(self, task_id, current_action, task_status, mode):
        with self._lock:
            self._feedback["task_id"] = int(task_id)
            self._feedback["current_action"] = str(current_action)
            self._feedback["task_status"] = str(task_status)
            self._feedback["mode"] = int(mode)

    def get_current_pose(self):
        with self._lock:
            self._latest_pose = self._resolve_pose()
            return self._latest_pose

    def get_current_yaw(self):
        pose = self.get_current_pose()
        if pose is None:
            return None
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def get_map_info(self):
        with self._lock:
            return self._map_info

    # ------------------------------------------------------------------
    # move_base 失联时的降级开关
    # ------------------------------------------------------------------
    def activate_nav_fallback(self, duration_s=60.0):
        """move_base 连续失败时，把这台车切到“自带绕障直行”一段时间。"""
        self._nav_fallback_until = rospy.Time.now().to_sec() + float(duration_s)
        rospy.logwarn(
            "[%s] move_base 不可用（多半是 amcl/地图/TF 链路问题），"
            "本车改用自带激光绕障直行 %.0f 秒", self.ns, duration_s,
        )

    def nav_fallback_active(self):
        return rospy.Time.now().to_sec() < self._nav_fallback_until

    # ------------------------------------------------------------------
    # RobotState 发布
    # ------------------------------------------------------------------

    def _publish_robot_state(self, _event):
        msg = RobotState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.robot_ns = self.ns
        msg.team = self.team
        msg.hp = self.default_hp
        msg.ammo = self.default_ammo
        msg.alive = True
        msg.in_combat = (self.active_action == "ATTACK")
        msg.yaw = 0.0
        with self._lock:
            msg.hp = float(self.hp)
            msg.ammo = float(self.ammo)
            msg.alive = bool(self.is_alive)
            pose = self._resolve_pose()
            self._latest_pose = pose
            if pose is not None:
                msg.pose = pose
                q = pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                msg.yaw = float(yaw)
            msg.twist = self._latest_twist
            msg.current_task_id = self._feedback["task_id"]
            msg.current_action = self._feedback["current_action"]
            msg.task_status = self._feedback["task_status"]
            msg.mode = self._feedback["mode"]

        self._state_pub.publish(msg)
        
         # 保险：死亡状态下持续 stop（即使 dead_stop_timer 因某种原因没启动）
        if not msg.alive:
            self.publish_stop_velocity()
