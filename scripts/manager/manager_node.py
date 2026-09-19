#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TeamManager - 红蓝双方团队管理主节点

核心循环：
  1) 观测全局状态（GlobalObserver）
  2) 格式化规划输入（BattleStateFormatter）
  3) 调用 LLMClient 进行博弈决策（含角色分配 + 目标选择 + 战术选择）
  4) 分发任务到各小车（TaskDispatcher）

频率设计（高低频分离）：
  - 主循环频率：由 loop_hz 控制（默认 20Hz）
  - 角色分配/目标选择：每帧执行（~20Hz）
  - 大模型调用：由 LLMClient 内部控制（use_llm=True 时每 5 秒一次）
"""

import copy
import json
import os

import rospy
from robot_vs.msg import GameState
from std_msgs.msg import String

from interfaces import BaseObserver, BaseFormatter, BasePlanner, BaseDispatcher
from battle_state_formatter import BattleStateFormatter
from global_observer import GlobalObserver
from llm_client import LLMClient
from task_dispatcher import TaskDispatcher


try:
    text_type = unicode  # type: ignore[name-defined]
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


class TeamManager(object):
    """ROS1 团队管理主节点。

    核心循环：
      1) 观测全局状态
      2) 格式化规划输入
      3) 调用 LLMClient 进行博弈决策（含角色分配、目标选择、战术选择）
      4) 分发任务
    """

    def __init__(self, team_color="red", my_cars=None, loop_hz=20.0,
                 state_timeout_s=2.0, default_patrol_points=None,
                 enemy_topic="/referee/enemy_state",
                 llm_enabled=False,
                 llm_service_url="http://127.0.0.1:8001/plan",
                 llm_timeout_s=8.0):
        if my_cars is None:
            my_cars = []
        self.team_color = str(team_color)
        self.my_cars = list(my_cars)
        self.loop_hz = float(loop_hz)
        self.state_timeout_s = float(state_timeout_s)
        self.default_patrol_points = list(default_patrol_points) if default_patrol_points else []
        self.enemy_topic = str(enemy_topic)
        self.llm_enabled = bool(llm_enabled)
        self.llm_service_url = str(llm_service_url)
        self.llm_timeout_s = float(llm_timeout_s)

        # ====== 初始化各组件 ======
        self.observer = GlobalObserver(
            my_cars=self.my_cars,
            state_timeout=self.state_timeout_s,
            enemy_topic=self.enemy_topic,
        )
        self.formatter = BattleStateFormatter()
        self.llm_client = LLMClient(
            patrol_points=(self.default_patrol_points or None),
            use_llm=self.llm_enabled,
            llm_service_url=self.llm_service_url,
            llm_timeout_s=self.llm_timeout_s,
        )
        self.dispatcher = TaskDispatcher(
            my_cars=self.my_cars,
        )

        # ====== 比赛状态同步 ======
        self._game_status = "IDLE"
        # 进入 IDLE 时是否还需要补发一次 STOP（避免残留任务让车继续跑）
        self._idle_stop_pending = True
        self._game_state_sub = rospy.Subscriber(
            "/game/state", GameState, self._on_game_state, queue_size=10
        )

        # ====== 叙事事件（发到 /game/narrative，事件式：任务发生变化时才发）======
        # 说明：仓库里目前没有订阅者，保留这个话题是为了后续接解说/录制；
        # 决策循环是 20 Hz，所以这里必须做“变化检测”，否则会以 100+ msg/s 空刷。
        self._narrative_pub = rospy.Publisher("/game/narrative", String, queue_size=100)
        self._narrative_signature = None

        # ====== 决策统计（用于调试） ======
        self._decision_count = 0
        self._last_role_log_time = 0.0
        self._role_log_interval = 3.0  # 每 3 秒打印一次角色分配

        rospy.loginfo(
            "TeamManager initialized: team_color=%s my_cars=%s loop_hz=%.3f state_timeout_s=%.2f enemy_topic=%s patrol_points=%s llm_enabled=%s llm_service_url=%s llm_timeout_s=%.2f",
            self.team_color,
            self.my_cars,
            self.loop_hz,
            self.state_timeout_s,
            self.enemy_topic,
            self.default_patrol_points,
            self.llm_enabled,
            self.llm_service_url,
            self.llm_timeout_s,
        )

    @classmethod
    def from_ros_params(cls):
        """从 ROS 参数服务器加载配置"""
        team_color = rospy.get_param("~team_color", "red")
        my_cars = rospy.get_param("~my_cars", [])
        loop_hz = rospy.get_param("~loop_hz", 20.0)  # 默认 20Hz
        state_timeout_s = rospy.get_param("~state_timeout_s", 2.0)
        default_patrol_points = rospy.get_param("~default_patrol_points", [])
        node_name = rospy.get_name().strip("/")
        default_enemy_topic = "/{}/enemy_state".format(node_name) if node_name else "/referee/enemy_state"
        enemy_topic = rospy.get_param("~enemy_topic", default_enemy_topic)
        llm_config = rospy.get_param("~llm", {})
        if not isinstance(llm_config, dict):
            llm_config = {}
        llm_enabled = rospy.get_param("~llm_enabled", llm_config.get("enabled", False))
        llm_service_url = rospy.get_param("~llm_service_url", llm_config.get("service_url", "http://127.0.0.1:8001/plan"))
        llm_timeout_s = rospy.get_param("~llm_timeout_s", llm_config.get("timeout_s", 8.0))

        cls._validate_params(
            team_color, my_cars, loop_hz, state_timeout_s,
            default_patrol_points, enemy_topic,
            llm_service_url, llm_timeout_s
        )
        return cls(
            team_color=team_color,
            my_cars=my_cars,
            loop_hz=loop_hz,
            state_timeout_s=state_timeout_s,
            default_patrol_points=default_patrol_points,
            enemy_topic=enemy_topic,
            llm_enabled=llm_enabled,
            llm_service_url=llm_service_url,
            llm_timeout_s=llm_timeout_s,
        )

    @staticmethod
    def _validate_params(team_color, my_cars, loop_hz, state_timeout_s,
                         default_patrol_points, enemy_topic,
                         llm_service_url, llm_timeout_s):
        """验证参数合法性"""
        if not isinstance(team_color, str):
            raise ValueError("~team_color must be a string")

        if not isinstance(my_cars, list):
            raise ValueError("~my_cars must be a list of strings")

        if not all(isinstance(car, str) and car for car in my_cars):
            raise ValueError("~my_cars must contain non-empty strings only")

        try:
            hz = float(loop_hz)
        except Exception:
            raise ValueError("~loop_hz must be a float")

        if hz <= 0.0:
            raise ValueError("~loop_hz must be > 0")

        try:
            timeout_s = float(state_timeout_s)
        except Exception:
            raise ValueError("~state_timeout_s must be a float")

        if timeout_s <= 0.0:
            raise ValueError("~state_timeout_s must be > 0")

        if not isinstance(default_patrol_points, list):
            raise ValueError("~default_patrol_points must be a list")

        if not isinstance(enemy_topic, str) or not enemy_topic:
            raise ValueError("~enemy_topic must be a non-empty string")

        if not isinstance(llm_service_url, str) or not llm_service_url:
            raise ValueError("~llm_service_url or ~llm.service_url must be a non-empty string")

        try:
            llm_timeout = float(llm_timeout_s)
        except Exception:
            raise ValueError("~llm_timeout_s or ~llm.timeout_s must be a float")

        if llm_timeout <= 0.0:
            raise ValueError("~llm_timeout_s or ~llm.timeout_s must be > 0")

    def build_fallback_tasks(self):
        """构建降级任务（当决策失败时使用）"""
        fallback = {}
        for ns in self.my_cars:
            fallback[ns] = {
                "action": "STOP",
                "target": {"x": 0.0, "y": 0.0},
                "mode": 0,
                "reason": "fallback_on_exception in manager.py",
                "timeout": 2.0,
            }
        return fallback

    def _on_game_state(self, msg):
        """比赛状态回调。裁判是比赛状态的唯一权威来源，直接跟随其发布的状态。"""
        new_status = str(msg.status)
        if new_status != self._game_status:
            rospy.loginfo(
                "[%s] game_status: %s -> %s", self.team_color, self._game_status, new_status
            )
            if new_status == "IDLE":
                # 比赛被中止/复位：下一次循环补发 STOP
                self._idle_stop_pending = True
        self._game_status = new_status

    def _send_stop_to_all(self, reason="match_ended"):
        """给所有小车发 STOP"""
        for ns in self.my_cars:
            self.dispatcher.send_stop(ns, reason)
        rospy.loginfo("[%s] STOP sent to %d robots (reason=%s)",
                      self.team_color, len(self.my_cars), reason)

    def _publish_narrative(self, message):
        """向 /game/narrative 发一条纯文本叙事"""
        try:
            if isinstance(message, dict):
                text = json.dumps(message, ensure_ascii=True)
            else:
                text = self._to_text(message, u"")
            if text_type is not str:
                payload = text.encode("utf-8")
            else:
                payload = text
            self._narrative_pub.publish(String(payload))
        except Exception:
            pass

    def _to_text(self, value, default=u""):
        if value is None:
            value = default
        try:
            if isinstance(value, text_type):
                return value
            if isinstance(value, binary_type):
                return value.decode("utf-8", "replace")
            return text_type(value)
        except Exception:
            try:
                return text_type(default)
            except Exception:
                return u""

    def _log_decision(self, tasks):
        """
        打印决策日志（角色分配、目标选择、战术选择）
        每 3 秒打印一次，避免刷屏
        """
        now = rospy.Time.now().to_sec()
        if now - self._last_role_log_time < self._role_log_interval:
            return
        self._last_role_log_time = now

        # 从 llm_client 中获取博弈状态
        roles = getattr(self.llm_client, 'roles', {})
        targets = getattr(self.llm_client, 'targets', {})
        tactic = getattr(self.llm_client, 'current_tactic', 'unknown')
        focus_target = getattr(self.llm_client, 'focus_target', None)

        rospy.loginfo("=" * 60)
        rospy.loginfo("[%s] 博弈决策状态", self.team_color.upper())
        rospy.loginfo("  战术: %s", tactic)
        rospy.loginfo("  集火目标: %s", focus_target if focus_target else "无")
        rospy.loginfo("  角色分配:")
        for car_id, role in roles.items():
            target_id = targets.get(car_id)
            target_str = target_id if target_id else "无目标"
            rospy.loginfo("    %s → %s → 目标: %s", car_id, role, target_str)
        rospy.loginfo("=" * 60)

    def run_cycle(self):
        """
        单次决策循环（只对存活车辆分配任务）
        """
        # 1. 观测全局状态
        state = self.observer.get_battle_state()

        # 2. 从 state 中提取存活车辆列表
        #    GlobalObserver.get_battle_state() 返回的是 dict：
        #    {"friendly": {ns: {"state": {...}, "stamp": .., "stale": bool}}, ...}
        friendly = state.get("friendly", {}) if isinstance(state, dict) else {}
        alive_cars = []
        for ns in self.my_cars:
            record = friendly.get(ns)
            car_state = record.get("state") if isinstance(record, dict) else None
            if not isinstance(car_state, dict):
                # 尚未收到该车状态：先按存活处理，交由超时/失联逻辑兜底
                alive_cars.append(ns)
                continue
            try:
                hp = float(car_state.get("hp", 100.0))
            except (TypeError, ValueError):
                hp = 100.0
            if bool(car_state.get("alive", True)) and hp > 0.0:
                alive_cars.append(ns)

        # 如果没有任何存活车辆，发送 STOP 并终止决策
        if not alive_cars:
            rospy.logwarn("[%s] 所有车辆已阵亡，停止决策", self.team_color)
            for ns in self.my_cars:
                self.dispatcher.send_stop(ns, "all_dead")
            return {}

        # 临时将 self.my_cars 替换为存活列表，以便后续组件使用
        original_my_cars = self.my_cars
        self.my_cars = alive_cars

        # 3. 格式化规划输入（使用存活车辆列表）
        prompt_input = self.formatter.build(state, self.team_color, self.my_cars)

        # 4. 调用 LLMClient 进行博弈决策
        tasks = self.llm_client.plan_tasks(prompt_input)

        # 5. 恢复原列表
        self.my_cars = original_my_cars

        # 6. 分发任务（只分发给存活车辆，tasks 中应该只包含存活车辆）
        self.dispatcher.dispatch(tasks)

        # 7. 打印决策日志
        self._log_decision(tasks)

        # 8. 发布叙事事件
        self._publish_narrative_events(tasks)

        self._decision_count += 1
        return tasks

    def _publish_narrative_events(self, tasks):
        """发布叙事事件到 /game/narrative"""
        signature = self._task_signature(tasks)
        if signature == self._narrative_signature:
            return
        self._narrative_signature = signature

        team_text = self._to_text(self.team_color, u"")

        # 1) 发布 Leader 战略理由（如果有）
        leader_order = getattr(self.llm_client, "last_leader_order", "")
        if leader_order:
            self._publish_narrative({
                "team": team_text,
                "event": "leader_order",
                "msg": u"[%s_leader] %s" % (team_text, self._to_text(leader_order, u"")),
            })

        # 2) 发布司令（Manager）的决策叙事
        actions_summary = []
        for ns, task in tasks.items():
            ns_text = self._to_text(ns, u"")
            action = self._to_text(task.get("action", "STOP"), u"STOP").upper()
            reason = self._to_text(task.get("reason", ""), u"")
            tgt = task.get("target", {})
            tgt_str = u"(%.2f,%.2f)" % (float(tgt.get("x", 0)), float(tgt.get("y", 0)))
            actions_summary.append(u"%s=%s%s" % (ns_text, action, tgt_str))
        self._publish_narrative(
            {
                "team": team_text,
                "event": "command",
                "msg": u"[%s_manager] order: %s" % (team_text, u", ".join(actions_summary)),
            },
        )

        # 3) 发布每条任务的叙事（含 reason）
        for ns, task in tasks.items():
            ns_text = self._to_text(ns, u"")
            action = self._to_text(task.get("action", "STOP"), u"STOP").upper()
            reason = self._to_text(task.get("reason", ""), u"")
            tgt = task.get("target", {})
            tgt_str = u"(%.2f, %.2f)" % (float(tgt.get("x", 0)), float(tgt.get("y", 0)))
            self._publish_narrative(
                {
                    "team": team_text,
                    "event": "command",
                    "msg": u"[%s] %s %s - %s" % (ns_text, action, tgt_str, reason),
                    "reason": reason,
                },
            )

    @staticmethod
    def _task_signature(tasks):
        """把任务字典压成可比较的元组，用来判断这一轮任务有没有变化。"""
        if not isinstance(tasks, dict):
            return ()
        items = []
        for ns, task in tasks.items():
            if not isinstance(task, dict):
                continue
            target = task.get("target")
            if not isinstance(target, dict):
                target = {}
            try:
                target_x = round(float(target.get("x", 0.0)), 2)
                target_y = round(float(target.get("y", 0.0)), 2)
            except (TypeError, ValueError):
                target_x, target_y = 0.0, 0.0
            items.append((
                str(ns),
                str(task.get("action", "STOP")).upper(),
                target_x,
                target_y,
                str(task.get("reason", "")),
            ))
        return tuple(sorted(items))

    def run(self):
        """主循环"""
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(1.0, "Manager loop: game_status=%s", self._game_status)

            if self._game_status == "FINISHED":
                # 比赛结束：持续发 STOP
                self._send_stop_to_all("match_ended")
                rate.sleep()
                continue

            # 比赛未开始：空转等待（取消注释）
            if self._game_status == "IDLE":
                # 未开赛或已中止：补发一次 STOP 后空转等待
                if self._idle_stop_pending:
                    self._send_stop_to_all("game_idle")
                    self._idle_stop_pending = False
                rate.sleep()
                continue

            # _game_status == "PLAYING": 正常规划
            self._idle_stop_pending = True
            try:
                self.run_cycle()
            except Exception as exc:
                rospy.logwarn("TeamManager cycle failed: %s", exc)
                fallback_tasks = self.build_fallback_tasks()
                try:
                    self.dispatcher.dispatch(copy.deepcopy(fallback_tasks))
                except Exception as dispatch_exc:
                    rospy.logwarn("Fallback dispatch failed: %s", dispatch_exc)
            rate.sleep()


def main():
    rospy.init_node("team_manager")

    try:
        manager = TeamManager.from_ros_params()
    except Exception as exc:
        rospy.logwarn("TeamManager param/init error: %s", exc)
        # 当参数非法时使用保守默认值，确保节点保持可运行。
        node_name = rospy.get_name().strip("/")
        default_enemy_topic = "/{}/enemy_state".format(node_name) if node_name else "/referee/enemy_state"
        manager = TeamManager(
            team_color="red",
            my_cars=[],
            loop_hz=20.0,
            state_timeout_s=5.0,
            enemy_topic=default_enemy_topic
        )

    manager.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
