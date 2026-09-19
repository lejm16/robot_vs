#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from robot_vs.msg import TaskCommand


try:
	text_type = unicode  # type: ignore[name-defined]
	binary_type = str
except NameError:
	text_type = str
	binary_type = bytes


class TaskDispatcher(object):
	"""将团队任务发布到 /<ns>/car_task 话题。"""

	def __init__(self, my_cars=None, default_timeout=2.0):
		if my_cars is None:
			my_cars = rospy.get_param("~my_cars", [])
		if not isinstance(my_cars, list):
			my_cars = []

		self.my_cars = list(my_cars)
		self.default_timeout = float(default_timeout)
		self._publishers = {}
		self._task_seq = 0
		self._last_task_signature = {}
		self._last_task_id = {}
		self._last_logged_id = {}

		# 场地边界：所有下发的目标点都会被夹进这个矩形，防止小车往场外开
		# 默认值对应 worlds/world0.world（围墙 x=±4.05, y=±2.0，留出车身余量）
		self.arena_min_x = float(rospy.get_param("~arena_min_x", -3.9))
		self.arena_max_x = float(rospy.get_param("~arena_max_x", 3.9))
		self.arena_min_y = float(rospy.get_param("~arena_min_y", -1.9))
		self.arena_max_y = float(rospy.get_param("~arena_max_y", 1.9))

		for ns in self.my_cars:
			self._ensure_publisher(ns)

	def _to_number(self, value, default=0.0):
		try:
			return float(value)
		except (TypeError, ValueError):
			return float(default)

	def _clamp_to_arena(self, value, low, high):
		"""把坐标夹到场地范围内（参数写反了也能自愈）。"""
		if high < low:
			low, high = high, low
		return max(low, min(self._to_number(value, low), high))

	def _ensure_publisher(self, ns):
		if ns in self._publishers:
			return self._publishers[ns]

		topic = "/{}/car_task".format(ns)
		self._publishers[ns] = rospy.Publisher(topic, TaskCommand, queue_size=10)
		rospy.loginfo("TaskDispatcher publisher created: %s", topic)
		return self._publishers[ns]

	def _normalize_tasks(self, tasks):
		if tasks is None:
			raise ValueError("tasks must not be None")

		if isinstance(tasks, dict):
			return tasks

		# 兼容旧版 list 格式：
		# [{"car":"robot_x","type":"idle","reason":"..."}, ...]
		rospy.logwarn("tasks is a list, using legacy format normalization. Consider updating LLM output to dict format.")
		if isinstance(tasks, list):
			normalized = {}
			for item in tasks:
				if not isinstance(item, dict):
					continue
				ns = item.get("car")
				if not ns:
					continue
				normalized[ns] = {
					"action": self._to_text(item.get("type", "STOP"), "STOP").upper(),
					"target": {
						"x": float(item.get("target_x", 0.0)),
						"y": float(item.get("target_y", 0.0)),
						"yaw": float(item.get("target_yaw", 0.0)),
					},
					"mode": int(item.get("mode", 0)),
					"reason": self._to_text(item.get("reason", "legacy format"), "legacy format"),
					"timeout": float(item.get("timeout", self.default_timeout)),
				}
			return normalized

		raise ValueError("tasks must be dict or list")

	def _next_task_id(self):
		self._task_seq += 1
		return self._task_seq

	def _safe_stop_task(self, reason):
		return {
			"action": "STOP",
			"target": {"x": 0.0, "y": 0.0,"yaw": 0.0},
			"mode": 0,
			"reason": reason,
			"timeout": self.default_timeout,
		}

	def _task_signature(self, task):
		action = self._to_text(task.get("action", "STOP"), "STOP")
		target = task.get("target", {})
		if not isinstance(target, dict):
			target = {}

		target_x = float(target.get("x", 0.0))
		target_y = float(target.get("y", 0.0))
		target_yaw = float(target.get("yaw", 0.0))
		mode = int(task.get("mode", 0))
		timeout = float(task.get("timeout", self.default_timeout))

		return (action, target_x, target_y, target_yaw, mode, timeout)

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

	def _assign_task_id(self, ns, task):
		signature = self._task_signature(task)
		last_signature = self._last_task_signature.get(ns)

		if last_signature == signature and ns in self._last_task_id:
			return self._last_task_id[ns]

		if "task_id" in task:
			task_id = int(task.get("task_id"))
		else:
			task_id = self._next_task_id()
		self._last_task_signature[ns] = signature
		self._last_task_id[ns] = task_id
		return task_id

	def _build_task_msg(self, ns, task):
		msg = TaskCommand()
		msg.task_id = int(self._assign_task_id(ns, task))
		msg.action_type = self._to_text(task.get("action", "STOP"), "STOP")

		target = task.get("target", {})
		if not isinstance(target, dict):
			target = {}
		raw_x = self._to_number(target.get("x", 0.0), 0.0)
		raw_y = self._to_number(target.get("y", 0.0), 0.0)
		msg.target_x = self._clamp_to_arena(raw_x, self.arena_min_x, self.arena_max_x)
		msg.target_y = self._clamp_to_arena(raw_y, self.arena_min_y, self.arena_max_y)
		if abs(msg.target_x - raw_x) > 1e-6 or abs(msg.target_y - raw_y) > 1e-6:
			rospy.loginfo_throttle(
				5.0,
				"dispatch: 目标点 (%.2f, %.2f) 超出场地，已夹到 (%.2f, %.2f)",
				raw_x, raw_y, msg.target_x, msg.target_y,
			)
		msg.target_yaw = float(target.get("yaw", 0.0))

		msg.mode = int(task.get("mode", 0))
		msg.reason = self._to_text(task.get("reason", ""), "")
		msg.timeout = float(task.get("timeout", self.default_timeout))
		return msg

	def dispatch(self, tasks):
		normalized = self._normalize_tasks(tasks)

		if self.my_cars:
			target_ns_list = list(self.my_cars)
		else:
			target_ns_list = sorted(normalized.keys())

		for ns in target_ns_list:
			try:
				task = normalized.get(ns)
				if task is None:
					task = self._safe_stop_task("missing task for robot")

				pub = self._ensure_publisher(ns)
				msg = self._build_task_msg(ns, task)
				pub.publish(msg)

				# 决策循环 20 Hz，逐帧打印会刷爆 rosout；只在任务真的换了才打一条
				if self._last_logged_id.get(ns) != msg.task_id:
					self._last_logged_id[ns] = msg.task_id
					rospy.loginfo(
						"dispatch ns=%s task_id=%d action=%s target=(%.2f, %.2f) reason=%r",
						ns,
						msg.task_id,
						msg.action_type,
						msg.target_x,
						msg.target_y,
						msg.reason,
					)
			except Exception as exc:
				rospy.logwarn("dispatch failed for %s: %s", ns, exc)

	def send_stop(self, ns, reason="stop"):
		"""给指定小车发布一条 STOP 任务（供 Manager 兜底/结束比赛时调用）。"""
		try:
			pub = self._ensure_publisher(ns)
			msg = self._build_task_msg(ns, self._safe_stop_task(reason))
			pub.publish(msg)
		except Exception as exc:
			rospy.logwarn("send_stop failed for %s: %s", ns, exc)
