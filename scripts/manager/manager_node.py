#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy

import rospy

from battle_state_formatter import BattleStateFormatter
from global_observer import GlobalObserver
from llm_client import LLMClient
from task_dispatcher import TaskDispatcher


class TeamManager(object):
	"""ROS1 team manager main node.

	Core loop:
	  1) observe global state
	  2) format planner input
	  3) ask LLM planner for tasks
	  4) dispatch tasks
	"""

	def __init__(self, team_color, my_cars, loop_hz=0.2,
				 observer=None, formatter=None, llm_client=None, dispatcher=None):
		self.team_color = str(team_color)
		self.my_cars = list(my_cars)
		self.loop_hz = float(loop_hz)

		self.observer = observer if observer is not None else GlobalObserver()
		self.formatter = formatter if formatter is not None else BattleStateFormatter()
		self.llm_client = llm_client if llm_client is not None else LLMClient()
		self.dispatcher = dispatcher if dispatcher is not None else TaskDispatcher()

		rospy.loginfo(
			"TeamManager initialized: team_color=%s my_cars=%s loop_hz=%.3f",
			self.team_color,
			self.my_cars,
			self.loop_hz,
		)

	@classmethod
	def from_ros_params(cls):
		team_color = rospy.get_param("~team_color")
		my_cars = rospy.get_param("~my_cars")
		loop_hz = rospy.get_param("~loop_hz", 0.2)

		cls._validate_params(team_color, my_cars, loop_hz)
		return cls(team_color=team_color, my_cars=my_cars, loop_hz=loop_hz)

	@staticmethod
	def _validate_params(team_color, my_cars, loop_hz):
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

	def build_fallback_tasks(self):
		# Fallback keeps each robot in a safe idle state if planning failed.
		return [
			{
				"car": car,
				"type": "idle",
				"reason": "fallback_on_exception",
			}
			for car in self.my_cars
		]

	def run_cycle(self):
		state = self.observer.get_battle_state()
		prompt_input = self.formatter.build(state, self.team_color, self.my_cars)
		tasks = self.llm_client.plan_tasks(prompt_input)
		self.dispatcher.dispatch(tasks)
		return tasks

	def run(self):
		rate = rospy.Rate(self.loop_hz)
		while not rospy.is_shutdown():
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
	rospy.init_node("team_manager")# maybe error

	try:
		manager = TeamManager.from_ros_params()
	except Exception as exc:
		rospy.logwarn("TeamManager param/init error: %s", exc)
		# Keep node alive with conservative defaults when params are invalid.
		manager = TeamManager(team_color="unknown", my_cars=[], loop_hz=0.2)

	manager.run()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
