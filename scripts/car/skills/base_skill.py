#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 技能状态常量
RUNNING = "RUNNING"
SUCCESS = "SUCCESS"
FAILED = "FAILED"


class BaseSkill(object):
    """所有小车技能的抽象基类。

        每个技能表示一个原子行为（如 GoTo、Stop）。
        生命周期：
            1) start()  - 技能激活时调用一次
            2) update() - 每个 tick 调用一次，返回 RUNNING / SUCCESS / FAILED
            3) stop()   - 技能取消或结束时调用一次
    """

    def __init__(self, skill_manager):
        self.skill_manager = skill_manager
        self._status = RUNNING

    @property
    def status(self):
        return self._status

    def start(self, params=None):
        """初始化技能；在第一次 update() 前调用一次。"""
        pass

    def update(self):
        """执行一步技能逻辑。

        返回：
            str: RUNNING、SUCCESS 或 FAILED
        """
        raise NotImplementedError("Subclasses must implement update()")

    def stop(self):
        """技能取消或结束时执行清理。"""
        pass
