#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import random
import json

import rospy
import requests

from interfaces import BasePlanner


class LLMClient(BasePlanner):
    """
    博弈决策核心（带角色分配）
    功能：
    1. 角色分配（Attack / Support-1 / Support-2）
    2. 目标选择（集火威胁最大 / 压制其他）
    3. 战术选择（进攻/防守/平衡）
    4. 低血量撤退（HP < 20 → RETREAT）
    5. 阵亡车辆自动过滤（基于 HP <= 0 强制标记 alive=False）
    6. 击杀后动态响应：人数优势时追击（若有可见敌人则直接攻击），人数劣势时攻守分离
    """

    def __init__(self, patrol_points=None, use_llm=False,
                 llm_service_url="http://127.0.0.1:8001/plan",
                 llm_timeout_s=8.0):
        self._patrol_points = patrol_points or [
            {"x": 1.5, "y": 0.0},
            {"x": 0.0, "y": 0.0},
            {"x": 1.5, "y": 1.5},
            {"x": 0.0, "y": 1.5},
        ]
        self._use_llm = bool(use_llm)
        self._llm_service_url = str(llm_service_url)
        self._llm_timeout_s = float(llm_timeout_s)

        # 博弈状态
        self.roles = {}
        self.targets = {}
        self.current_tactic = 'balanced'
        self.focus_target = None

        # ---- 缓存敌方状态（最后已知位置） ----
        self._last_enemy_state = []

    def plan_tasks(self, battle_state):
        if self._use_llm:
            try:
                return self._llm_plan(battle_state)
            except Exception as e:
                rospy.logwarn("LLM planning failed, fallback to rule: %s", e)

        return self._rule_plan(battle_state)

    def _rule_plan(self, battle_state):
        team_state, enemy_state, robot_ids = self._extract_state(battle_state)

        if enemy_state:
            self._last_enemy_state = enemy_state

        if not robot_ids:
            rospy.logwarn("No robot_ids, returning empty")
            return {}

        team_state = [car for car in team_state if car.get('alive', True)]
        if not team_state:
            rospy.logwarn("All friendly cars are dead, returning empty")
            return {}

        robot_ids = [car['id'] for car in team_state]

        rospy.loginfo("=== _rule_plan called ===")
        rospy.loginfo("robot_ids: %s", robot_ids)
        rospy.loginfo("team_state: %s", team_state)
        rospy.loginfo("enemy_state: %s", enemy_state)
        rospy.loginfo("last_enemy_state: %s", self._last_enemy_state)

        # 计算存活数量（用于战术分支）
        alive_count = len(team_state)
        enemy_alive_count = len([e for e in enemy_state if e.get('visible', True)])

        rospy.loginfo("alive_count=%d, enemy_alive_count=%d", alive_count, enemy_alive_count)

        threats = self._assess_threats(team_state, enemy_state)
        primary_threat = threats[0] if threats else None
        low_hp_enemy = self._find_lowest_hp(enemy_state)

        self.roles = self._assign_roles(team_state, enemy_state)
        self.targets = self._select_targets(
            team_state, enemy_state, self.roles, primary_threat, low_hp_enemy
        )
        self.current_tactic = self._select_tactic(team_state, enemy_state)

        tasks = self._generate_tasks(team_state, enemy_state, alive_count, enemy_alive_count)

        rospy.loginfo("Final tasks: %s", tasks)
        return tasks

    def _extract_state(self, battle_state):
        friendly = battle_state.get('friendly', {})
        enemy = battle_state.get('enemy', {})

        team_state = []
        robot_ids = []

        for ns, data in friendly.items():
            if not isinstance(data, dict):
                continue
            state = data.get('state', {})
            if isinstance(state, dict):
                hp = self._to_float(state.get('hp', 100.0), 100.0)
                alive = bool(state.get('alive', True))
                if hp <= 0:
                    alive = False

                car_x, car_y, car_yaw = self._extract_pose(state)

                team_state.append({
                    'id': ns,
                    'x': car_x,
                    'y': car_y,
                    'yaw': car_yaw,
                    'hp': hp,
                    'ammo': self._to_float(state.get('ammo', 50.0), 50.0),
                    'alive': alive,
                    'in_combat': bool(state.get('in_combat', False))
                })
                robot_ids.append(ns)

        enemy_state = []
        enemy_block = enemy.get('state', {})
        if isinstance(enemy_block, dict):
            enemies = enemy_block.get('visible_enemies', [])
            if not enemies:
                enemies = enemy_block.get('enemies', [])
            for e in enemies:
                if isinstance(e, dict):
                    enemy_state.append({
                        'id': e.get('robot_ns', e.get('id', 'unknown')),
                        'x': self._to_float(e.get('x', 0.0), 0.0),
                        'y': self._to_float(e.get('y', 0.0), 0.0),
                        'hp': self._to_float(e.get('hp', 100.0), 100.0),
                        'visible': bool(e.get('visible', True))
                    })

        return team_state, enemy_state, robot_ids

    @staticmethod
    def _to_float(value, default=0.0):
        """把 ROS 消息里的数值安全地转成 float。"""
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    @staticmethod
    def _quaternion_to_yaw(q):
        """四元数（字典形式）转 yaw。"""
        try:
            qx = float(q.get('x', 0.0))
            qy = float(q.get('y', 0.0))
            qz = float(q.get('z', 0.0))
            qw = float(q.get('w', 1.0))
        except (TypeError, ValueError):
            return 0.0
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _extract_pose(self, state):
        """从 RobotState 字典中取出 (x, y, yaw)。

        GlobalObserver 会把 RobotState 递归转成字典，位姿位于
        state['pose']['position']['x'/'y']，而不是平铺的 x/y。
        这里同时兼容平铺写法，避免上游格式变化时再次退化到原点。
        """
        x = state.get('x')
        y = state.get('y')
        yaw = state.get('yaw')

        pose = state.get('pose')
        if isinstance(pose, dict):
            position = pose.get('position')
            if not isinstance(position, dict):
                position = pose
            if x is None:
                x = position.get('x')
            if y is None:
                y = position.get('y')
            orientation = pose.get('orientation')
            if yaw is None and isinstance(orientation, dict):
                yaw = self._quaternion_to_yaw(orientation)

        if x is None or y is None:
            position = state.get('position')
            if isinstance(position, dict):
                if x is None:
                    x = position.get('x')
                if y is None:
                    y = position.get('y')

        return (self._to_float(x, 0.0),
                self._to_float(y, 0.0),
                self._to_float(yaw, 0.0))

    def _assess_threats(self, team_state, enemy_state):
        if not enemy_state or not team_state:
            return []

        scored_enemies = []
        for enemy in enemy_state:
            if not enemy.get('visible', True):
                continue

            score = 0.0
            min_dist = min([self._distance(team, enemy) for team in team_state])
            if min_dist < 5.0:
                score += (5.0 - min_dist) / 5.0 * 80
            elif min_dist < 10.0:
                score += (10.0 - min_dist) / 10.0 * 40

            avg_hp = sum(t['hp'] for t in team_state) / len(team_state)
            if avg_hp < 80:
                score += 20

            enemy['threat_score'] = score
            scored_enemies.append(enemy)

        scored_enemies.sort(key=lambda e: e.get('threat_score', 0), reverse=True)
        return scored_enemies

    def _assign_roles(self, team_state, enemy_state):
        alive_cars = [t for t in team_state if t.get('alive', True)]
        if len(alive_cars) < 3:
            return {t['id']: 'Attack' for t in alive_cars}

        scores = {}
        for car in alive_cars:
            if enemy_state:
                min_dist = min([self._distance(car, e) for e in enemy_state])
                dist_score = max(0, (10 - min_dist) / 10 * 100)
            else:
                dist_score = 50

            angle_score = self._calc_angle_score(car, enemy_state)
            hp_score = car.get('hp', 0) / 100 * 100
            scores[car['id']] = dist_score * 0.8 + angle_score * 0.1 + hp_score * 0.1

        sorted_cars = sorted(scores.items(), key=lambda x: x[1], reverse=True)

        roles = {
            sorted_cars[0][0]: 'Attack',
            sorted_cars[1][0]: 'Support-1',
            sorted_cars[2][0]: 'Support-2'
        }
        return roles

    def _calc_angle_score(self, car, enemies):
        if not enemies:
            return 50

        nearest = min(enemies, key=lambda e: self._distance(car, e))
        car_yaw = car.get('yaw', 0)
        dx = nearest['x'] - car['x']
        dy = nearest['y'] - car['y']
        angle_to_enemy = math.atan2(dy, dx)

        diff = car_yaw - angle_to_enemy
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        diff = abs(diff)

        return (math.pi - diff) / math.pi * 100

    def _select_targets(self, team_state, enemy_state, roles, primary_threat, low_hp_enemy):
        targets = {}

        if not enemy_state:
            return targets

        if not primary_threat:
            primary_threat = enemy_state[0] if enemy_state else None

        for car_id, role in roles.items():
            if role == 'Attack':
                targets[car_id] = primary_threat['id'] if primary_threat else None
            elif role == 'Support-1':
                targets[car_id] = primary_threat['id'] if primary_threat else None
            else:
                if low_hp_enemy and low_hp_enemy['id'] != (primary_threat['id'] if primary_threat else None):
                    targets[car_id] = low_hp_enemy['id']
                elif primary_threat:
                    targets[car_id] = primary_threat['id']
                else:
                    targets[car_id] = None

        self.focus_target = primary_threat['id'] if primary_threat else None
        return targets

    def _find_lowest_hp(self, enemy_state):
        if not enemy_state:
            return None
        alive_enemies = [e for e in enemy_state if e.get('visible', True)]
        if not alive_enemies:
            return None
        return min(alive_enemies, key=lambda e: e.get('hp', 100))

    def _select_tactic(self, team_state, enemy_state):
        team_hp = sum(t.get('hp', 0) for t in team_state)
        enemy_hp = sum(e.get('hp', 0) for e in enemy_state if e.get('visible', True))

        if enemy_hp == 0:
            return 'balanced'

        ratio = team_hp / enemy_hp
        if ratio > 1.3:
            return 'aggressive'
        elif ratio < 0.7:
            return 'defensive'
        else:
            return 'balanced'

    # ================================================================
    # 核心修改：_generate_tasks 和 _get_formation_positions
    # ================================================================

    def _generate_tasks(self, team_state, enemy_state, alive_count, enemy_alive_count):
        """生成任务字典（含击杀后动态响应）"""
        tasks = {}

        # ---- 根据人数对比决定战术模式 ----
        if alive_count > enemy_alive_count:
            tactic_mode = "pursuit"      # 人数优势：追击
            rospy.loginfo("[Tactic] 人数优势 (%d vs %d) → 追击模式", alive_count, enemy_alive_count)
        elif alive_count < enemy_alive_count:
            tactic_mode = "attack_and_retreat"  # 人数劣势：一攻一守
            rospy.loginfo("[Tactic] 人数劣势 (%d vs %d) → 攻守模式", alive_count, enemy_alive_count)
        else:
            tactic_mode = "balanced"
            rospy.loginfo("[Tactic] 人数持平 (%d vs %d) → 平衡模式", alive_count, enemy_alive_count)

        # 计算队形/追击位置（在追击模式下，formation_positions 是最后已知位置）
        formation_positions = self._get_formation_positions(
            team_state, enemy_state, tactic_mode
        )

        # 获取角色列表（用于攻守模式分配）
        role_list = list(self.roles.items())
        attack_cars = [cid for cid, role in role_list if role == 'Attack']
        support_cars = [cid for cid, role in role_list if role in ('Support-1', 'Support-2')]

        for car in team_state:
            car_id = car['id']
            role = self.roles.get(car_id, 'Support-2')
            target_id = self.targets.get(car_id)

            hp = car.get('hp', 100)

            # ---- 1. 低血量撤退（最高优先级） ----
            if hp < 20:
                if enemy_state:
                    nearest_enemy = min(enemy_state, key=lambda e: self._distance(car, e))
                    retreat_x = car['x'] - (nearest_enemy['x'] - car['x']) * 2.0
                    retreat_y = car['y'] - (nearest_enemy['y'] - car['y']) * 2.0
                    retreat_x = max(-15, min(15, retreat_x))
                    retreat_y = max(-15, min(15, retreat_y))
                else:
                    retreat_x, retreat_y = 0.0, 0.0

                tasks[car_id] = self._build_task(
                    action='RETREAT',
                    target={'x': retreat_x, 'y': retreat_y},
                    mode=0,
                    reason='low hp ({:.0f}) retreat'.format(hp),
                    timeout=12.0
                )
                continue

            # ---- 2. 攻守模式（人数劣势）特殊处理 ----
            if tactic_mode == "attack_and_retreat":
                if car_id in attack_cars:
                    # Attack 车 → 向前攻击（使用原本的目标或追击位置）
                    if target_id:
                        target = self._find_enemy_by_id(enemy_state, target_id)
                        if target:
                            tasks[car_id] = self._build_task(
                                action='ATTACK',
                                target={'x': target['x'], 'y': target['y']},
                                mode=2,
                                reason='{} attack forward (劣势反击)'.format(role),
                                timeout=4.0
                            )
                            continue
                    elif enemy_state:
                        # 如果没有指定目标，攻击最近的敌人
                        nearest = min(enemy_state, key=lambda e: self._distance(car, e))
                        tasks[car_id] = self._build_task(
                            action='ATTACK',
                            target={'x': nearest['x'], 'y': nearest['y']},
                            mode=2,
                            reason='{} attack nearest (劣势反击)'.format(role),
                            timeout=4.0
                        )
                        continue

                elif car_id in support_cars:
                    # Support 车 → 撤退掩护
                    if enemy_state:
                        nearest_enemy = min(enemy_state, key=lambda e: self._distance(car, e))
                        retreat_x = car['x'] - (nearest_enemy['x'] - car['x']) * 2.0
                        retreat_y = car['y'] - (nearest_enemy['y'] - car['y']) * 2.0
                    else:
                        retreat_x, retreat_y = 0.0, 0.0
                    tasks[car_id] = self._build_task(
                        action='RETREAT',
                        target={'x': retreat_x, 'y': retreat_y},
                        mode=0,
                        reason='{} retreat (掩护)'.format(role),
                        timeout=12.0
                    )
                    continue

            # ---- 3. 追击模式（人数优势） ----
            if tactic_mode == "pursuit":
                # ---- 关键修改：如果有可见敌人，直接攻击 ----
                if enemy_state:
                    # 攻击最近的敌人
                    nearest = min(enemy_state, key=lambda e: self._distance(car, e))
                    tasks[car_id] = self._build_task(
                        action='ATTACK',
                        target={'x': nearest['x'], 'y': nearest['y']},
                        mode=2,
                        reason='pursuit attack (人数优势)',
                        timeout=3.0
                    )
                else:
                    # 没有可见敌人 → 前往最后已知位置（formation_positions 已计算）
                    pos = formation_positions.get(car_id, {'x': car['x'], 'y': car['y']})
                    tasks[car_id] = self._build_task(
                        action='GOTO',
                        target=pos,
                        mode=1,
                        reason='pursuit (追击位置)',
                        timeout=20.0
                    )
                continue

            # ---- 4. 正常模式（有目标 → 攻击，无目标 → 队形移动） ----
            if target_id:
                target = self._find_enemy_by_id(enemy_state, target_id)
                if target:
                    tasks[car_id] = self._build_task(
                        action='ATTACK',
                        target={'x': target['x'], 'y': target['y']},
                        mode=2,
                        reason='{} targeting {}'.format(role, target_id),
                        timeout=4.0
                    )
                    continue

            pos = formation_positions.get(car_id, {'x': car['x'], 'y': car['y']})
            tasks[car_id] = self._build_task(
                action='GOTO',
                target=pos,
                mode=1,
                reason='formation move ({})'.format(role),
                timeout=20.0
            )

        return tasks

    def _get_formation_positions(self, team_state, enemy_state, tactic_mode):
        """
        计算队形/追击位置
        - pursuit: 朝敌人方向推进（若无可见敌人则使用最后已知位置）
        - attack_and_retreat: 由 _generate_tasks 特殊处理，这里返回空
        - balanced: 可见敌人用三角队形，不可见则追击/侦察
        """
        if not team_state:
            return {}

        # ---- 追击模式：向前推进（朝敌人方向） ----
        if tactic_mode == "pursuit":
            # 用最后已知敌人位置作为目标
            target_x, target_y = 0.0, 0.0
            if self._last_enemy_state:
                target_x = sum(e.get('x', 0.0) for e in self._last_enemy_state) / len(self._last_enemy_state)
                target_y = sum(e.get('y', 0.0) for e in self._last_enemy_state) / len(self._last_enemy_state)
                rospy.loginfo("[Pursuit] 追击目标: (%.1f, %.1f)", target_x, target_y)
            else:
                rospy.loginfo("[Pursuit] 无历史位置，朝地图中心推进")

            positions = {}
            car_ids = sorted([t['id'] for t in team_state])
            for car_id in car_ids:
                positions[car_id] = {'x': target_x, 'y': target_y}
            return positions

        # ---- 攻守模式：由 _generate_tasks 特殊处理，这里不返回位置 ----
        if tactic_mode == "attack_and_retreat":
            return {}

        # ---- 平衡模式：原有逻辑 ----
        if enemy_state:
            center_x = sum(t['x'] for t in team_state) / len(team_state)
            center_y = sum(t['y'] for t in team_state) / len(team_state)

            offset_scale = 1.5 if self.current_tactic == 'aggressive' else (-0.8 if self.current_tactic == 'defensive' else 0.0)

            positions = {}
            car_ids = sorted([t['id'] for t in team_state])
            for i, car_id in enumerate(car_ids):
                angle = i * 2.0 * math.pi / 3.0
                positions[car_id] = {
                    'x': center_x + math.cos(angle) * 6.0,
                    'y': center_y + math.sin(angle) * 6.0
                }
            return positions

        # 没有可见敌人 → 追击/侦察
        if self._last_enemy_state:
            return self._get_pursuit_positions(team_state)
        return self._get_scout_positions(team_state)

    def _get_pursuit_positions(self, team_state):
        """追击模式：所有存活小车前往最后已知的敌人平均位置"""
        if not team_state or not self._last_enemy_state:
            return {}

        avg_x = sum(e.get('x', 0.0) for e in self._last_enemy_state) / len(self._last_enemy_state)
        avg_y = sum(e.get('y', 0.0) for e in self._last_enemy_state) / len(self._last_enemy_state)

        rospy.loginfo("[Pursuit] 追击目标位置: (%.1f, %.1f)", avg_x, avg_y)

        positions = {}
        car_ids = sorted([t['id'] for t in team_state])
        for car_id in car_ids:
            positions[car_id] = {'x': avg_x, 'y': avg_y}

        return positions

    def _get_scout_positions(self, team_state):
        """分散侦察点（必须落在场地内，world0.world 围墙为 x=±4.05 / y=±2.0）。"""
        scout_points = [
            {'x': 3.2, 'y': 1.2},
            {'x': -3.2, 'y': 1.2},
            {'x': 0.0, 'y': -1.5},
        ]

        positions = {}
        car_ids = sorted([t['id'] for t in team_state])
        for i, car_id in enumerate(car_ids):
            point = scout_points[i % len(scout_points)]
            positions[car_id] = {'x': point['x'], 'y': point['y']}
            rospy.loginfo("[Scout] %s → (%.1f, %.1f)", car_id, point['x'], point['y'])

        return positions

    def _distance(self, a, b):
        dx = a.get('x', 0) - b.get('x', 0)
        dy = a.get('y', 0) - b.get('y', 0)
        return math.sqrt(dx*dx + dy*dy)

    def _find_enemy_by_id(self, enemy_state, target_id):
        for e in enemy_state:
            if e.get('id') == target_id:
                return e
        return None

    def _build_task(self, action, target, mode, reason, timeout):
        return {
            'action': action,
            'target': {
                'x': float(target.get('x', 0)),
                'y': float(target.get('y', 0)),
                'yaw': float(target.get('yaw', 0))
            },
            'mode': int(mode),
            'reason': str(reason),
            'timeout': float(timeout)
        }

    def _llm_plan(self, battle_state):
        friendly = battle_state.get('friendly', {})
        robot_ids = [ns for ns in friendly.keys()]

        if self._llm_service_url:
            try:
                response = requests.post(
                    self._llm_service_url,
                    json={'battle_state': battle_state, 'robot_ids': robot_ids},
                    timeout=self._llm_timeout_s
                )
                response.raise_for_status()
                result = response.json()
                if isinstance(result, dict):
                    return self._normalize_llm_tasks(result, robot_ids)
            except Exception as e:
                rospy.logwarn("LLM request failed: %s", e)

        return self._rule_plan(battle_state)

    def _normalize_llm_tasks(self, tasks, robot_ids):
        result = {}
        for rid in robot_ids:
            raw = tasks.get(rid)
            if raw:
                result[rid] = {
                    'action': raw.get('action', 'STOP'),
                    'target': raw.get('target', {'x': 0, 'y': 0}),
                    'mode': raw.get('mode', 0),
                    'reason': raw.get('reason', 'llm decision'),
                    'timeout': raw.get('timeout', 6.0)
                }
            else:
                result[rid] = self._build_task(
                    action='STOP',
                    target={'x': 0, 'y': 0},
                    mode=0,
                    reason='missing llm task',
                    timeout=2.0
                )
        return result
