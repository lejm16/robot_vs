#!/usr/bin/env python
# -*- coding: utf-8 -*-


class BattleStateFormatter(object):
    """Converts raw battle state to planner input."""

    def build(self, state, team_color, my_cars):
        if state is None:
            state = {}

        return {
            "team_color": str(team_color),
            "my_cars": list(my_cars),
            "battle_state": state,
        }
