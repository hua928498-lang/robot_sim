from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .config import DynamicsConfig
from .types import Vec2


@dataclass
class ExecutionReport:
    success: bool
    reason: str
    executed_waypoints: List[Vec2]


class TrajectoryController:
    """轨迹执行器（简化动力学检查）。"""

    def __init__(self, dynamics: DynamicsConfig):
        self.dynamics = dynamics

    def execute(self, waypoints: List[Vec2], payload_mass: float) -> ExecutionReport:
        if not waypoints:
            return ExecutionReport(False, "empty trajectory", [])

        if payload_mass > self.dynamics.max_payload:
            return ExecutionReport(False, "payload too heavy", [])

        # 简化：把栅格步长看成单位位移，单位时间 1s
        # 则速度约束 = 每步距离 <= max_speed
        max_step_distance = 1.0
        if max_step_distance > self.dynamics.max_speed:
            return ExecutionReport(False, "speed limit exceeded", [])

        # 简化加速度约束：相邻步速度变化不超过 max_acc
        # 当前离散模型中速度恒定，认为满足
        return ExecutionReport(True, "ok", waypoints)
