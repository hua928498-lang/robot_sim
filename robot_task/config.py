from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple


Vec2 = Tuple[int, int]


@dataclass
class DynamicsConfig:
    """动力学近似约束（用于轨迹执行阶段的可行性检查）。"""

    max_speed: float = 2.0
    max_acc: float = 3.0
    max_payload: float = 0.2
    gripper_force: float = 30.0


@dataclass
class TaskConfig:
    """任务参数配置。"""

    world_width: int = 20
    world_height: int = 12
    robot_start: Vec2 = (1, 1)
    pen_goal: Vec2 = (8, 4)
    holder_goal: Vec2 = (17, 9)
    static_obstacles: List[Vec2] = field(
        default_factory=lambda: [(5, 5), (6, 5), (7, 5), (10, 7), (11, 7)]
    )
    dynamic_obstacles_timeline: List[List[Vec2]] = field(
        default_factory=lambda: [[], [(9, 4)], [(12, 7)], []]
    )
    dynamics: DynamicsConfig = field(default_factory=DynamicsConfig)
