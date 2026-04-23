from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple


Vec2 = Tuple[int, int]


@dataclass(frozen=True)
class Pose2D:
    """简化的 2D 位姿（x, y, yaw）。"""

    x: int
    y: int
    yaw: float = 0.0


@dataclass(frozen=True)
class Detection:
    """视觉模块输出的目标检测结果。"""

    name: str
    pose: Pose2D
    confidence: float


@dataclass(frozen=True)
class GridWorld:
    """离散化工作空间。"""

    width: int
    height: int
    static_obstacles: List[Vec2]


@dataclass(frozen=True)
class PlanResult:
    """规划器输出。"""

    success: bool
    waypoints: List[Vec2]
    reason: str = ""
