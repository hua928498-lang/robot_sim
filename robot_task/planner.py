from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from .types import PlanResult, Vec2


@dataclass
class AStarPlanner:
    """2D 栅格 A* 规划器，模拟机械臂末端在平面上的避障路径。"""

    width: int
    height: int

    def plan(self, start: Vec2, goal: Vec2, obstacles: Set[Vec2]) -> PlanResult:
        if start in obstacles:
            return PlanResult(success=False, waypoints=[], reason="start blocked")
        if goal in obstacles:
            return PlanResult(success=False, waypoints=[], reason="goal blocked")

        open_set: List[Tuple[int, Vec2]] = []
        heapq.heappush(open_set, (0, start))

        came_from: Dict[Vec2, Optional[Vec2]] = {start: None}
        g_score: Dict[Vec2, int] = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return PlanResult(success=True, waypoints=self._reconstruct(came_from, goal))

            for nxt in self._neighbors(current):
                if nxt in obstacles:
                    continue
                tentative = g_score[current] + 1
                if nxt not in g_score or tentative < g_score[nxt]:
                    g_score[nxt] = tentative
                    priority = tentative + self._manhattan(nxt, goal)
                    heapq.heappush(open_set, (priority, nxt))
                    came_from[nxt] = current

        return PlanResult(success=False, waypoints=[], reason="no path")

    def _neighbors(self, node: Vec2) -> List[Vec2]:
        x, y = node
        candidates = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        return [n for n in candidates if 0 <= n[0] < self.width and 0 <= n[1] < self.height]

    @staticmethod
    def _manhattan(a: Vec2, b: Vec2) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def _reconstruct(came_from: Dict[Vec2, Optional[Vec2]], goal: Vec2) -> List[Vec2]:
        path = [goal]
        cur = goal
        while came_from[cur] is not None:
            cur = came_from[cur]  # type: ignore[assignment]
            path.append(cur)
        path.reverse()
        return path
