from __future__ import annotations

from dataclasses import dataclass
from typing import List, Set, Tuple

from .config import TaskConfig
from .controller import ExecutionReport, TrajectoryController
from .perception import CameraPerception, PerceptionState
from .planner import AStarPlanner
from .types import Detection, Vec2


@dataclass
class StageResult:
    name: str
    success: bool
    details: str


class PickPlaceTaskManager:
    """将视觉、规划、控制串成完整任务流程。"""

    def __init__(self, config: TaskConfig):
        self.cfg = config
        self.perception = CameraPerception(
            PerceptionState(objects={"pen": config.pen_goal, "holder": config.holder_goal})
        )
        self.planner = AStarPlanner(width=config.world_width, height=config.world_height)
        self.controller = TrajectoryController(config.dynamics)

    def run_known_pose_stage(self) -> List[StageResult]:
        """已知位姿阶段：直接使用配置中的笔和笔筒位置。"""
        return self._run_pipeline(pen=self.cfg.pen_goal, holder=self.cfg.holder_goal)

    def run_unknown_pose_stage(self) -> List[StageResult]:
        """未知位姿阶段：通过视觉检测目标位置。"""
        detections = self.perception.detect(["pen", "holder"])
        pos = self._extract_positions(detections)
        if "pen" not in pos or "holder" not in pos:
            return [StageResult("perception", False, "missing pen or holder detection")]
        return self._run_pipeline(pen=pos["pen"], holder=pos["holder"])

    def _run_pipeline(self, pen: Vec2, holder: Vec2) -> List[StageResult]:
        results: List[StageResult] = []

        pick_results = self._plan_and_execute(
            stage_name="pick", start=self.cfg.robot_start, goal=pen, payload_mass=0.0
        )
        results.extend(pick_results)
        if not pick_results[-1].success:
            return results

        place_results = self._plan_and_execute(
            stage_name="place", start=pen, goal=holder, payload_mass=0.02
        )
        results.extend(place_results)
        return results

    def _plan_and_execute(
        self, stage_name: str, start: Vec2, goal: Vec2, payload_mass: float
    ) -> List[StageResult]:
        stage_results: List[StageResult] = []
        base_obstacles: Set[Vec2] = set(self.cfg.static_obstacles)

        # 第一次规划
        plan = self.planner.plan(start=start, goal=goal, obstacles=base_obstacles)
        if not plan.success:
            stage_results.append(StageResult(f"{stage_name}.plan", False, plan.reason))
            return stage_results
        stage_results.append(
            StageResult(f"{stage_name}.plan", True, f"waypoints={len(plan.waypoints)}")
        )

        # 执行中考虑动态障碍，触发重规划
        current_path = plan.waypoints
        current_start = start
        for tick, dynamic_obs in enumerate(self.cfg.dynamic_obstacles_timeline):
            combined = base_obstacles | set(dynamic_obs)
            blocked_index = self._first_blocked_idx(current_path, combined)
            if blocked_index is not None:
                current_start = current_path[max(blocked_index - 1, 0)]
                replan = self.planner.plan(start=current_start, goal=goal, obstacles=combined)
                if not replan.success:
                    stage_results.append(
                        StageResult(f"{stage_name}.replan@{tick}", False, replan.reason)
                    )
                    return stage_results
                current_path = replan.waypoints
                stage_results.append(
                    StageResult(
                        f"{stage_name}.replan@{tick}",
                        True,
                        f"new_waypoints={len(current_path)}",
                    )
                )

        report: ExecutionReport = self.controller.execute(
            waypoints=current_path, payload_mass=payload_mass
        )
        stage_results.append(StageResult(f"{stage_name}.execute", report.success, report.reason))
        return stage_results

    @staticmethod
    def _extract_positions(detections: List[Detection]) -> dict[str, Vec2]:
        return {det.name: (det.pose.x, det.pose.y) for det in detections}

    @staticmethod
    def _first_blocked_idx(path: List[Vec2], obstacles: Set[Vec2]) -> int | None:
        for idx, p in enumerate(path):
            if p in obstacles:
                return idx
        return None
