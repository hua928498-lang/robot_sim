from __future__ import annotations

"""基于 mplib 的抓取-放置任务执行器。

设计目标：
1. 直接对接你已安装的 sapien 3.0 + mplib + maniskill 技术栈。
2. 提供“已知位姿 -> 未知位姿”的统一代码框架。
3. 支持静态障碍 + 动态障碍触发重规划。

注意：不同版本的 mplib API 在参数名上可能有差异，
本文件将核心调用集中在 `MplibPlannerAdapter` 里，若你的本地版本参数不同，
只需改该类中的少量代码即可。
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence


@dataclass
class Pose3D:
    p: Sequence[float]  # [x, y, z]
    q: Sequence[float]  # [w, x, y, z]


@dataclass
class BoxObstacle:
    center: Sequence[float]  # [x, y, z]
    half_size: Sequence[float]  # [hx, hy, hz]


@dataclass
class RobotModelConfig:
    urdf_path: str
    srdf_path: str
    move_group: str
    user_link_names: List[str]
    user_joint_names: List[str]


@dataclass
class PickPlaceConfig3D:
    robot: RobotModelConfig
    start_qpos: List[float]
    pick_pose: Pose3D
    place_pose: Pose3D
    grasp_approach_offset: float = 0.10
    place_approach_offset: float = 0.10
    static_obstacles: List[BoxObstacle] = field(default_factory=list)
    dynamic_obstacles_timeline: List[List[BoxObstacle]] = field(default_factory=list)


class MplibPlannerAdapter:
    """对 mplib 进行薄封装，便于版本适配。"""

    def __init__(self, model_cfg: RobotModelConfig):
        try:
            import mplib  # type: ignore
        except Exception as exc:  # pragma: no cover - runtime dependency
            raise RuntimeError(
                "未检测到 mplib，请先确认环境可 `import mplib`。"
            ) from exc

        self._mplib = mplib
        self._planner = mplib.Planner(
            urdf=model_cfg.urdf_path,
            srdf=model_cfg.srdf_path,
            user_link_names=model_cfg.user_link_names,
            user_joint_names=model_cfg.user_joint_names,
            move_group=model_cfg.move_group,
        )

    def update_obstacles(self, obstacles: List[BoxObstacle]) -> None:
        """把当前障碍物更新给规划器。"""
        # 兼容常见 API：update_point_cloud / update_collision_objects
        if hasattr(self._planner, "remove_point_cloud"):
            try:
                self._planner.remove_point_cloud()
            except Exception:
                pass

        if hasattr(self._planner, "update_collision_objects"):
            collision_objects: List[Dict[str, Any]] = []
            for obs in obstacles:
                collision_objects.append(
                    {
                        "type": "box",
                        "pose": list(obs.center) + [1.0, 0.0, 0.0, 0.0],
                        "size": [2 * obs.half_size[0], 2 * obs.half_size[1], 2 * obs.half_size[2]],
                    }
                )
            self._planner.update_collision_objects(collision_objects)
            return

        if hasattr(self._planner, "update_point_cloud"):
            # 将障碍盒中心近似为稀疏点云（简单版本，便于快速落地）
            points: List[List[float]] = [list(obs.center) for obs in obstacles]
            if points:
                self._planner.update_point_cloud(points, resolution=0.02)

    def plan_to_pose(self, target: Pose3D, start_qpos: Sequence[float]) -> Dict[str, Any]:
        """规划到目标末端位姿。"""
        # 常见 mplib API: plan_pose(goal_pose, start_qpos, time_step=...)
        goal_pose = self._mplib.Pose(target.p, target.q)
        if hasattr(self._planner, "plan_pose"):
            return self._planner.plan_pose(goal_pose, start_qpos, time_step=1 / 250)

        raise RuntimeError("当前 mplib 版本缺少 plan_pose，请在 MplibPlannerAdapter.plan_to_pose 中适配。")


class PickPlaceRunner3D:
    """抓取放置任务执行器（mplib 版）。"""

    def __init__(self, cfg: PickPlaceConfig3D):
        self.cfg = cfg
        self.planner = MplibPlannerAdapter(cfg.robot)

    @staticmethod
    def _with_offset(pose: Pose3D, dz: float) -> Pose3D:
        p = [pose.p[0], pose.p[1], pose.p[2] + dz]
        return Pose3D(p=p, q=pose.q)

    def run(self) -> List[str]:
        logs: List[str] = []

        # 1) 抓取前接近位姿
        qpos = list(self.cfg.start_qpos)
        pick_approach = self._with_offset(self.cfg.pick_pose, self.cfg.grasp_approach_offset)

        self.planner.update_obstacles(self.cfg.static_obstacles)
        r1 = self.planner.plan_to_pose(pick_approach, qpos)
        if r1.get("status") not in ("Success", "success", 0):
            return [f"pick_approach 规划失败: {r1}"]
        logs.append("pick_approach 规划成功")

        # 2) 下探抓取位姿
        r2 = self.planner.plan_to_pose(self.cfg.pick_pose, qpos)
        if r2.get("status") not in ("Success", "success", 0):
            return logs + [f"pick_pose 规划失败: {r2}"]
        logs.append("pick_pose 规划成功")

        # 3) 放置前接近位姿，期间允许动态障碍触发重规划
        place_approach = self._with_offset(self.cfg.place_pose, self.cfg.place_approach_offset)
        all_dynamic = self.cfg.dynamic_obstacles_timeline or [[]]
        success = False
        for tick, dyn_obs in enumerate(all_dynamic):
            self.planner.update_obstacles(self.cfg.static_obstacles + dyn_obs)
            r3 = self.planner.plan_to_pose(place_approach, qpos)
            if r3.get("status") in ("Success", "success", 0):
                logs.append(f"place_approach 在 tick={tick} 规划成功")
                success = True
                break
            logs.append(f"place_approach 在 tick={tick} 失败，触发重规划")

        if not success:
            return logs + ["place_approach 最终规划失败"]

        # 4) 下探放置位姿
        r4 = self.planner.plan_to_pose(self.cfg.place_pose, qpos)
        if r4.get("status") not in ("Success", "success", 0):
            return logs + [f"place_pose 规划失败: {r4}"]

        logs.append("place_pose 规划成功")
        logs.append("任务完成：抓笔并放入笔筒")
        return logs
