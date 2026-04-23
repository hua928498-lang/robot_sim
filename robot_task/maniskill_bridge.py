from __future__ import annotations

"""ManiSkill 桥接层。

作用：
- 提供环境创建函数。
- 提供从观测中提取笔和笔筒位姿的统一入口（你可按自己的任务 obs 结构修改）。
"""

from dataclasses import dataclass
from typing import Any, Dict, Tuple


@dataclass
class DetectedTargets:
    pen_xyz: Tuple[float, float, float]
    holder_xyz: Tuple[float, float, float]


def make_env(env_id: str, obs_mode: str = "rgbd", control_mode: str = "pd_joint_pos"):
    try:
        import gymnasium as gym  # type: ignore
        import mani_skill  # noqa: F401  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("未检测到 gymnasium/mani_skill，请先安装后再运行。") from exc

    return gym.make(env_id, obs_mode=obs_mode, control_mode=control_mode)


def parse_targets_from_obs(obs: Dict[str, Any]) -> DetectedTargets:
    """从 ManiSkill 观测中提取目标位姿。

    默认读取：
    - obs['extra']['pen_pose'][:3]
    - obs['extra']['holder_pose'][:3]

    如果你的任务字段不同，只需要改这个函数。
    """
    extra = obs.get("extra", {})
    pen_pose = extra.get("pen_pose")
    holder_pose = extra.get("holder_pose")
    if pen_pose is None or holder_pose is None:
        raise KeyError(
            "观测中未找到 extra.pen_pose / extra.holder_pose。请根据你的 ManiSkill 任务修改 parse_targets_from_obs。"
        )

    pen_xyz = (float(pen_pose[0]), float(pen_pose[1]), float(pen_pose[2]))
    holder_xyz = (float(holder_pose[0]), float(holder_pose[1]), float(holder_pose[2]))
    return DetectedTargets(pen_xyz=pen_xyz, holder_xyz=holder_xyz)
