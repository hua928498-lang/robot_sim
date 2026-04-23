from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

from .types import Detection, Pose2D


Vec2 = Tuple[int, int]


@dataclass
class PerceptionState:
    """模拟视觉状态：可配置为已知位姿或未知位姿（由检测输出）。"""

    objects: Dict[str, Vec2]


class CameraPerception:
    """视觉模块（示例实现）。

    在接入 SAPIEN / ManiSkill 后，可在 `detect` 中读取相机图像并调用检测模型。
    """

    def __init__(self, state: PerceptionState):
        self.state = state

    def detect(self, names: List[str]) -> List[Detection]:
        detections: List[Detection] = []
        for name in names:
            if name not in self.state.objects:
                continue
            x, y = self.state.objects[name]
            detections.append(
                Detection(name=name, pose=Pose2D(x=x, y=y, yaw=0.0), confidence=0.98)
            )
        return detections
