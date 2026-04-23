from __future__ import annotations

from .config import TaskConfig
from .task_manager import PickPlaceTaskManager


def print_stage(title: str, results):
    print(f"\n=== {title} ===")
    for r in results:
        mark = "✅" if r.success else "❌"
        print(f"{mark} {r.name}: {r.details}")


def run_demo() -> None:
    cfg = TaskConfig()
    manager = PickPlaceTaskManager(cfg)

    known = manager.run_known_pose_stage()
    print_stage("Known Pose Stage", known)

    unknown = manager.run_unknown_pose_stage()
    print_stage("Unknown Pose Stage", unknown)


if __name__ == "__main__":
    run_demo()
