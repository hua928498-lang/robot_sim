from __future__ import annotations

import argparse
from pathlib import Path

from .config import TaskConfig
from .mplib_task import (
    BoxObstacle,
    PickPlaceConfig3D,
    PickPlaceRunner3D,
    Pose3D,
    RobotModelConfig,
)
from .task_manager import PickPlaceTaskManager


def run_grid_demo() -> None:
    cfg = TaskConfig()
    manager = PickPlaceTaskManager(cfg)

    print("\n=== Known Pose Stage (grid demo) ===")
    for r in manager.run_known_pose_stage():
        mark = "✅" if r.success else "❌"
        print(f"{mark} {r.name}: {r.details}")

    print("\n=== Unknown Pose Stage (grid demo) ===")
    for r in manager.run_unknown_pose_stage():
        mark = "✅" if r.success else "❌"
        print(f"{mark} {r.name}: {r.details}")


def run_mplib_demo(args: argparse.Namespace) -> None:
    robot_cfg = RobotModelConfig(
        urdf_path=args.urdf,
        srdf_path=args.srdf,
        move_group=args.move_group,
        user_link_names=args.user_links.split(","),
        user_joint_names=args.user_joints.split(","),
    )

    cfg = PickPlaceConfig3D(
        robot=robot_cfg,
        start_qpos=[float(x) for x in args.start_qpos.split(",")],
        pick_pose=Pose3D(
            p=[float(x) for x in args.pick_xyz.split(",")],
            q=[float(x) for x in args.pick_quat.split(",")],
        ),
        place_pose=Pose3D(
            p=[float(x) for x in args.place_xyz.split(",")],
            q=[float(x) for x in args.place_quat.split(",")],
        ),
        static_obstacles=[BoxObstacle(center=[0.45, 0.0, 0.2], half_size=[0.08, 0.08, 0.2])],
        dynamic_obstacles_timeline=[[], [BoxObstacle(center=[0.55, 0.0, 0.3], half_size=[0.06, 0.06, 0.2])], []],
    )

    logs = PickPlaceRunner3D(cfg).run()
    print("\n=== mplib pick-place run ===")
    for line in logs:
        print(f"- {line}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot pick-place task runner")
    sub = parser.add_subparsers(dest="mode", required=True)

    sub.add_parser("grid_demo", help="运行不依赖外部库的离散 demo")

    mp = sub.add_parser("mplib_demo", help="运行 mplib 版本抓取放置流程")
    mp.add_argument("--urdf", type=str, required=True)
    mp.add_argument("--srdf", type=str, required=True)
    mp.add_argument("--move-group", type=str, default="panda_hand")
    mp.add_argument("--user-links", type=str, required=True)
    mp.add_argument("--user-joints", type=str, required=True)
    mp.add_argument("--start-qpos", type=str, required=True)
    mp.add_argument("--pick-xyz", type=str, default="0.45,0.0,0.12")
    mp.add_argument("--pick-quat", type=str, default="1,0,0,0")
    mp.add_argument("--place-xyz", type=str, default="0.62,-0.15,0.14")
    mp.add_argument("--place-quat", type=str, default="1,0,0,0")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.mode == "grid_demo":
        run_grid_demo()
    elif args.mode == "mplib_demo":
        for p in [args.urdf, args.srdf]:
            if not Path(p).exists():
                raise FileNotFoundError(f"模型文件不存在: {p}")
        run_mplib_demo(args)


if __name__ == "__main__":
    main()
