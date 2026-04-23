# robot_sim：抓笔放入笔筒任务代码

这个版本提供两套运行路径：

1. `grid_demo`：纯 Python 离散仿真（无需外部依赖），用于验证任务流程逻辑。  
2. `mplib_demo`：对接 `mplib + SAPIEN/ManiSkill` 的真实路径规划骨架，可直接接你的机器人模型文件。

---

## 代码结构

- `robot_task/task_manager.py`：离散场景下的感知-规划-重规划-执行流程。
- `robot_task/mplib_task.py`：`mplib` 抓取放置执行器（3D 位姿 + 动态障碍重规划）。
- `robot_task/maniskill_bridge.py`：ManiSkill 环境桥接（建环境、从 obs 提取笔/笔筒位姿）。
- `robot_task/main.py`：命令行入口（`grid_demo` / `mplib_demo`）。

---

## 运行步骤（你当前环境：sapien 3.0 + mplib + maniskill）

## 1) 先验证基础流程

```bash
python -m robot_task.main grid_demo
```

如果成功，你会看到 pick/place 的规划与执行日志。

## 2) 准备机器人模型参数（URDF/SRDF）

你需要提供：

- `--urdf`：机器人 urdf 路径
- `--srdf`：机器人 srdf 路径
- `--user-links`：规划涉及 link 名（逗号分隔）
- `--user-joints`：规划涉及关节名（逗号分隔）
- `--start-qpos`：初始关节角（逗号分隔）

## 3) 运行 mplib 抓取放置

```bash
python -m robot_task.main mplib_demo \
  --urdf /path/to/robot.urdf \
  --srdf /path/to/robot.srdf \
  --move-group panda_hand \
  --user-links panda_link0,panda_link1,panda_link2,panda_link3,panda_link4,panda_link5,panda_link6,panda_link7,panda_hand \
  --user-joints panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7 \
  --start-qpos 0,-0.5,0,-2.2,0,1.7,0.8 \
  --pick-xyz 0.45,0.00,0.12 \
  --pick-quat 1,0,0,0 \
  --place-xyz 0.62,-0.15,0.14 \
  --place-quat 1,0,0,0
```

---

## 如何接入未知位置笔/笔筒（ManiSkill 相机观测）

1. 在你的 ManiSkill 任务里开启相机观测（`obs_mode=rgbd`）。
2. 在 `robot_task/maniskill_bridge.py` 的 `parse_targets_from_obs` 中改成你任务真实字段。  
   目前默认读取：`obs['extra']['pen_pose']`、`obs['extra']['holder_pose']`。
3. 把提取到的 `pen_xyz / holder_xyz` 传给 `PickPlaceConfig3D.pick_pose/place_pose`。
4. 每个控制周期更新动态障碍，调用 `update_obstacles` + `plan_to_pose` 做重规划。

---

## 你下一步建议

- 先跑通固定笔、固定笔筒（已知位姿）。
- 再接入视觉检测输出（未知位姿）。
- 最后把动态障碍从“人工给定时间线”替换为“实时障碍检测结果”。

这样就能逐步满足你负责人提的完整目标。
