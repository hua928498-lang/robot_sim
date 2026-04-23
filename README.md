# robot_sim task scaffold

一个可扩展的任务骨架，包含：

- 相机感知接口（可接 SAPIEN / ManiSkill）
- A* 避障规划
- 动态障碍触发重规划
- 简化动力学约束执行检查
- 已知位姿与未知位姿两个阶段

## 运行

```bash
python -m robot_task.main
```
