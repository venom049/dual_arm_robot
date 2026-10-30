# W3 Dual Arm Workspace

本仓库负责 W3 双臂机械臂的 CAN bridge、ros2_control、真机启动和控制接口。工作源码的原远程为 `Ding141/dual_arm_robot`；本轮成果已合并至用户指定的 [venom049/dual_arm_robot 主分支](https://github.com/venom049/dual_arm_robot/tree/main)，保留原有提交历史及标定文件。

截至2026-10-30，本轮反馈新鲜度、最新反馈读取及测试说明见 [FEEDBACK_LATEST_20261019.md](docs/FEEDBACK_LATEST_20261019.md)。NEXT训练、模型、补偿与实机波形结果保存在配套仓库的[成果总入口](https://github.com/Ding141/factr2/blob/liyq-dev/results/20261030/README.md)。

本仓库需要与配套的 FACTR2 仓库一起使用：

- W3 控制：`Ding141/dual_arm_robot`
- FACTR2：`Ding141/factr2`

本地推荐目录：

```text
/home/users/w3_dual_arm_ws
/home/users/factr2
```

源码位于 `src/`，`build/`、`install/` 和 `log/` 是本地生成目录。W3 仓库只负责发布机器人状态和接收控制命令；NEXT 的数据采集、训练和推理放在另一个仓库，通过 ROS 2 话题连接。

现有真机安装、标定和启动流程见 [src/README.md](src/README.md)，控制链路说明见 [src/docs/CONTROL_PIPELINE.md](src/docs/CONTROL_PIPELINE.md)。

当前本机右臂、ROS域74、FACTR2模型与网页的启动入口：[2026-10-10右臂启动说明](src/docs/RIGHT_ARM_NEXT_STARTUP.md)。

空载偏差、卸载残留和末端奇异提示的新版采集/训练代码位于配套 FACTR2 仓库，操作说明：[直接末端位姿采集与训练](https://github.com/Ding141/factr2/blob/liyq-dev/docs/operator_guide/NEXT_POSE_CAPTURE_20261014.md)。本控制器新增 `/joint_position_controller/stop` 平滑制动；位姿采集桥一次下发完整原生目标，继续记录实际 `command_state`。10 月 14 日已在慢/快 10/12°/s 下完成真机采集，10 月 15 日完成单会话初步训练；高速覆盖、独立会话和接触卸载验收尚未通过。
