# 底层反馈积压：研究依据、实现及验证

## 研究依据和选型

2026-10-16日志中，CAN与驱动状态基本重合，但驱动状态与关节反馈按原时间戳比较出现约1.07°的位置差。驱动发布约350 Hz，硬件read约300 Hz；接收端KeepLast(50)，每次read调用一次spin_some。之前的隔离复现中，深度50的缓存年龄中位数约141.45 ms，深度1约1.44 ms。这些不是新的真机延迟标定参数。

|资料|与本问题相关的结论|本次使用方式|
|---|---|---|
|[Casini等，ECRTS 2019，Response-Time Analysis of ROS 2 Processing Chains Under Reservation-Based Scheduling](https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.ECRTS.2019.6)|执行器与处理链的实现选择会影响端到端响应；仅看发布频率不足以判断实时性。|逐层检查生产、消费与排队。论文研究的是Crystal，未将其响应时间界直接套用于Humble。|
|[Yates等，JSAC 2021，Age of Information: An Introduction and Survey](https://arxiv.org/abs/2007.08564)|信息年龄关注当前可用状态距其源时间有多旧，提供了区别于吞吐量的数据新鲜度视角。|控制端消费最新快照并显式记录源年龄；原始采集端仍保留全量日志。这是本项目的工程选型，不声称对所有控制系统最优。|
|[ROS 2 Humble QoS官方说明](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)|KeepLast和depth限制历史长度；传感器数据可使用best effort策略。|反馈订阅使用KeepLast(1)、best effort、volatile，避免消费50帧旧状态。|

参考的开源实现：

- [ros2/rclcpp，Humble executor.cpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/executor.cpp)：核对spin_some与spin_all的工作收集语义。使用一次创建的SingleThreadedExecutor，保留每次只收集一轮就绪工作；不使用无界追赶循环。
- [ros-controls/realtime_tools，Humble](https://github.com/ros-controls/realtime_tools/tree/humble)：使用RealtimePublisher发布预分配的反馈副本，控制读循环仅trylock。独立接收线程加RealtimeBuffer可进一步隔离ROS工作，但本次不引入额外接收线程或共享状态。
- [ros2/examples，Humble wait_set](https://github.com/ros2/examples/tree/humble/rclcpp/wait_set)：显式接收/触发是可选方向，本次未更换执行模型。

## 已实现

代码在`src/ieir_controllers/src/dm_hardware_interface.cpp`及对应头文件。

1. **最新反馈消费**：将状态订阅从深度50改为深度1，命令通路保持原策略。read、激活和停用轮询复用硬件节点的执行器，避免每个周期创建并添加/删除节点。不通过提高电机或控制频率解决积压。
2. **阻止时间回退**：数组源时间戳缺失、非法、重复、倒序、未来或超过允许年龄时，不覆盖缓存。每个关节也检查自身源时间戳；缺失的关节时间戳回退到数组时间戳。合法的状态码仍保留，激活/停用确认额外要求反馈有效。
3. **记录新鲜度**：每个关节增加只读状态接口`feedback_source_time`、`feedback_source_age`、`feedback_receive_age`、`feedback_valid`。时间单位为秒；receive_age基于本机steady clock，source_age基于ROS时钟。没有数据时，源时间为NaN、年龄为无穷大、valid为0。
4. **保留原时间的输出**：新增`/dm_hardware_interface/feedback_state`（sensor_msgs/JointState）。位置、速度、力矩仍为URDF坐标系中的rad、rad/s、Nm；原始整数时间戳不转换成控制周期时间。当前桥接器各关节时间相同；若以后提供不同关节源时间，整体消息使用最早的时间，并保留逐关节源时间监测值，不能据此宣称同时采样。
5. **不伪造新数据**：只有全部配置关节有效且接受了新帧时才发布上述快照；坏值不补零，不给旧值重新打时间戳。实时发布锁忙时跳过这次发布，不阻塞read。原`/joint_states`保持标准广播行为，不能将其新header当成设备采样时刻。
6. **采集留证**：FACTR2完整bag增加`/dynamic_joint_states`和新增源时间反馈话题，源码快照增加硬件接口cpp/hpp。没有切换现有adapter的state_topic或改动正在部署的NEXT输入。

硬件参数`feedback_max_age_s`默认0.05秒，必须为有限正数。这是新鲜度有效性门限，不是估计出来的时间平移补偿。超时后监测标记失效、源时间反馈停止发布；数值缓存保持最后有效值，不自动切换或停用电机/重力控制器。原有健康监测及运动监护仍需保持启用。

**时间语义边界**：MotorState目前只有桥接器发布时刻，没有电机采集时间。新增输出修复了“旧缓存被标成新控制周期”的可观测性问题，不能证明电机内部采样、CAN发送或接收线程绝对无延迟。当前read仍执行ROS回调，不宣称硬实时WCET已得到证明。

source_age检查要求桥接器与硬件接口共享ROS时钟，本机部署符合此假设。跨机器部署应先验证时钟同步；时钟回退时将失效反馈明确暴露，需受控重新配置恢复，不在测试数据上自动拟合时钟偏移。数值型状态接口中的绝对时间用于监测，需精确整数纳秒时使用新增JointState的header及原始bag。

## 验证

独立构建目录`log/feedback_latest_20261019/build`，没有覆盖主工作区install。测试环境强制ROS域196和localhost-only，仅发布合成MotorStateArray到隔离话题；从不调用激活、停用或电机write。CTest显式将新构建库目录放在LD_LIBRARY_PATH首位，避免测试误加载旧安装库。

6项真实硬件接口类测试通过：

- 连续突发只消费最新值，校准方向、单位和源时间戳正确。
- 无输入/超时不把旧值或零值伪装成新反馈。
- 重复、倒序、未来、过旧、无时间戳及非法时间戳均不能覆盖新缓存，随后合法帧可恢复。
- 缺失、离线、非有限关节不能组成有效完整快照。
- 混合关节时间使用最早源时间，旧关节不借新数组时间刷新缓存。
- 350 Hz生产、300 Hz读取时，新反馈源年龄P95为2.86496 ms；连续暂停读取200 ms后，首个恢复样本年龄为0.100921 ms。宽松判据为两者均小于20 ms。数值是本机隔离软件测试结果，不是总线或机器人真实时延上界。

FACTR2原采集保护相关19项测试也通过。详细输出在`log/feedback_latest_20261019/test.log`与`build/Testing/Temporary/LastTest.log`。

复算：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
cmake -S src/ieir_controllers -B log/feedback_latest_20261019/build \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PWD/log/feedback_latest_20261019/install" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build log/feedback_latest_20261019/build --target dm_feedback_latest -j2
ctest --test-dir log/feedback_latest_20261019/build -R '^dm_feedback_latest$' --output-on-failure
```

## 真机接入与验收边界

最初的离线验证仅完成源码实现、独立编译和隔离测试。之后在2026-10-19获得操作者“双臂已可靠支撑，右臂空载且区域清空”的确认，执行受控重启并整体加载新库。旧库已单独备份，采用新inode原子替换后启动，没有对活动映射库原位覆盖。

后续按同一冻结运动协议比较CAN→驱动→关节状态和新feedback_state：确认源年龄在全程及负载下稳定、暂停恢复不追赶旧队列、源时间戳未被覆盖、旧帧/离线门禁正常，再重新观察空载残差和惯性。不能以软件测试通过替代这一验收。

反馈时序改善可能改变实际闭环动态及NEXT的输入分布。旧模型基于旧通路训练，不应假定修复后仍然有效；验证后再统一采集/训练/推理的信号定义，必要时重新采集和训练。没有用本批独立日志重新拟合任何补偿延迟，也没有顺带修改速度半码定义、控制增益或摩擦前馈。

## 2026-10-19真机结果

右臂先低速进入模型范围，再完成同一冻结协议的62段运动、26次正式单程。左臂保持；原版NEXT模型、增益、前馈和滤波保持不变。右臂最终返回原目标，最大静态跟踪误差0.897°。733799帧原始CAN接收丢帧及内核时间戳缺失均为0；H5为29890行、100 Hz，最大间隔14.91 ms；NEXT同步观察30185行。

|指标|10月16日旧通路|10月19日新通路|
|---|---:|---:|
|桥接状态到关节状态的最大位置差|1.070974°|0.021857°|
|硬件读周期反馈源年龄P95|无源时间可比观测|2.737 ms|
|硬件读周期反馈源年龄最大值|无源时间可比观测|5.086 ms|
|运动单程原始残差向量RMS均值|0.231628 Nm|0.313544 Nm|
|有效末端力帧的力范数RMS|1.9319 N|2.8616 N|
|正式运动末端力有效输出覆盖率|71.20%|69.54%|

新源时间话题89895帧按源时间与桥接状态对应，位置浮点误差量级1e-14°、速度差为0、header差为0。右臂逐关节有效性监测未记录到失效样本。这里确认的是本轮负载下的软件反馈消费与源时间传播，不能推出电机内部采样或整个链路物理延迟为零；也没有将旧隔离实验的141 ms当作真机时延标定值。

**反馈积压明显改善，但空载误报没有通过验收。** 原始CAN仍支持低速脉动，速度半码基线仍在；旧模型在新时序输入下未显示更小残差。前后测试跨日期，温度及实际姿态、运动历史不完全相同，不构成单因素因果结论。本次数据继续作为独立诊断集，不用于重新拟合延迟、扣除零偏或训练。

完整报告：`/home/venom/factr2/log/next_feedback_trial_20261019_v1/comparison.md`，逐层分析及曲线见同目录`REPORT.md`。安装二进制SHA256为`542949d1e273667dc6f5b4b2a6c576f054f2e66cb77ae18cf90e6d906f051a15`；部署、旧库备份与重启证据位于`/home/venom/dual_arm_robot/log/feedback_live_20261019/`。

空间整理仅删除可重建的编译对象文件，并将部分历史文本观测日志无损压缩为`.gz`；压缩前后SHA256核对一致。原始CAN、bag、H5和模型保留。日志归档清单位于`/home/venom/factr2/log/feedback_live_20261019/lossless_log_archives.json`；需要旧CSV/JSONL明文时可用`gzip -dk 对应文件.gz`恢复，先确认恢复空间。
