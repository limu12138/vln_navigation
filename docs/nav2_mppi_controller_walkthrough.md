# nav2_mppi_controller 运行顺序与代码导读（面向理解 MPPI 原理）

本文以“程序运行顺序”为主线，把 `nav2_mppi_controller` 的关键文件串起来。

> 建议阅读顺序：先看 1/2/3 的调用链，再进 4 的 MPPI 细节。

## 1. 入口：MPPIController 是 Nav2 的 Controller 插件

- 插件导出点：`PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)`
- 它被 `controller_server` 加载，并周期性调用 `computeVelocityCommands()`。

关键文件：
- `src/navigation2/nav2_mppi_controller/include/nav2_mppi_controller/controller.hpp`
- `src/navigation2/nav2_mppi_controller/src/controller.cpp`

你会在 `MPPIController::computeVelocityCommands()` 看到每个控制周期的固定流程：
1) 取动态参数锁
2) 将全局路径 plan 变换到局部坐标系（`PathHandler::transformPath()`）
3) 锁 costmap（保证评分期间地图一致）
4) 调用 `Optimizer::evalControl()` 输出 `TwistStamped`

## 2. Optimizer：MPPI 的核心优化器

关键文件：
- `src/navigation2/nav2_mppi_controller/include/nav2_mppi_controller/optimizer.hpp`
- `src/navigation2/nav2_mppi_controller/src/optimizer.cpp`

核心入口：`Optimizer::evalControl(robot_pose, robot_speed, plan, goal_checker)`

在一次控制周期内，Optimizer 大体做：
- `prepare(...)`：把本周期的状态、速度、路径等拷贝/转换到张量
- `optimize()`：
  - `generateNoisedTrajectories()`：采样控制 -> rollout 轨迹
  - `critic_manager_.evalTrajectoriesScores(...)`：按 critic 叠加成本
  - `updateControlSequence()`：softmax 权重更新控制序列
- （可选）平滑控制序列（Savitsky–Golay）
- 从控制序列中取本周期要输出的第一个控制量（或 offset 后）

### 2.1 MPPI 在代码里的“形状”

- 控制序列 `control_sequence_`：形状是 `[time_steps]`，表示“名义控制”
- 采样控制 `state_.cvx/state_.cwz`：形状是 `[batch_size, time_steps]`
- 采样轨迹 `generated_trajectories_`：形状是 `[batch_size, time_steps, ...]`
- 成本 `costs_`：形状是 `[batch_size]`

MPPI 的一次迭代可以理解为：
- 在名义控制序列周围采样出很多条控制序列（batch）
- rollout 成很多条轨迹
- 每条轨迹打分得到一个标量成本
- 用 softmax 权重把采样控制“加权平均”回一条新的名义控制序列

## 3. NoiseGenerator：如何采样控制序列

关键文件：
- `src/navigation2/nav2_mppi_controller/include/nav2_mppi_controller/tools/noise_generator.hpp`
- `src/navigation2/nav2_mppi_controller/src/noise_generator.cpp`

核心点：噪声张量 `noises_vx/noises_wz/noises_vy` 的 shape 是 `[batch_size, time_steps]`。

采样公式对应到代码：
- `state.cvx = control_sequence.vx + noises_vx`
- `state.cwz = control_sequence.wz + noises_wz`

也就是：$u_k^i = u_k + \epsilon_k^i$，其中 $\epsilon \sim \mathcal{N}(0, \sigma^2)$。

## 4. CriticManager + Critics：如何对轨迹打分

关键文件：
- `src/navigation2/nav2_mppi_controller/include/nav2_mppi_controller/critic_manager.hpp`
- `src/navigation2/nav2_mppi_controller/src/critic_manager.cpp`
- `src/navigation2/nav2_mppi_controller/include/nav2_mppi_controller/critics/*.hpp`
- `src/navigation2/nav2_mppi_controller/src/critics/*.cpp`

CriticManager 会从参数 `critics: [ ... ]` 里读出 critic 名称列表，然后用 pluginlib 加载。

评分时序：
- 每个 critic 看到同一份 `CriticData`（含 state、轨迹、路径、costs 等引用）
- critic 通过 `data.costs[...] += ...` 叠加自己的成本项

## 5. updateControlSequence：softmax 更新（MPPI 的核心更新步骤）

在 `Optimizer::updateControlSequence()` 里能看到典型结构：
1) 计算与噪声相关的控制代价项（信息论 MPPI 常见写法）
2) `costs_normalized = costs - min(costs)`（数值稳定）
3) `weights = exp(-1/T * costs_normalized)`
4) softmax 归一化权重
5) `control_sequence = sum(sampled_controls * weights)`

## 6. 建议你实际调试时看哪些 topic

- `/cmd_vel_nav`：controller_server 输出（通常是 MPPI 的直接输出）
- `/cmd_vel`：可能经过 velocity_smoother，可能重新混合线/角

---

如果你希望我继续“逐行注释”更多文件，建议下一步从 `optimizer.cpp` 的几个函数开始：
- `Optimizer::evalControl()`
- `Optimizer::generateNoisedTrajectories()`
- `Optimizer::updateControlSequence()`
- `Optimizer::integrateStateVelocities(...)`
