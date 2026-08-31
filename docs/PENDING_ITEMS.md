# lunokhod 悬挂事项清单（PENDING ITEMS）

> 记录所有"知道存在但没闭合"的事项。每条含：状态 / 内容 / 下一步 / 优先级。
> 更新规则：处理完一条划掉，新增一条补上，日期标注。

---

## 主线（底盘运动系统）

### P1. LineFollower（灰度寻迹）⬜ 未开工
- **内容**：控制链最后一块——灰度传感器 → 偏差 → 纠偏 Twist。完成后 `灰度→纠偏→限幅→逆解→轮速` 全闭环
- **素材**：legacy C 巡线导航（`Lib/control/nav/`）
- **下一步**：先读 legacy 巡线代码，定组件边界（传感抽象 + 纠偏控制器 + Twist 输出）
- **优先级**：🔴 高（主线的下一块）

### P2. IMU 姿态解算系统 ⬜ 设计阶段
- **内容**：小车 IMU 需求（先 gyro_z，扩展多维姿态解算）。ICM-20602 = **6 轴无磁力计**（3 陀螺 + 3 加速度计）→ yaw 只能纯陀螺积分（漂移），roll/pitch 靠加速度计互补
- **方向**：最小通用核心（镜像 wheel 思路）——`Ahrs` 组件：gyro 积分 + accel 互补（Mahony）+ 可选 mag；Config 聚合 + dt 穿透 + 单位约定 + 行为锚点
- **素材**：`Lib/driver/hal/imu/icm20602/`（驱动）+ 用户 vibecode 过的 Mahony
- **下一步**：先讲原理（互补滤波/四元数/Mahony），再定组件边界写 DESIGN
- **优先级**：🔴 高（近期有真机需求）

## 架构/一致性

### P3. Ramp 跨项目统一（DRY）⬜ 待决策
- **内容**：`control/wheel/src/ramp.cpp` 与 `twist_acc_limiter` 的 `ramp()` **算法完全相同**（max_step=rate·dt + clamp）
- **权衡**：提取公共 `inc/ramp.hpp`（两项目共用）vs 接受重复 20 行（零依赖原则）
- **下一步**：决策后执行；注意 wheel v0.1.0 已发布，提取是 v0.2.0 的变更（破坏性）
- **优先级**：🟡 中（两项目都已发布，随时可做）

### P4. kinematics 分支名统一 ⬜ 未执行
- **内容**：本地 `main` vs 远端 `master` 不一致
- **下一步**：`git branch -m main master` + 远端处理（或反方向）
- **优先级**：🟡 中

### P5. kinematics docs 去留 ⬜ 待拍板
- **内容**：(a) 仓库保留副本（开源库惯例）vs (b) 瘦身纯代码库（README 指路上级目录）
- **优先级**：🟢 低

### P6. ARCHITECTURE.md 术语同步 ⬜ 部分完成
- **内容**：SpeedLimiter → TwistAccLimiter、三级愿景 → v1 范围 + Wheel 承接 S 曲线
- **优先级**：🟢 低（多数已由 agent 生成 docs 覆盖，待一致性核对）

## 重构线

### P7. FOC 重构 ⬜ 未开工
- **内容**：`Lib/foc/`（foc.c + foc_transform.h + gimbal_ctrl + motor_param + motor_tune）→ C++ 组件化
- **优先级**：🟡 中（foc 云台是用户中期目标）

### P8. Gimbal 重构 ⬜ 未开工
- **内容**：`legacy/gimbal_2axis/`（STM32H743 工程）→ 与 lunokhod 体系整合
- **优先级**：🟡 中

## 学习线（ROS）

### P9. ROS2 Humble（WSL2）⬜ 未起步
- **内容**：里程碑 1 = 发布/订阅 + kinematics 包装成 `/cmd_vel` 节点；未来 Gazebo 重仿真（可能双系统）
- **下一步**：WSL2 装 Ubuntu 22.04 + ROS2 Humble
- **优先级**：🟢 低（独立学习线，与竞赛主线并行）

## 评审收尾（来自 REVIEW_RESPONSE.md 2026-08-22 查验）

### P10. 根级版本控制决策 ⬜ 待决策
- **内容**：根目录无 git，docs/（ARCHITECTURE/DESIGN/复盘）不在版本控制；三仓互不关联 + 工作树脏（kinematics 5 docs 未提交删除、twist_acc_limiter 未提交改动）——评审最大隐患
- **选项**：(a) 并成一个仓（带根提交）vs (b) 维持三仓 + 根级 docs 仓
- **下一步**：决策后执行；此前先做 P11 的工作树收尾（未提交改动落盘）
- **优先级**：🔴 高

### P11. PID 默认语义决策 ⬜ 待决策（与定案冲突）
- **内容**：评审建议 `limit_out_=0` 默认改为"不限制"；但 wheel 已定案 **"0=disabled"惯例**（max_rate=0 关斜坡、thresh=0 关分离）——两者冲突
- **决策点**：0 语义维持（文档写明坑）vs 改 sentinel vs 分层（0=中立 + preset）
- **影响**：若改，牵动 wheel v0.2.0（已发布 v0.1.0 不可改）
- **优先级**：🔴 高（真实踩坑风险，但改法需先拍板）

### P12. 防御校验 ⬜ 未开工
- **内容**：OmniDrive 构造 `wn>6` 越界写 `J[6][3]`、`wn<2` 数学无意义——加断言/参数校验；twist_acc_limiter `reset()` 补测试
- **优先级**：🟡 中（2 小时量级）

### P13. LICENSE + 主观预测 ⬜ 未开工
- **内容**：补 LICENSE（选 MIT）；删 `docs/DESIGN.md` 的"预期 Stars 100-250"类主观预测
- **优先级**：🟢 低（10 分钟量级）

### P14. CI 流水线 ⬜ 未开工
- **内容**：GitHub Actions——三仓各一（-Werror + 跑测试 + example），消除"全绿依赖手动"
- **优先级**：🟢 低（半天量级）

---

## 处理记录
- 2026-08-22：wheel v0.1.0 发布（主线第 3/4 块完成）；本文档创建
- 2026-08-22：外部评审查验（REVIEW_RESPONSE.md 落盘，14 条全属实无误报）；新增 P10~P14；docs 落盘（WHEEL_LESSONS / PENDING_ITEMS / AHRS_BUILD_GUIDE / REVIEW_RESPONSE）
