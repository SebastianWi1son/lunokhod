# Kinematics 开发指南（伪代码版）

> 配套文档：`DESIGN.md`（架构/公式）、`THEORY_AND_REFERENCE.md`（理论推导）
> 本文是"怎么做"的操作流程，伪代码 = 执行步骤，不是最终代码形状。

---

## 总览：五阶段 + 验证循环

```
FOREACH stage IN [1, 2, 3, 4, 5]:
    DO stage
    RUN 验证循环(stage)        // 每阶段结束必须验证，不许带病进下一阶段
ENDFOR

验证循环 = {
    step1 编译:   g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined test_kinematics.cpp
    step2 测试:   跑测试，断言全绿
    step3 对照:   手算/笔算 2~3 个关键数值，确认"测试通过 ≠ 公式对"
    step4 提交:   git commit（小步提交，每阶段一个 commit）
}
```

---

## STAGE 1：types + differential_drive（第一道门）

```
// 目标：最小可用 + CRTP 模式跑通
// 验收：直行/旋转/互逆三条数值断言全绿

1. 建目录：
   kinematics/
   ├── include/kinematics/
   │   ├── contracts.hpp
   │   ├── differential_drive.hpp
   │   └── kinematics.hpp          // 唯一入口，先只 include 上面两个
   ├── tests/test_kinematics.cpp
   └── examples/differential_drive_example.cpp

2. contracts.hpp（不需要模板，现在就能写）：
   STRUCT Twist { float vx; float vy; float wz; }        // 输入
   STRUCT WheelSpeeds { float values[6]; uint8_t count; } // 输出，定长数组

3. differential_drive.hpp（CRTP 模式，套用 Dog/Cat）：
   TEMPLATE<typename Derived> CLASS Chassis:
       METHOD inverse(cmd):                            // 统一接口
           RETURN static_cast<const Derived*>(this)->inverse_impl(cmd)
       METHOD forward(spd):
           RETURN static_cast<const Derived*>(this)->forward_impl(spd)

   CLASS DifferentialDrive : Chassis<DifferentialDrive>:
       CTOR(wheelbase w, wheel_radius r): 存成员 w_, r_
       METHOD inverse_impl(cmd):
           out.count = 2
           out.values[0] = (cmd.vx - cmd.wz * w_ / 2) / r_    // 左轮
           out.values[1] = (cmd.vx + cmd.wz * w_ / 2) / r_    // 右轮
           RETURN out
       METHOD forward_impl(spd):
           t.vx = r_ * (spd.values[1] + spd.values[0]) / 2
           t.wz = r_ * (spd.values[1] - spd.values[0]) / w_
           t.vy = 0                                          // 非完整约束
           RETURN t

4. tests/test_kinematics.cpp（手写 CHECK 宏，不引框架）：
   CHECK 直行:   inverse({0.5, 0, 0})   → 两轮相等 = 16.667   (w=0.16, r=0.03)
   CHECK 旋转:   inverse({0, 0, 1})     → 左负右正 = ±2.667
   CHECK 互逆:   forward(inverse({0.5,0,1})) ≈ {0.5, 0, 1}   (误差 < 1e-5)
   CHECK 零输入: inverse({0,0,0})       → 全 0

5. kinematics.hpp：
   #pragma once
   #include "contracts.hpp"
   #include "differential_drive.hpp"

6. examples/differential_drive_example.cpp：
   DifferentialDrive chassis(0.16f, 0.03f)
   打印 inverse({0.5, 0, 1}) 三行结果，与手算对照

验收：验证循环 4 步全过。此阶段不写 mecanum/omni，克制。
```

---

## STAGE 2：mecanum + omni（翻译官显式化）

```
// 目标：三种底盘收敛到"同一翻译官 + 三本字典"
// 时机：写第二个底盘时发现"公式又是手写展开" → 停下，先提取翻译官

1. 提取翻译官（放独立头文件或 differential_drive.hpp 内）：
   TEMPLATE<size_t N> METHOD jacobian_apply(const float (&J)[N][3], const Twist& cmd):
       FOR i IN 0..N-1:
           out.values[i] = J[i][0]*cmd.vx + J[i][1]*cmd.vy + J[i][2]*cmd.wz
       out.count = N
       RETURN out
   // 这就是"矩阵 × 向量"，所有线性底盘共用，一份代码

2. mecanum_drive.hpp：
   CLASS MecanumDrive : Chassis<MecanumDrive>:
       CTOR(lx, ly, r): 存成员
       METHOD inverse_impl(cmd):
           J[4][3] = { {1,-1,-(lx+ly)}, {1,1,(lx+ly)}, {1,1,-(lx+ly)}, {1,-1,(lx+ly)} } / r
           RETURN jacobian_apply(J, cmd)                // ← 一行
       METHOD forward_impl(spd):                        // 伪逆退化为平均
           vx = r * (FL+FR+RL+RR) / 4
           vy = r * (-FL+FR+RL-RR) / 4
           wz = r * (-FL+FR-RL+RR) / (4*(lx+ly))

3. omni_drive.hpp：
   CLASS OmniDrive : Chassis<OmniDrive>:
       CTOR(R, n, gamma, r): 存成员
       METHOD inverse_impl(cmd):
           FOR i IN 0..n-1:
               θ = i * 2π/n + gamma
               J[i] = { sin(θ), -cos(θ), -R } / r
           RETURN jacobian_apply(J, cmd)                // ← 一行
       METHOD forward_impl(spd):
           // 最小二乘伪逆 pinv(J)：3 轮时 3×3 求逆（constexpr 函数），
           // 先手写 3 轮特例，N>3 用一般公式

4. 测试新增：
   CHECK Mecanum vy 通道:  {0, 0.5, 0} → FL=FR 同号, RL=RR 同号, 符号相反
   CHECK Mecanum 互逆:     forward(inverse({0.5,0.3,1})) ≈ 原值
   CHECK Omni N=3 互逆:    forward(inverse(...)) ≈ 原值
   CHECK Omni N=4 互逆:    同上
   CHECK 差速回归:         阶段 1 的断言仍全绿（改代码不许破坏旧功能）

验收：验证循环全过。此时三种底盘的 inverse_impl 都只有一行矩阵乘。
```

---

## STAGE 3：SpeedLimiter（可选附赠组件 · Twist 空间斜坡发生器）

### 3.1 定位（2026-08 修正）

**小礼包**：可选组件，独立文件 `inc/speed_limiter.hpp`，**不进 chassis.hpp 聚合入口**。

- 为什么是"小礼包"不是核心：限幅是**应用层策略**，不是运动学数学——行业惯例放上层（ROS 导航栈 acc_lim、驱动器固件 ramping），没有一家运动学库内置它
- 升维机会：THEORY 3.1 分层图里 "Velocity Limiter" 那一格。将来做完整控制系统库（限幅→逆解→里程计→轮PID）时，它升为正式组件；没机会就自己用
- 核心保持纯净：chassis.hpp 聚合入口 = types + kinematics + 三驱动类，不含它

### 3.2 设计决策（开工前记住，别摇摆）

| 决策点 | 结论 | 为什么 |
|---|---|---|
| 限幅对象 | Twist 三通道**独立**限幅（vx/vy/wz） | 运动学一致性：不能在轮速空间限（J 约束会被单个轮子的独立限幅破坏）；三通道独立因各向加速度极限不同 |
| dt 来源 | `limit(cmd, dt)` **每帧传参** | 与 legacy `pid_calculate(..., dt)` 同风格；实测 dt 自适应，比固定步长灵活 |
| 构造 | 注入三通道 `max_acc` | 参数构造期定死；不做 setter（除非运行时调参是硬需求——YAGNI） |
| 状态 | `prev_`（Twist） | 本组件唯一状态：上一帧输出；斜坡必须知道"上次走到哪" |
| 起步 | 首帧 `prev_ = {0,0,0}` | 从静止起步，0→0.5 按斜坡爬升 |
| 防御 | `dt <= 0` → 直通返回 cmd | 非法 dt 不冻结不崩；直通是安全侧（不阻挡指令） |
| clamp | 手写三目 或 `std::clamp` | `<algorithm>` 零成本；嵌入式风格可手写三目 |
| 复位 | `reset()` 清 prev_ | 停赛后重新起步用；等价 legacy `dsp_traj_reset` |
| 上报机制 | 方案 a：`LimitResult` 返回式（每通道饱和标志） | 一次拿全、无“事后查询”时序陷阱；匹配“vx 撞墙 wz 没撞”的观测需求；调用方结构化绑定 |

### 3.3 伪代码（完整）

```cpp
// inc/speed_limiter.hpp —— 可选附赠：Twist 空间加速度限幅器（斜坡发生器）
// 定位：上游指令（导航/PID/遥控，可任意跳变）→ 平滑斜坡输出 → kinematics inverse
// 使用：需要时单独 #include "speed_limiter.hpp"（chassis.hpp 不含它）

#include "contracts.hpp"          // Twist

struct LimitResult {              // 限幅输出 + 每通道饱和标志（上报机制）
    Twist out;                    // 限幅后的速度指令
    bool  vx_lim, vy_lim, wz_lim; // 对应通道本帧是否被限幅（撞墙）
};

class SpeedLimiter {
public:
    SpeedLimiter(float acc_vx, float acc_vy, float acc_wz)
        : acc_{acc_vx, acc_vy, acc_wz}, prev_{0, 0, 0} {}

    // 每帧调用：cmd 可跳变，输出每通道最多变化 acc*dt
    LimitResult limit(const Twist& cmd, float dt) {
        if (dt <= 0.0f) return {{cmd, false, false, false}};   // 防御：非法 dt 直通
        LimitResult r;
        r.out.vx = ramp(cmd.vx, prev_.vx, acc_.vx, dt, r.vx_lim);
        r.out.vy = ramp(cmd.vy, prev_.vy, acc_.vy, dt, r.vy_lim);
        r.out.wz = ramp(cmd.wz, prev_.wz, acc_.wz, dt, r.wz_lim);
        prev_ = r.out;                                          // 状态更新
        return r;
    }

    void reset() { prev_ = {0, 0, 0}; }

private:
    // clamp 到 [prev−acc·dt, prev+acc·dt]，lim 报告是否被夹住
    static float ramp(float target, float prev, float acc, float dt, bool& lim) {
        float max_step = acc * dt;              // 本帧最大允许变化量
        float lo = prev - max_step;
        float hi = prev + max_step;
        lim = (target < lo) || (target > hi);
        return target < lo ? lo : (target > hi ? hi : target);
    }

    Twist acc_;     // 三通道加速度上限 (m/s², m/s², rad/s²)
    Twist prev_;    // 上一帧输出（本组件唯一状态）
};

// 调用方（C++17 结构化绑定）：
// auto [out, vx_lim, vy_lim, wz_lim] = limiter.limit(cmd, dt);
```

### 3.4 测试锚点（手算有理数）

| # | 场景 | 输入 | 期望输出序列 | 验证点 |
|---|---|---|---|---|
| T1 | 起步爬坡 | cmd=0.5, acc=1, dt=0.1 | 0, 0.1, 0.2, 0.3, 0.4, 0.5 | 斜率 = acc；收敛到目标 |
| T2 | 停止下坡 | 从 0.5 后 cmd=0 | 0.5, 0.4, ..., 0 | 上升/下降对称 |
| T3 | 方向反转 | 0.3 → -0.3 | 0.3, 0.2, 0.1, 0, -0.1, -0.2, -0.3 | 过零平滑无跳变 |
| T4 | 未饱和跟踪 | 0.1 → 0.15（acc 大） | 0.1, 0.15 | 慢变指令直通（限幅器不该拖后腿） |
| T5 | 三通道独立 + 上报 | vx 大幅跳变 | vx 序列按斜坡，且 **vx_lim==true、vy_lim==false、wz_lim==false** | 通道隔离 + 饱和标志正确 |
| T6 | 保持 | target 连续不变 | 输出不变 | 稳态无漂移 |

测试方式：`test/test_speed_limiter.cpp`，同样 ALL PASS + 退出码 0 + -Werror。

### 3.5 与 legacy dsp_ramp_t 的对照（你已经写过的东西）

| | legacy `dsp_ramp_t` | SpeedLimiter |
|---|---|---|
| 通道 | 单通道标量 | Twist 三通道 |
| 状态 | prev_out | prev_ |
| 接口 | `calc(ramp, target, dt)` | `limit(cmd, dt)` |
| 作用域 | 轮速空间（wheel 层） | **Twist 空间（上层）** |

**两级限幅架构**：上层 SpeedLimiter（Twist 空间，保运动学一致性）+ 下层 legacy dsp_traj（轮速空间 S 曲线，电机最后防线）——行业标准，你的 2024 代码已实现后半级。

### 3.6 命名评估（练手）

| 候选名 | 语义 | 评价 |
|---|---|---|
| `SpeedLimiter` | 限"速度" | 易与 max_vel（限速）混淆——它实际限的是**变化率** |
| `AccelLimiter` | 限加速度 | 最准确；但行为上是斜坡（ramp），名字是限幅（limiter） |
| `SlewRateLimiter` | slew rate = 变化率 | 行业术语，准确但英文门槛 |
| `RampFilter` | 斜坡滤波 | 弱化"限制"语义 |

方法名对照：`limit()` / `step()` / `update()`——legacy 风格是 `wheel_update` / `pid_calculate` / `dsp_ramp_calc`。命名三原则评估后自选，把结论写进 AGENT.md。

### 3.7 验收标准

1. `-Werror` 零警告；T1~T6 全 PASS，退出码 0
2. chassis.hpp 聚合入口**不含** speed_limiter（核心纯净）
3. DESIGN.md 架构树标注"可选附赠，非核心"（文档定位一致化）
4. examples/ 提供调用示范：链路 `limiter → inverse → 打印`（下一步的仿真演示素材）

---

## STAGE 4：测试加固 + 示例 + 文档

```
1. tests 加固：
   - 随机 fuzz:   FOR 1000 次: 随机 (vx,vy,wz) → forward(inverse(x)) 误差 < 1e-5
   - 边界:        wheelbase=0 / radius=0 的防御（或文档声明"不检查"）
   - 编译期:      static_assert 几个常数用例（constexpr 实例化验证）

2. examples/：
   - differential_drive_example.cpp
   - mecanum_drive_example.cpp
   - dual_sensor_fusion.cpp   ← sensor_fusion 模板（互补滤波 + 双点几何航向）

3. README.md：
   - 用法 3 行示例（include → 构造 → inverse）
   - 公式表（三张 J 矩阵）
   - 参数表（w/r、lx/ly/r、R/n/γ/r 的物理含义）
   - 刻意不覆盖的范围（引用 DESIGN.md）

4. 2D 仿真轨迹（可选但推荐，进 README 动图）：
   FOR 命令序列 IN [直行→转弯→原地转→后退]:
       轮速 = 库输出
       位姿积分: x += vx*cos(θ)*dt; y += vx*sin(θ)*dt; θ += wz*dt
       画轨迹 → 与预期形状（正方形/S 形）对照
```

---

## STAGE 5：平台 + 发布

```
1. 交叉编译冒烟:
   arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m4 -mfloat-abi=hard -fsyntax-only include/kinematics/kinematics.hpp
   // 确认 MCU 工具链可编、零动态分配、零异常依赖

2. PlatformIO 注册:
   写 library.json（name/version/frameworks=arduino?/platforms=*）
   本地 pio pkg install 验证

3. 发布前检查清单:
   □ 零依赖确认:  grep -r "#include <" 只出现 <cmath> <cstdint>
   □ 头文件保护:  全部 #pragma once
   □ 命名一致:    项目内 snake_case（与设计文档对齐）
   □ 阶段 1~3 测试全绿
   □ 示例可编译
```

---

## 设计决策备忘（开工前记住，别摇摆）

```
1. 不用纯虚基类（ChassisBase）：编译期已定底盘，运行时无需"插座"。
   加了反而多 vptr + 一次虚调用，违背"编译期模板派发"原则。
   将来若需运行时切换底盘（config 选型），再加 ChassisBase 不迟。

2. 参数放派生类构造器（w/r、lx/ly/r、R/n/γ/r），基类只做接口派发，不碰参数。

3. 翻译官显式化：先手写公式跑通，再抽 jacobian_apply + J 矩阵。
   顺序 = 先有重复，再收敛（不要一上来就上抽象）。

4. 统一接口签名：设计文档的 inverse<Chassis>(cmd) 不传对象有缺陷（参数在实例上），
   实现用 chassis.inverse(cmd) 成员调用（或 inverse(chassis, cmd)）。

5. C++17 陷阱备忘（foc 项目教训）：C99 指定初始化数组 [IDX] = {...} 在 C++17 不合法，
   C++20 才有。kinematics 里所有表格用构造函数/普通数组初始化。

6. 每次只做一件事：改公式 → 跑测试 → 提交。禁止"顺手重构"。
```

---

## 验证渠道速查（四层）

```
L1 单元测试（必须）:  tests/test_kinematics.cpp，数值断言 + 互逆 + fuzz
L2 编译期（必须）:    -Wall -Wextra -Werror + sanitizer + static_assert
L3 2D 仿真（推荐）:   30 行位姿积分器画轨迹（pybullet/Webots 对纯运动学过重）
L4 交叉编译（发布前）: arm-none-eabi-g++ -fsyntax-only

心法：L1/L2 锁死"对不对"，L3 解决"看得见"，别用仿真器替代测试。
```
