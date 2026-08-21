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
   │   ├── types.hpp
   │   ├── differential_drive.hpp
   │   └── kinematics.hpp          // 唯一入口，先只 include 上面两个
   ├── tests/test_kinematics.cpp
   └── examples/differential_drive_example.cpp

2. types.hpp（不需要模板，现在就能写）：
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
   #include "types.hpp"
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

## STAGE 3：speed_limiter（本库唯一的"状态"）

```
// 目标：平滑限幅，三通道独立
CLASS SpeedLimiter:
    state: prev_  (Twist, 初始全 0)
            max_vel_, max_acc_  (每通道各一组)

    METHOD setMaxVel(vx, vy, wz): 存上限
    METHOD setMaxAcc(ax, ay, az): 存上限
    METHOD step(target, dt):
        FOR each channel c IN [vx, vy, wz]:
            delta_c = clamp(target.c - prev_.c, -max_acc_.c * dt, +max_acc_.c * dt)
            prev_.c += delta_c
        RETURN prev_                    // 限幅后输出（内部也可先夹 max_vel）

测试：
   CHECK 阶跃输入:  target=2.0, a_max=1.0, dt=0.1 → 每步最多走 0.1
   CHECK 收敛:      连续 step 最终到达 target
   CHECK 单调:      输出序列不超调、无震荡
   CHECK 保持:      target 不变时输出不再变化

验收：全绿。注意 dt 非法值（0/负）防御：IF dt <= 0 RETURN prev_
```

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
