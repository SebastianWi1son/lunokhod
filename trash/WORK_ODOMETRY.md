---
class: work
generated: false
---
> **类：D 施工单（临时物）** —— 参考实现，**不是代码源**。
> 唯一的代码源是 `inc/odometry.hpp`（**你手写**的那份）。
> 验收通过后本文件进 `trash/`。
> 设计权威：[`ODOMETRY_DESIGN.md`](ODOMETRY_DESIGN.md)（v3.1，O1~O9 + R1~R12 全拍板）

# WORK_ODOMETRY.md — odometry 施工单

本文件由 AI 生成（2026-09-13），**已实测 113/113 通过**（对照 `test/test_odometry.cpp`）。
生成方式：按设计 §4/§5 写一版，在仓库外编译运行；进仓库的是**标注版**，供你审阅。

---

## 0. 这份文件怎么用

> **前提**：先读 [`log/ODOMETRY_FAQ.md`](log/ODOMETRY_FAQ.md)
> （概念、术语表、一次 `update()` 的全景、你问过的 4 个问题的解答）。
> 术语不清就进来看代码，只会变成无意义的照抄。

### 方式：照抄 + 每抄一处瞄一眼 `D#`

按你自己的习惯抄（边抄边在脑子里过一遍），但**多做一个动作**：

> 每抄到一个 `// D#` 标注，看一眼它的「另一种写法会怎样」——
> **“我要是写成那样，哪条测试会红？”**

抄完跑 `ctest`。然后挑 1~2 处自己重写，再跑一次。

### 抄的时候盯住这 7 处

不用先盲猜（术语没建立之前猜不出来），但**抄到对应位置时要停下来想**：

| # | 盯什么 | 对应决策点 |
|---|---|---|
| 1 | `yaw_ += wz*dt` 和位移计算，谁在前？为什么？ | D2 |
| 2 | `twist_scale_` 乘了几个通道？ | D3 |
| 3 | 两个 yaw 出口是由几个成员算出来的？ | D4 |
| 4 | `reset()` 清的是「状态」还是「配置」？ | D6 |
| 5 | 样本里的 `twist_` 是标定前还是标定后？ | D8 |
| 6 | sink 已设、但 `ws == nullptr` 时发生什么？ | D7 |
| 7 | wrap 怎么做才不用 `while` 循环？ | D5 |

答案在同行的注释里（每一处都写了“为什么”和“另一种写法会怎样”）。

### 判分工：用 §3 的判据

**“这一处如果我写错了，测试会红吗？”**
会红 → 值得自己敲；不会红 → 直接接受。

---

## 1. 参考实现

### 1.1 `kinematics/inc/contracts.hpp`（**加** 3 行，不动已有内容）

```cpp
// D1：Pose 放数据层（O3 定案）—— 跨组件输出契约归 contracts.hpp，不进 odometry.hpp
struct Pose { float x_, y_, yaw_; };
```

> **为什么**：`Pose` 是"输出契约"，会被导航/融合层消费，和 `Twist`/`WheelSpeeds` 同级。
> **另一种写法**：把 `Pose` 定义在 `odometry.hpp` 里 → 将来融合层要用就得 include odometry，依赖方向倒挂。

### 1.2 `kinematics/inc/odometry.hpp`（新建）

```cpp
#pragma once
#include "contracts.hpp"
#include <cmath>
#include <cstdint>

// D1 样本契约（R4：ws 用 WheelSpeeds，支持 3/4/N 轮，不硬编码 4）
struct OdometrySample {
    uint32_t    tick_;   // 时钟刻度（单位由调用方约定）
    float       dt_;     // 本帧间隔 (s)
    WheelSpeeds ws_;     // 原始轮速（实际值，只作记录，不参与积分）
    Twist       cmd_;    // 指令 twist（期望值）
    Twist       twist_;  // 本帧实际体速度（D8：**标定后**）
    Pose        pose_;   // 本帧积分后位姿
};

// D7 注入式 sink（R5：带 ctx）—— 函数指针捕获不了状态
using SampleSink = void (*)(void* ctx, const OdometrySample&);

class Odometry {
public:
    explicit Odometry(float twist_scale = 1.0f) : twist_scale_(twist_scale) {}

    void set_sink(SampleSink sink, void* ctx = nullptr) { sink_ = sink; sink_ctx_ = ctx; }

    // D6：只清位姿；sink_ / sink_ctx_ / twist_scale_ 保留
    void reset() { x_ = 0.0f; y_ = 0.0f; yaw_ = 0.0f; }

    Pose update(const Twist& body_vel, float dt, uint32_t tick,
                const Twist& cmd = Twist{0.0f, 0.0f, 0.0f},
                const WheelSpeeds* ws = nullptr) {
        // D3：twist_scale_ 乘**三个**通道（均匀轮径误差等价于整体 Twist 缩放）
        const float vx = twist_scale_ * body_vel.vx_;
        const float vy = twist_scale_ * body_vel.vy_;
        const float wz = twist_scale_ * body_vel.wz_;

        // D2：半隐式欧拉（R6）—— 必须先积分 yaw，再用**新** yaw 旋转位移
        yaw_ += wz * dt;
        const float c = std::cos(yaw_);
        const float s = std::sin(yaw_);
        x_ += (vx * c - vy * s) * dt;
        y_ += (vx * s + vy * c) * dt;

        const Pose p = pose();   // D9：返回值与 pose() 同源，保证一致

        // D7：R7 守卫 —— sink 已设但 ws==nullptr 时**整帧不记录**
        if (sink_ != nullptr && ws != nullptr) {
            const OdometrySample smp{tick, dt, *ws, cmd,
                                     Twist{vx, vy, wz},   // D8：标定后的 twist
                                     p};
            sink_(sink_ctx_, smp);
        }
        return p;
    }

    // D4：内部只存**一个**连续 yaw；两个出口分工，不存第二个成员
    Pose  pose() const { return Pose{x_, y_, wrap_(yaw_)}; }   // 导航/上层：wrap
    float yaw_continuous() const { return yaw_; }              // 融合层：连续，多圈单调
    float yaw_ref() const { return yaw_continuous(); }         // 融合层校准源出口

private:
    // D5：wrap 到 (-π, π]。fmod 后补正，**不用 while 循环**（角度可能上百圈）
    static float wrap_(float y) {
        const float pi = 3.14159265358979323846f;
        const float tp = 6.28318530717958647692f;
        float r = std::fmod(y + pi, tp);
        if (r < 0.0f) { r += tp; }
        return r - pi;   // 本实现返回 [-π, π)；-π 与 +π 是同一个角，契约上等价
    }

    // D10：全部成员在声明处初始化
    float       x_ = 0.0f;
    float       y_ = 0.0f;
    float       yaw_ = 0.0f;          // 连续累计，不 wrap
    float       twist_scale_ = 1.0f;
    SampleSink  sink_ = nullptr;
    void*       sink_ctx_ = nullptr;
};
```

---

## 2. 决策点索引

| # | 决策点 | 定案 | 另一种写法会怎样 |
|---|---|---|---|
| **D1** | `Pose` 放哪 | `contracts.hpp` | 放 odometry.hpp → 融合层依赖倒挂 |
| **D2** | 积分顺序 | **先 yaw，后位移**（半隐式） | 抄 `simulation_demo.cpp`（显式欧拉）→ **实测 33 条 FAIL** |
| **D3** | scale 乘几个通道 | **三个**（vx/vy/wz） | 只乘 vx/vy → 测试 [8] 红（转角不缩放） |
| **D4** | yaw 存几个值 | **一个**连续值，出口处 wrap | 存两个成员 → 两者会不同步，测试 [6] 红 |
| **D5** | wrap 实现 | `fmod(y+π, 2π)` 补正 | `while` 循环 → 30 rad 要转 5 次、3000 rad 要 477 次（性能陷阱，功能上仍对） |
| **D6** | `reset()` 清什么 | **只清位姿** | 把 sink 也清了 → 测试 [9]/[10] 红 |
| **D7** | sink 记录时机 | 积分**之后**，`ws != nullptr` 才记 | 忘了判空 → 测试 [10] 红；记在积分前 → 样本里 pose 是上一帧 |
| **D8** | 样本 `twist_` 语义 | **标定后**（`Twist{vx,vy,wz}`） | 传 `body_vel` → 校准源对外报的是标定前的值，融合层会用错 |
| **D9** | 返回值 vs `pose()` | 同一个 `p` | 各算各的 → 可能不一致（设计 §5 明说必须一致） |
| **D10** | 成员初始化 | 声明处 `= 0.0f` / `= nullptr` | 漏初始化 → STAGE1 的 E5 坑（"未初始化成员"）重演 |

---

## 3. 分工判据（哪些建议自己敲）

> 判据：**这一处如果我写错了，测试会红吗？**
> 会红 → 有效练习，值得自己敲；不会红 → 抄了不产生理解，直接接受。

| 类别 | 决策点 | 建议 |
|---|---|---|
| **机械抄写**（敲了不产生理解） | D1 的 struct 字段、D10 的成员声明、函数签名 | **直接接受** |
| **判断点**（敲错必红） | **D2 积分顺序**、**D4 单/双成员**、**D5 wrap**、**D6 reset 保留什么**、**D7 守卫时机** | **自己重写**（挑 2~3 个） |
| **语义点**（不红但会埋雷） | **D3 scale 通道数**、**D8 样本 twist_ 语义**、**D9 一致** | 至少想清楚"为什么"，红不红是次要的 |

**推荐配额**：第一次写 odometry，**D2 + D5 + D6** 三处自己写。理由：
- D2 是**本组件最核心的判断**（也正是 `simulation_demo.cpp` 踩错的地方）
- D5 是**纯算法手活**（边界 + 性能，两个约束同时满足）
- D6 是**状态生命周期**判断（哪些该清、哪些不该清，这是有状态组件的通病）

---

## 4. 怎么跑

```bash
# 1) 你的 inc/odometry.hpp 一写出来，cmake 会自动挂上 test_odometry
cmake -S kinematics -B build && cmake --build build -j
ctest --test-dir build --output-on-failure

# 2) 想单独跑、看详细逐条输出
g++ -std=c++17 -Wall -Wextra -Werror -Iinc -Itest test/test_odometry.cpp -o /tmp/t_odo && /tmp/t_odo
```

`ctest` 会先打印 `-- odometry: 检测到 inc/odometry.hpp -> 已注册 test_odometry`。

**逐条进度条**：113 条 CHECK 里绿几条就是进度。分阶段目标：

| 写到哪 | 应该变绿的测试 |
|---|---|
| 只有 `Pose` + 空壳类 | 编译过，全红 |
| 加了状态 + `update()` 积分 | [1][2][7][8][12] 绿 |
| 加了 `pose()`/`yaw_continuous()`/`yaw_ref()` | [3][4][5][6] 绿 |
| 加了 sink | [9][10] 绿 → 113/113 |

---

## 5. 验收与处置（D 类规则）

- [ ] `ctest` → `test_odometry` **113/113 通过**
- [ ] `-Wall -Wextra -Werror` 零告警
- [ ] `kinematics/docs/IMPL.md` 补一段 odometry 的代码地图（**只加一屏，不要逐行复述**）
- [ ] `docs/TODO.md` 关闭 P15
- [ ] **本文件（`WORK_ODOMETRY.md`）移进 `trash/`** —— 施工单完成即失效，代码本体才是源

> 为什么必须移走：留着它就是**第二个代码源**。改了 `inc/odometry.hpp` 却忘了改这里，
> 下一个人（或三个月后的你）会照着过期的施工单干活。这正是 D 类存在的意义。
