---
class: work
generated: false
accepted: true
---
> **类：D 施工单（临时物）** —— 冻结 lunokhod 之前**只剩两件事要你手敲**，都在这页。
> **✅ 已验收（2026-09-17）→ 已归档 `../trash/`**（一次性）。设计权威：[`chassis_loop/docs/DESIGN.md`](../chassis_loop/docs/DESIGN.md) ·
> 待办源：[`TODO.md`](TODO.md)（P25 / P12）。
> **分工**：本页的 `inc/` 代码 **你手敲**；改名 / 加作用域 / 文档 / 测试 **我做**（本轮已做完的那批见 §4）。
> **位置写法**：只写符号名与可搜索的原文（不写行号）。

> ## ✅ 本施工单已闭合（2026-09-17）
> **P25** 用户手敲并已核（第一版多留一行 → 限幅每拍走两步，已修）；**P12** 由 AI 直接改 → **模板参数**根治。
> **谁敲的（备查）**：P25 = **用户手敲**（第一版多留一行 → 限幅每拍走两步 → 用户删）；
> P12 = **用户敲两版**（夹住 → 判无效）→ **AI 按授权重写成模板版** → **用户最后把 `kMinWheels` 改成 3**。
> 下面 §2 记的是**最终形态**；契约与决策已入 `DESIGN.md` / `kinematics/AGENTS.md` 账本，
> 验收记录见 `docs/log/HANDOFF.md` 的"冻结快照"。

# WORK_FREEZE.md — 冻结前收尾（P25 + P12）

## 0. 一句话

**装配层的限幅饱和标志（P25）** 与 **`OmniDrive` 的轮数校验（P12）** —— 两个都在你手里，
其余（命名空间 / effort 加口 / 改名 / cast / 文档 / 测试）**我今天已经做完并验收**（§4）。

---

## 1. `P25` 装配层暴露限幅饱和标志 —— `chassis_loop/inc/chassis_loop.hpp`，3 处

**为什么**：`TwistAccLimiter::limit()` 已经把 `LimitResult` 交出来了（含 `is_vx_lim_` / `is_vy_lim_` /
`is_wz_lim_` 三个饱和标志），但装配层只取了 `.out_` 就把它扔掉 —— 上层于是**无法知道"我的指令被削过"**。
这就是 `TODO.md` P19 §7「执行器故障 / 轮间一致性」要的原料之一。

### 1.1 加一个成员（放在 `Twist t_cmd_final_` 旁边）

```cpp
    twist_acc_limiter::LimitResult lim_res_{};      ///< latest limiter result (含三个饱和标志)
```

> ⚠ **命名空间（2026-09-17 已全仓落地）**：本文件都在 `namespace lunokhod::chassis_loop` 里，
> 所以这里写**兄弟子命名空间** `twist_acc_limiter::LimitResult`（= `lunokhod::twist_acc_limiter::LimitResult`）；
> 而 `Twist` / `Pose` / `WheelSpeeds` 在根，裸名即可。详见 [`../AGENTS.md`](../AGENTS.md) §3.1。

### 1.2 改 `tick()` 里那一行（原文 → 新文）

```cpp
// 改前
        // --- limiter ---
        t_cmd_final_ = limiter_.limit(t_cmd_in_, dt).out_;

// 改后
        // --- limiter ---
        lim_res_ = limiter_.limit(t_cmd_in_, dt);   // 三个饱和标志留给上层看
        t_cmd_final_ = lim_res_.out_;
```

### 1.3 加只读口（放在 `cmd()` 之后）

```cpp
    twist_acc_limiter::LimitResult limit_result() const { return lim_res_; }   // 上一拍 tick() 的限幅结果
```

**顺手删一行残留注释**：`tick()` 里 `// --- process to res残差 ---` 那行是笔误留下的，删掉。

**注意事项**
- `LimitResult` 由 `twist_acc_limiter.hpp` 提供，装配层**已经 include 了它**，不用加头文件。
- 语义边界：**只有 `tick()` 跑过之后才有意义**（构造后 = 三个 flag 全 false、`out_` 全 0），和 `wheel_effort(i)` 同一性质。
- 别把它做成"报警逻辑" —— 库**只交事实**（`AGENTS.md` §5.3）：饱和了要不要报警、要不要降速，由上层决定。

**我随后会加的测试（你不用写）**：`chassis_loop/test/test_chassis_loop.cpp` 新增一组 ——
① 阶跃指令第一拍 `is_vx_lim_ == true` 且 `out_.vx_` = 手算值（`acc_vx × dt`）；
② 指令爬满之后 `is_vx_lim_ == false`；
③ 一致性：`limit_result().out_` 与 `cmd()` **逐位相等**（两条独立路径给同一个事实）。

---

## 2. `P12` `OmniDrive` 轮数 → ✅ **已根治（AI 改，2026-09-17）**

**定案：轮数提成模板参数**（三轮迭代的结果，过程见 [`../kinematics/AGENTS.md`](../kinematics/AGENTS.md) 账本第一行）：

```cpp
template <uint8_t N>
class OmniDrive : public Kinematics<OmniDrive<N>> {
public:
    static constexpr uint8_t kMaxWheels = 6;    // 上界 = WheelSpeeds::values_ 的容量
    static constexpr uint8_t kMinWheels = 3;    // 下界 3：N=3 是全向可解的最小值（N=2 时 J 的秩只有 2，vx 不可控）
    static_assert(N >= kMinWheels && N <= kMaxWheels, "OmniDrive: 轮数必须 3..6");

    OmniDrive(float cr, float gamma, float wr) : cr_(cr), gamma_(gamma), wr_(wr) {}
    uint8_t wheel_count() const { return N; }

    WheelSpeeds inverse_impl(const Twist& t_cmd) const {
        float J[N][3];                       // ← 数组界 = 循环界 = 契约容量，同一个 N
        for (uint8_t i = 0; i < N; ++i) { /* ... */ }
        return jacobian_apply(J, N, t_cmd);
    }
    // forward_impl 同理（n = N，无运行期轮数）
private:
    float cr_, gamma_, wr_;                  // wn_ 整个消失
};
```

**为什么它比前两版好**

| 版本 | 非法轮数的后果 | 判据 |
|---|---|---|
| 夹到 [2,6] | **静默按 6 轮建模**（算错但「看着在跑」） | ❌ 洗白数据（铁律 §2.5） |
| 判无效 `is_valid()` | 车不动（安全方向），但**要调用方自觉查** | ❌ 靠人品 |
| **模板参数** | **写不出来**（编译期） | ✅ 不依赖任何人的记性 |

**机器验证（新增，改坏必红）**：`kinematics/test/compile_fail/omni_over_capacity.cpp` 故意写 `OmniDrive<7>`；
`kinematics/CMakeLists.txt` 用 `try_compile` 检查它**必须编不过** —— 它若编过（守卫被删/放宽），
**cmake 配置阶段直接 FATAL_ERROR**（本地与 CI 一起红）。已实测：删掉 `static_assert` → 配置立刻失败。

**调用点（8 处，AI 已改）**：`OmniDrive omni(3, 0.15f, 0.0f, 0.03f)` → `OmniDrive<3> omni(0.15f, 0.0f, 0.03f)`；
`WheelLoop<OmniDrive>` → `WheelLoop<OmniDrive<3>>`（装配层两个边界用例）。

## 3. 你敲完我跑什么（一次说清）

1. 四档编译：Debug · Release · 严格（`-Wconversion -Wshadow -pedantic`）· ASan+UBSan —— **零告警**
2. 全量 `ctest`（聚合 + 各组件独立）+ 消费模式（库模式零泄漏）
3. §1.3 / §2.2 那两组新测试落地 → 重跑**变异审计**（往新加的口上打变异，必须变红）
4. **行为锚点**：KND 隔离副本仿真输出**逐位比对**（期望 `md5 9ab64f43…`）
5. `python3 scripts/ci_local.py --clean` + 文档门禁

## 4. 本轮我已经做完的（**不用你碰**）

| 项 | 内容 |
|---|---|
| **P29** | `wheel` 的 15 个全局名收进 `namespace wheel`（4 个转发头 + `wheel.hpp`/`wheel.cpp`），全仓调用点跟上，转发头里**过期的注释**一并改 |
| **P26** | 执行器组契约加**第 5 个方法** `effort(i)`（决策 **D13**）：`wheel::Wheel::effort()` → `WheelSet::effort(i)` → `ChassisLoop::wheel_effort(i)`；`SetPwmFn`→`SetEffortFn`、`set_pwm_`→`set_effort_`、`out_pwm`→`out_effort` 等命名中性化 |
| **P23** | `float → int16_t` 全部显式 `static_cast`（`-Wconversion` 严格档全仓零告警，本条关闭） |
| **P32** | **全仓命名空间**（你拍板 A 案 + 数据放根）：`lunokhod::` 根 + `lunokhod::<组件>`；10 个库文件 + 全部消费方 + CI 示例 + 接入指南；规则进 `AGENTS.md` §3.1 |
| 文档 | `DESIGN`（§5 接口块 / §5.2 契约表 / §8.2 **D13**）· `IMPL`（代码地图）· `INTEGRATION`（§5.2 单轮只读口 + 示例跟上 P29）· `TODO`（P23/P26/P29 收口）· `wheel` 教训日志 · `HANDOFF` |
| 验收 | 聚合 9/9 · 四档零告警 · **15/15 变异变红** · 消费模式零泄漏 · 接入指南示例走 CMake 链真编真跑 · **KND 仿真输出逐位不变** |
