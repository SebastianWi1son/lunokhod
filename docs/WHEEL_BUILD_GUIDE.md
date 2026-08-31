# Wheel 构建指导（控制系统第三块积木）

> 日期：2026-08-22
> 定位：kinematics（✅）→ TwistAccLimiter（✅）→ **Wheel** → LineFollower
> 素材：legacy `Lib/control/wheel/wheel.c` + `Lib/algorithm/{dsp,pid}`（C 代码，需移植）
> **开发工作流：SemVer + 渐进稳定分支 → 见 `control/wheel/docs/GIT_BRANCHING.md`（跟着走就是完整 git branching 流程）**

## 0. 一句话

**Wheel = 单轮的"执行层"**：测速回调 → S 曲线规划（Jerk 平滑）→ 速度环 PID → 输出回调。轮子数 = Wheel 实例数（diff 2 个、mecanum 4 个、omni 3 个）。

## 1. 定位（控制链第三层，独立组件）

```
TwistAccLimiter（Twist 空间斜坡）→ inverse（kinematics）→ 每轮 target_speed
                                                        ↓
                                          Wheel（轮速空间 S 曲线 + PID）← 本组件
                                                        ↓
                                                  电机/PWM
```

- **输入**：target_speed（标量，单轮）——**不依赖 kinematics**（Twist 是上层的事），零依赖组件
- **与 TwistAccLimiter 的关系**：原 SpeedLimiter"三级愿景"被拆分——加速度斜坡留在上层，**Jerk/S 曲线下沉到这里**（legacy dsp_traj 就是 S 曲线）
- **硬件解耦**：测速/输出全部走回调，组件本身零硬件依赖（与 kinematics 同哲学）

## 2. 移植范围（3 个组件 + 1 个轮子）

legacy 依赖链：`wheel.c` → `dsp.h`（LPF/Ramp/S曲线）+ `pid.h`（工业级 PID）

| legacy | 本质 | C++ 类名候选 |
|---|---|---|
| `dsp_lpf_t` | 一阶低通（alpha = dt/(Tf+dt)） | `Lpf` / `FirstOrderFilter` |
| `dsp_ramp_t` | 斜率限幅（与 TwistAccLimiter::ramp 同类！） | `Ramp` / `SlewLimiter` |
| `dsp_traj_t` | S 曲线 = ramp + 两级 LPF 级联 | `SmoothPlanner` / `TrajPlanner` |
| `pid_instance_t` | 工业级 PID（微分先行/梯形积分/积分分离/抗饱和） | `Pid` / `PidController` |
| `wheel_t` | 组合：回调 + 规划器 + PID | `Wheel` |

**重要观察**：`dsp_ramp_t` 的算法和 TwistAccLimiter 的 `ramp()` **完全相同**（max_step = rate·dt + clamp）。统一时机到了——要么 Wheel 复用（提取公共实现），要么承认两处重复。**建议：提取 `inc/ramp.hpp` 独立类，TwistAccLimiter 和 Wheel 共用**（DRY，且是命名评估的好素材）。

## 3. 项目结构（镜像 twist_acc_limiter）

```
lunokhod/control/wheel/
├── CMakeLists.txt
├── inc/
│   ├── lpf.hpp            ← 一阶低通（声明）
│   ├── ramp.hpp           ← 斜率限幅（声明；TwistAccLimiter 未来共用）
│   ├── smooth_planner.hpp ← S 曲线规划器（ramp + 2×LPF 组合）
│   ├── pid.hpp            ← 工业级 PID（声明）
│   └── wheel.hpp          ← Wheel 组合类（声明）
├── src/                   ← 各 .cpp 实现（声明/定义分离）
├── test/test_wheel.cpp    ← 锚点测试
└── examples/example_wheel.cpp  ← 链式 demo（含电机模型仿真）
```

## 4. 组件设计（伪代码 + 锚点）

### 4.1 Lpf（一阶低通）

```cpp
// inc/lpf.hpp
class Lpf {
public:
    Lpf(float Tf) : Tf_(Tf), prev_(0.0f) {}
    float calc(float raw, float dt) {          // dt<=0 直通
        float alpha = dt / (Tf_ + dt);
        prev_ = alpha * raw + (1.0f - alpha) * prev_;
        return prev_;
    }
    void reset() { prev_ = 0.0f; }
private:
    float Tf_, prev_;
};
```

**锚点**：Tf=0.02, dt=0.01 → alpha=1/3；阶跃 1.0 → 0.3333, 0.5556, 0.7037, 0.8025...（收敛 1.0，无超调）

### 4.2 Ramp（斜率限幅——与 TwistAccLimiter::ramp 算法相同）

```cpp
// inc/ramp.hpp
class Ramp {
public:
    Ramp(float max_rate) : max_rate_(max_rate), prev_(0.0f) {}
    float calc(float target, float dt) {       // dt<=0 直通
        float step = max_rate_ * dt;
        float out = target;
        if (target > prev_ + step)      out = prev_ + step;
        else if (target < prev_ - step) out = prev_ - step;
        prev_ = out;      // ⚠️ 状态写回铁律（见下）
        return out;       // 区间含端点语义
    }
    void reset() { prev_ = 0.0f; }
private:
    float max_rate_, prev_;
};
```

**踩坑实录（2026-08-22）**：初版用 `return prev_ + step` 提前退出，忘记写回 `prev_` —— 状态永远停在 0，输出恒为 step，永不爬升。有状态组件铁律：**先算 out → 统一写回 → 最后 return**（与 TwistAccLimiter E1 同类坑，一坑两踩）。

**锚点**：max_rate=1500 rpm/s, dt=0.02 → step=30；0→1000 rpm 需要 34 帧 = 0.68s（已锁定）

### 4.3 SmoothPlanner（S 曲线 = ramp + 2×LPF 级联）

```cpp
// inc/smooth_planner.hpp —— 组合（不是继承）
class SmoothPlanner {
public:
    SmoothPlanner(float max_rate, float Tf)
        : ramp_(max_rate), f1_(Tf), f2_(Tf) {}
    float calc(float target, float dt) {
        float ramped = ramp_.calc(target, dt);
        return f2_.calc(f1_.calc(ramped, dt), dt);   // 两级 LPF 串联
    }
    void reset() { ramp_.reset(); f1_.reset(); f2_.reset(); }
private:
    Ramp ramp_;
    Lpf  f1_, f2_;
};
```

**锚点**：比纯 ramp 更慢到达（两级 LPF 拖尾），但无超调（LPF 单调收敛）；终值收敛 1000

### 4.4 PID（工业级，5 段流程）

> **配置方案定案（2026-08-22）**：弃用 9 个位置参数（C 时代痛点：只能按顺序猜着填），改 **Config struct + 默认值**。
> 评估结论：C++20 designated initializer / builder 链式对 8 个独立数值并无额外收益（builder 的价值在参数间有依赖关系时），config 填表最清晰。
> 防误改三层保险：① 默认值消灭"复制整表"（调用处只出现显式改的字段）② 预置语义常量 = 唯一真相 ③ 测试锚点测行为不测数值。
> **分层（2026-08-22 补充）**：`PIDConfig` 默认值保持 0 中立（0 = disabled 语义跟随 legacy：max_rate 0 关斜坡、thresh 0 关分离）；具体轮速环数值（3600/1200/20/10000/0.005）是应用层知识，放**调用处/preset**，不进通用层。

```cpp
// inc/pid.hpp
struct PIDConfig {
    float kp_ = 0.0f;                 // 0 = 中立（调用方必须配 kp/ki/limit 等真参数）
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float limit_out_ = 0.0f;          // 硬钳位，无 0=关闭语义（0 就是钳到 0）
    float limit_i_ = 0.0f;            // 积分抗饱和钳位，同上
    float thresh_i_sep_ = 0.0f;       // ≤0 = 积分分离关闭（0 = disabled）
    float max_rate_out_ = 0.0f;       // ≤0 = 输出斜坡关闭（0 = disabled）
    float d_filter_Tf_ = 0.0f;        // D 项低通时间常数
};

class PID {
public:
    explicit PID(const PIDConfig &cfg) : cfg_(cfg) {}
    float calc(float target, float measure, float dt);
    void reset();
private:
    PIDConfig cfg_;                  // 配置存下来（可 getter 读）
    // 状态：integral_, error_prev_, measure_prev_, d_filter_(Lpf), ramp_out_(Ramp)
};
```

调用处（只改差异，不复制整表）——**预置常量放应用层，不放 pid.hpp**：
```cpp
namespace wheel_config {                     // 应用层：轮速环出厂值
    inline constexpr PIDConfig speed_loop = [] {
        PIDConfig c;
        c.kp_ = 3.0f;  c.ki_ = 1.0f;  c.kd_ = 0.0f;
        c.limit_out_ = 3600.0f;  c.limit_i_ = 1200.0f;
        c.thresh_i_sep_ = 20.0f;  c.max_rate_out_ = 10000.0f;
        c.d_filter_Tf_ = 0.005f;
        return c;
    }();
}

PIDConfig cfg = wheel_config::speed_loop;   // 拿到出厂值
cfg.kd_ = 0.2f;                             // 就改这一个字段
PID pid(cfg);
```

calc 流程（与 legacy 一致）：
```
1. dt 防御：dt<=0 或 dt>0.5 → dt=0.001
2. error = target − measure
3. P = kp·error
4. I = 梯形积分(ki·dt·(error+error_prev)/2) → 抗饱和钳位(±limit_integral)
   → 积分分离：|error|>thresh 时冻结积分（thresh<=0 关闭）
5. D = 微分先行(−kd·(measure−measure_prev)/dt) → 低通滤波(d_filter)
6. out = P+I+D → 钳位(±limit_out) → 输出斜坡(output_ramp，0=关闭)
7. 状态更新 error_prev/measure_prev
```

**锚点**：kp=1, ki=kd=0 → out=error；纯积分：error 恒定 10, ki=1, dt=0.01 → 每帧积分 +0.1×...（梯形）；微分先行：measure 突变不产生 D 项冲击

### 4.5 Wheel（组合 + 回调）

> **单位约定（2026-08-22 定案）**：本库所有速度数值**单位无关**——算法是纯数值运算（LPF/Ramp/SmoothPlanner/PID 不关心 1.0 是 rpm 还是 rad/s），单位由调用方约定。
> 命名一律中性（`MeasureSpeedFn`、`target_speed_`），不绑 rpm。参考 legacy 的单位：轮速 rpm、PWM 占空比标量（int16）。
> **dt 归属（2026-08-22 定案）**：dt = 每次 update 的**实测间隔**（算法输入），不是板级配置死值（设计值）——M 法测速公式 `rpm = Δ脉冲/PPR × (1/dt) × 60` 数学上必需；中断延迟/任务抖动会让实际间隔 ≠ 配置值，用配置死值积分会误差累积。所以 dt 穿透回调（`MeasureSpeedFn(motor_id, dt)`），由 Wheel 作为时间基准提供者。
> **单位验证检查（每次发布必做）**：锚点测试标注参考单位；跨单位使用时（如 rad/s），必须写单位换算验证测试（见 §8 验收标准第 5 条），防止"单位错了编译照过、行为全错"的隐蔽失效。

```cpp
// inc/wheel.hpp
// 回调类型（嵌入式：函数指针，零开销；见 §5 决策）
using MeasureSpeedFn = float (*)(uint8_t motor_id, float dt);   // 测速：采样+换算（M 法）；数值单位由调用方约定
using SetPwmFn   = void  (*)(uint8_t motor_id, int16_t pwm); // 输出：PWM 占空比标量

// 规划器配置（SmoothPlanner 的参数聚合成块，默认 = legacy 实测值）
struct SmoothPlannerConfig {
    float max_rate = 1500.0f;   // 转速变化率上限（单位/秒，参考 rpm/s）
    float Tf       = 0.02f;     // 滤波时间常数（秒）——与 LPF 的 Tf 同量纲
};

class Wheel {
public:
    // 按组件边界聚合：SmoothPlanner 一块、PID 一块，motor_id/回调是绑定与运行时依赖，不入 config
    Wheel(uint8_t motor_id,
          const SmoothPlannerConfig &planner,
          const PIDConfig &pid,
          MeasureSpeedFn measure_speed, SetPwmFn set_pwm);
    void set_target(float target_speed);
    void update(float dt);     // 测速 → 规划 → PID → 输出
    void stop();               // 清目标 + 复位 + 急停 PWM=0
    float current_speed() const;
private:
    uint8_t motor_id_;
    float   target_speed_, current_speed_;
    SmoothPlanner planner_;    // 轮速空间 S 曲线（Jerk 平滑）
    PID           pid_;        // 速度环
    MeasureSpeedFn measure_speed_;
    SetPwmFn set_pwm_;
};
```

调用处（config 只改差异，不复制整表）：
```cpp
SmoothPlannerConfig pc;   // 默认 = legacy 实测值（1500 / 0.02）
PIDConfig cfg;            // 默认 = 轮速环出厂值
cfg.kd_ = 0.2f;           // 就改这一个字段
Wheel w(0, pc, cfg, encoder_get_rpm, motor_set_pwm);
```

update 流程（legacy 原样）：
```
1. current_speed_ = measure_speed_(motor_id_, dt)   // 测速回调
2. planned = planner_.calc(target_speed_, dt)     // S 曲线规划
3. pwm = pid_.calc(planned, current_speed_, dt)   // 速度环
4. set_pwm_(motor_id_, (int16_t)pwm)              // 输出回调
```

**参数（legacy 实测值）**：max_rate=1500 rpm/s, Tf=0.02, limit_out=3600, limit_integral=1200, sep_thresh=20, output_max_rate=10000, d_filter_Tf=0.005

## 5. 回调设计决策（4 选 1，你来评）

| 方案 | 开销 | 灵活度 | 嵌入式适配 |
|---|---|---|---|
| **函数指针**（legacy 原样） | 零 | 低（无捕获） | ✅ 最常用 |
| `std::function` | 类型擦除（通常无堆分配，但有开销） | 高（可捕获 lambda） | ⚠️ STM32 慎重 |
| 模板回调 `Wheel<GetFn,SetFn>` | 零 | 高 | ✅ 零开销但类型复杂 |
| 虚接口 `class ICallback` | 虚调用一次 | 中 | ⚠️ 不必要 |

**判断标准**：真机回调来自电机库（`encoder_get_rpm`/`motor_set_pwm`，无捕获需求）→ **函数指针**够了；若测试里想用 lambda 模拟，测试文件可以直接写普通函数。**v1 用函数指针**（与 legacy 一致，零开销），模板化留给将来"真有必要捕获"时。

**决策定案（2026-08-22）**：函数指针 + motor_id 参数（Wheel 构造时绑定，update 时传给回调——函数指针无法捕获状态，id 只能走参数通道）+ `MeasureSpeedFn`/`SetPwmFn` 命名（中性，不绑 rpm 单位；measure = 采样+计算，区别于 read/get 的读缓存语义）。

**升级触发器**（什么情况才放弃函数指针）：① 回调必须携带状态且类型编译期已知 → 模板回调；② 回调运行时才确定 → `std::function`；③ 多个回调对象层次结构 → 虚接口。现在三个都不触发。

## 6. CMake（STATIC，镜像 twist_acc_limiter）

```cmake
cmake_minimum_required(VERSION 3.28)
project(lunokhod_wheel CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 库 STATIC: 声明/定义分离
add_library(wheel STATIC src/lpf.cpp src/ramp.cpp src/smooth_planner.cpp
                       src/pid.cpp src/wheel.cpp)
target_include_directories(wheel PUBLIC ${PROJECT_SOURCE_DIR}/inc)
target_compile_options(wheel PRIVATE -Wall -Wextra -Werror)

# 测试
add_executable(test_wheel test/test_wheel.cpp)
target_link_libraries(test_wheel PRIVATE wheel)
target_compile_options(test_wheel PRIVATE -Wall -Wextra -Werror)

# 示例（链式：含电机模型仿真）
add_executable(example_wheel examples/example_wheel.cpp)
target_link_libraries(example_wheel PRIVATE wheel)
target_compile_options(example_wheel PRIVATE -Wall -Wextra -Werror)
```

**不需要依赖 kinematics**（Wheel 与 Twist 无关）——除非链式 demo 想展示全链路（限幅→逆解→wheel），那种情况 demo 里临时加 include 路径即可，库本身保持零依赖。

## 7. 构建步骤（命令）

```bash
# 1. 建工程目录（C 代码留在 lunokhod/wheel/ 作参考，或移进 control/wheel/legacy/）
mkdir -p /home/wilson/Dev/Workspace/lunokhod/control/wheel/{inc,src,test,examples}

# 2. 按 §4 写组件（LPF → Ramp → SmoothPlanner → PID → Wheel 顺序，逐层验证）
#    Ramp 建议先提取为公共组件（TwistAccLimiter 未来共用）

# 3. 写测试（锚点见 §4 各组件 + 集成）

# 4. 构建
cd /home/wilson/Dev/Workspace/lunokhod/control/wheel
cmake -B build -S .
cmake --build build
./build/test_wheel          # ALL PASS，退出码 0

# 5. 命令行快速验证（测试开发期）
g++ -std=c++17 -Wall -Wextra -Werror -Iinc test/test_wheel.cpp \
    src/*.cpp -o /tmp/tw && /tmp/tw
```

## 8. 验收标准

1. 五组件各自锚点 PASS + 集成测试 PASS，退出码 0，-Werror 零警告
2. Ramp 已提取为公共组件（TwistAccLimiter 未来切换共用）
3. 回调函数指针（`MeasureSpeedFn`/`SetPwmFn`），零硬件依赖，可拖入 STM32 工程
4. example 含**电机模型闭环仿真**（见下，L3 级验证）
5. **单位验证**（每次发布必做）：锚点标注参考单位（legacy 为 rpm）；任何跨单位使用（rad/s、m/s、编码器计数）必须写单位换算验证测试——单位错了编译照过、行为全错，这是最隐蔽的失效方式，必须由测试钉死

## 9. 电机模型仿真（example 的"看得见"部分）

Wheel 是执行层，纯单元测试看不到"速度是否收敛"。example 里给一个**一阶惯性电机模型**（单位 = rpm，参考单位；Wheel 接口本身单位无关）：

```cpp
// 电机模型：PWM → rpm 一阶惯性（模拟真实电机动态）
struct MotorModel {
    float rpm = 0.0f;
    float T = 0.1f;                    // 电机时间常数 100ms
    float step(float pwm, float dt) {  // 满 PWM=3600 → 目标 3000rpm
        float target = pwm / 3600.0f * 3000.0f;
        rpm += (target - rpm) * dt / (T + dt);
        return rpm;
    }
};
```

闭环 demo：`Wheel(target=1000) → update → measure_speed 回调读 MotorModel → set_pwm 喂回 MotorModel`
观察：rpm 爬坡收敛到 1000（S 曲线 + PID 双平滑），无超调/小超调，PWM 不振荡。**锚点：0.7s 内收敛 ±5%，稳态无振荡（单位：rpm，参考 legacy）。**

## 10. 命名评估（已定案，2026-08-22）

| 组件 | 定案 | 依据 |
|---|---|---|
| 低通 | `LPF` | 领域缩写直白（全大写避免与 lpf 混淆） |
| 斜坡 | `Ramp` | 短、表意清晰；与 TwistAccLimiter::ramp 算法相同（跨项目统一待定） |
| S 曲线 | `SmoothPlanner` | 表意"平滑"；SCurve 像类型缩写 |
| PID | `PID`（配 `PIDConfig`） | Controller 无额外信息增量；Config 默认 0 中立，preset 放应用层（见 §4.4） |
| 回调 | `MeasureSpeedFn`/`SetPwmFn` | 函数指针 + motor_id 参数；中性命名不绑 rpm（单位由调用方约定，见 §4.5）；measure = 采样+换算，区别 read/get |
| 单位 | 接口中性 + 文档声明 | 算法是纯数值运算，单位是调用方语义；命名不撒谎，单位靠锚点钉死（见 §8.5） |

**拼写教训（2026-08-22）**：`clac` vs `calc` 手滑——API 名拼错是硬伤（其他组件都叫 calc，唯独它 clac）。写完头文件后 grep 一遍全库命名一致性。
