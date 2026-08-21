# Stage 1 复盘（types + differential_drive）

> 阶段 1 完成于差分底盘：CRTP 结构 + 测试验证跑通。
> 本文按点记录本阶段的**错误与正确示例**、**讨论收获**，供阶段 2（mecanum + omni）开工前回看。

---

## 一、错误清单（按点，含正确示例）

### E1. 函数参数没写类型
```cpp
// ❌ 编译器不知道 cmd 是什么
void inverse(cmd) { ... }
// ✅ 完整签名：返回类型 + 参数类型 + const
WheelSpeeds inverse(const Twist& cmd) const { ... }
```
教训：C++ 函数签名必须完整。模板基类里尤其容易漏，因为"看起来是接口"。

### E2. 继承语法错误
```cpp
// ❌ 冒号位置错 + 缺 CRTP 模板参数
class DifferentialDrive public: Chassis { ... }
// ✅ 冒号在前；CRTP 必须传模板参数（告诉基类"孩子是谁"）
class DifferentialDrive : public Chassis<DifferentialDrive> { ... }
```

### E3. 返回类型与函数体矛盾
```cpp
// ❌ 签名 void，函数体却 return 值
void inverse_impl(const Twist &cmd) { ... return ws_; }
// ✅ 签名 WheelSpeeds
WheelSpeeds inverse_impl(const Twist& cmd) const { ... return out; }
```

### E4. 成员缓存替代局部返回
```cpp
// ❌ 成员缓存：不可重入（ISR 打断会互相踩）、破坏 const、常驻内存
WheelSpeeds ws_; Twist t_;   // 成员
// ✅ 局部变量返回：纯函数、可重入、零常驻内存
WheelSpeeds out; ... return out;
```
规律：**纯数学函数永远局部变量返回；只有有状态的对象（如未来的 SpeedLimiter）才用成员**。

### E5. 未初始化成员
```cpp
// ❌ 无构造函数 → w_/r_ 是垃圾值（-Wall 会警告）
// ✅ 构造函数初始化列表
DifferentialDrive(float wheelbase, float wheel_radius)
    : wb_(wheelbase), r_(wheel_radius) {}
```

### E6. 拼写 / 命名不一致
```cpp
WheelSpeed ws_;   // ❌ WheelSpeeds 拼错
// ❌ types.hpp 定义 values_/count_，实现里写 values/count → 编译错
// ✅ 全库统一一套字段名
```

### E7. const 不一致（隐藏炸弹，实例化时才炸）
```cpp
// ❌ 基类 const 方法内 static_cast<const Derived*>，调用非 const 的 impl
//    → 实例化时报 "passing 'const DifferentialDrive' as 'this' discards qualifiers"
WheelSpeeds inverse_impl(const Twist& cmd) const { ... }   // ✅ 接口层/实现层全 const
```
教训：CRTP 基类是 const（纯函数承诺），**派生实现必须 const 跟上**——这同时也是 CRTP 强制接口约束一致性的体现。

### E8. 入口文件名不一致
```cpp
#include "kinematics.hpp"   // ❌ 实际文件叫 kinematic.hpp（缺 s）
// ✅ 统一为 kinematics.hpp（与 DESIGN.md 一致）
```

### E9. CMake：两个 main 塞进一个 target
```cmake
# ❌ 两个 .cpp 都有 main() → 链接报 multiple definition
add_executable(app test/test_kinematics.cpp examples/differential_drive_example.cpp)
# ✅ header-only 库标准结构：INTERFACE 库 + 每个 main 一个 target
add_library(kinematics INTERFACE)
target_include_directories(kinematics INTERFACE ${PROJECT_SOURCE_DIR}/inc)
add_executable(test_kinematics test/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE kinematics)
add_executable(example_diff examples/differential_drive_example.cpp)
target_link_libraries(example_diff PRIVATE kinematics)
```

### E10. 模板未实例化 = 假绿（本阶段最重要的一课）
```cpp
// ❌ main 是空的 → Chassis<DifferentialDrive> 从未被实例化
//    → 模板深层错误（E7 等）全部隐藏，编译"全绿"是假绿
// ✅ 测试必须真正调用库：模板的错误在实例化点（调用处）爆发
DifferentialDrive chassis(0.16f, 0.03f);
auto ws = chassis.inverse_calc({0.5f, 0.0f, 1.0f});   // ← 触发实例化
```
报错信息里 `required from here: ...` 指向的就是调用处——**模板报错永远在调用点爆发，这是常态不是 bug**。

---

## 二、正确示例（阶段 1 最终形状）

```cpp
// inc/types.hpp —— 数据层：对外契约，POD
#pragma once
#include <cstdint>

struct Twist {          // 输入：本体系速度
    float vx_;
    float vy_;
    float wz_;
};
struct WheelSpeeds {    // 输出：各轮角速度
    uint8_t count_;
    float values_[6];
};

// inc/drive_differential.hpp —— 行为层：CRTP 统一接口 + 差速实现
#pragma once
#include "types.hpp"

template<typename Derived>
class Chassis {
public:
    WheelSpeeds inverse_kinematics(const Twist& cmd) const {
        return static_cast<const Derived*>(this)->inverse_impl(cmd);
    }
    Twist forward_kinematics(const WheelSpeeds& fb) const {
        return static_cast<const Derived*>(this)->forward_impl(fb);
    }
};

class DifferentialDrive : public Chassis<DifferentialDrive> {
public:
    DifferentialDrive(float wheelbase, float wheel_radius)
        : wb_(wheelbase), r_(wheel_radius) {}

    WheelSpeeds inverse_impl(const Twist& cmd) const {
        WheelSpeeds out;
        out.count_ = 2;
        out.values_[0] = (cmd.vx_ - cmd.wz_ * wb_ / 2.0f) / r_;   // Left
        out.values_[1] = (cmd.vx_ + cmd.wz_ * wb_ / 2.0f) / r_;   // Right
        return out;
    }
    Twist forward_impl(const WheelSpeeds& fb) const {
        Twist t;
        t.vx_ = r_ * (fb.values_[1] + fb.values_[0]) / 2.0f;
        t.wz_ = r_ * (fb.values_[1] - fb.values_[0]) / wb_;
        t.vy_ = 0.0f;                                             // 非完整约束
        return t;
    }
private:
    float wb_;   // wheelbase (m)
    float r_;    // wheel radius (m)
};
```

```cpp
// test/test_kinematics.cpp —— 测试驱动骨架（锚点 + 互逆 + 边界）
#include "kinematics.hpp"
#include <cstdio>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)
static bool close(float a, float b) { return (a - b < 1e-5f) && (b - a < 1e-5f); }

int main() {
    DifferentialDrive chassis(0.16f, 0.03f);
    // 锚点：直行 0.5/0.03 = 16.667（手算，独立渠道）
    auto s1 = chassis.inverse_kinematics({0.5f, 0.0f, 0.0f});
    CHECK(close(s1.values_[0], 16.6667f));
    CHECK(close(s1.values_[1], 16.6667f));
    // 锚点：原地转 ±0.16/(2×0.03) = ±2.667
    auto s2 = chassis.inverse_kinematics({0.0f, 0.0f, 1.0f});
    CHECK(close(s2.values_[0], -2.6667f));
    CHECK(close(s2.values_[1], +2.6667f));
    // 往返：forward(inverse(x)) ≈ x
    Twist cmd{0.5f, 0.0f, 1.0f};
    auto back = chassis.forward_kinematics(chassis.inverse_kinematics(cmd));
    CHECK(close(back.vx_, 0.5f) && close(back.wz_, 1.0f) && close(back.vy_, 0.0f));
    // 边界：零输入 → 全零
    auto s0 = chassis.inverse_kinematics({0.0f, 0.0f, 0.0f});
    CHECK(close(s0.values_[0], 0.0f) && close(s0.values_[1], 0.0f));

    if (fails == 0) printf("ALL PASS\n");
    return fails;   // 退出码 = 结果：0 = 全绿
}
```

---

## 三、讨论收获（阶段 1 沉淀）

### 1. 测试方法论（通用，放之四海）
```
三类用例，各防一种错：
  对答案（锚点，手算特殊值） → 防系统性错误（公式整体错）
  走来回（互逆/守恒断言）    → 防局部错误（符号/系数错），覆盖面大
  虐极端（零/极值/非法值）   → 防特例错误（除零、未初始化）
铁律：期望值必须来自独立渠道（物理意义/权威文档），不能来自被测代码本身
      —— 否则程序验证自己 = 同义反复（互逆测试防不了"来回都错同一段"）
```

### 2. 命名评估框架（已写入 AGENT.md，新名字出现时按此评估）
```
① 信息增量：后缀必须添加新信息，否则删除（inverse_calc 冗余 → 删）
② 领域术语优先：IK/FK 等标准术语 > 自造词
③ 一层一个后缀：接口层无后缀，实现层 _impl，不混搭
```

### 3. struct vs class：行为与数据分离
- struct = 纯数据契约（POD）：无不变量、聚合初始化、可 constexpr/memcpy——**必须暴露**（接口签名就是它们）
- 封装（private+getter）只服务于"有内部状态约束要保护"的类型
- "API 安全"由接口层强类型保证（inverse 只接受 Twist），不靠类型内部封装

### 4. CMake header-only 结构 + CLion 索引原理
- INTERFACE 库 + 每 main 一 target；头文件挂 target 后 CLion 才有语义分析（高亮/补全/纠错）
- 改 CMakeLists 后必须 Reload CMake Project
- 严重语法错误会让解析器放弃整个文件 → 白色纯文本

### 5. cstdio vs iostream
- 主机测试两者皆可；printf 胜在嵌入式一致性（MCU 串口重定向是事实标准）+ 宏里短
- 选择标准：全库统一

### 6. 数学学习路线（用啥学啥）
```
写 jacobian_apply（阶段 2） → 补矩阵乘法（3Blue1Brown 线性代数的本质 前 4 集）
写全向轮 → 补弧度制/三角
写 forward 伪逆 → 补 2×2/3×3 矩阵求逆
写 foc → 补欧拉公式/复数旋转、导数积分（PID）
```
原则：按写到的代码学，不按数学书顺序学。

### 7. 模板实例化心智模型
```
模板未调用 = 未真正编译（只查语法不查语义）
模板错误在实例化点（调用处）爆发：报错信息找 "required from here"
所以测试/example 的 main 必须真正调用库 —— 这是"假绿"的解毒剂
```

---

## 阶段 2 开工前自查（对照本复盘）

- [ ] mecanum/omni 的 impl 全部 const，局部变量返回
- [ ] 构造函数初始化所有参数成员
- [ ] 先写 `jacobian_apply`（翻译官）再写两个底盘——先有重复再收敛
- [ ] 锚点用例手算独立来源（Mecanum：{0.5,0,0} 四轮相等；{0,0.5,0} 前后轮组符号相反）
- [ ] 写完立刻实例化验证（test main 真正调用）
