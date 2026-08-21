# Stage 2 复盘（mecanum + omni + 翻译官收敛）

> 阶段 2 完成：三底盘（diff/mecanum/omni）+ `jacobian_apply` 翻译官 + 架构命名语义修正。
> 测试全绿（锚点 + 互逆 + 边界），-Wall -Wextra -Werror 零警告。
> 按点记录本阶段错误与收获，供阶段 3（SpeedLimiter）开工前回看。

---

## 一、错误清单（按点，含正确示例）

### E1. 模板参数没绑定数组尺寸（本阶段最大的编译坑）
```cpp
// ❌ MAX 是全局常量，数组类型死绑 [6][3]；模板参数 N 在签名里没出现 → 无法推导
constexpr uint8_t MAX = 6;
template<uint8_t N>
WheelSpeeds jacobian_apply(const float (&J)[MAX][3], uint8_t wn, const Twist& t_cmd)
// ❌ mecanum 传 [4][3] 也类型不匹配
// ✅ 模板参数直接绑定数组尺寸，N 自动推导（mec=4, omni=6）
template<uint8_t N>
WheelSpeeds jacobian_apply(const float (&J)[N][3], uint8_t wn, const Twist& t_cmd) {
    WheelSpeeds out_ws;
    out_ws.count_ = wn;
    for (uint8_t i = 0; i < wn; ++i) { ... }   // 循环 wn 行，不碰未初始化行
}
```
报错特征：`couldn't deduce template parameter 'N'`。

### E2. 假绿再现：新文件没被实例化
```cpp
// ❌ kinematics.hpp 没 include mecanum/omni → test 根本看不到它们 → 编译"全绿"是假绿
// ✅ 聚合入口补齐 include 后，模板错误才在实例化点暴露
```
教训重申：**任何新文件写完，第一件事是让它被真实调用**。

### E3. VLA（变长数组）是非法 C++
```cpp
float J[wn_];   // ❌ ① C++ 标准无 VLA（GCC 扩展）② 维度错，J 需要 N×3
// ✅ 定长上限 + 运行时行数（贴合库的定长数组哲学，零堆）
float J[6][3] = {};          // 全零初始化防手滑
for (uint8_t i = 0; i < wn_; ++i) { ... }
```

### E4. mecanum 的 J 缺 1/r_（数学 bug）
```cpp
// ❌ 轮速 = J × cmd 但 J 没除轮半径 → 输出大 r_ 倍
for (uint8_t i = 0; i < 4; ++i) { J[i][0] /= r_; J[i][1] /= r_; J[i][2] /= r_; }
// ✅ 统一除 r_；forward 侧乘 r_（对称）
```

### E5. 未定义符号 FL/FR/RL/RR → 轮序约定
```cpp
t.vx_ = r_ * (FL+FR+RL+RR) / 4;   // ❌ FL 等未定义
// ✅ 轮序约定（必须与 J 行序一致，并写进注释）：
// values_[0]=FL  values_[1]=FR  values_[2]=RL  values_[3]=RR
// forward 符号已验证与 inverse 互逆
```

### E6. 幽灵 include
```cpp
#include "drive_diff.h"     // ❌ 文件不存在（只有 .hpp）
#include "drive_diff.hpp"   // ✅
```

### E7. 构造函数名与类名不匹配
```cpp
class DiffDrive : ... { DiffDrive(...) ... }   // ✅
// ❌ 重命名类时漏改构造函数：class DiffDrive + DifferentialDrive(...) 编译错
```

### E8. 保留标识符
```cpp
constexpr float _2PI;   // ❌ 下划线+大写开头是 C++ 保留给实现的标识符（严格说是 UB）
constexpr float k2PI;   // ✅ k 前缀（常量惯例）
```

### E9. include 依赖倒挂 → 抽公共基座
```cpp
// ❌ mecanum/omni include drive_diff.hpp 只为拿 Chassis 基类 + jacobian_apply
//    drive_diff 成了"基座文件"，语义混乱
// ✅ 抽 chassis.hpp（行为基座），drive_* 各自 include 它
```

---

## 二、架构决策：命名语义修正（本阶段最重要的讨论）

### 问题：谁才是"底盘"？
```
Chassis<Derived>（基类）    内容 100% 是运动学运算，零底盘属性
DiffDrive / MecanumDrive / OmniDrive   有物理参数（轮距/轮径/轮数）= 真正的底盘
```

### 结论：基类改名 + 文件角色互换
```
inc/types.hpp          数据层：Twist / WheelSpeeds（POD 契约）
inc/kinematics.hpp     数学核心：Kinematics<Derived> 基类 + jacobian_apply + k2PI
inc/drive_diff.hpp     DiffDrive : public Kinematics<DiffDrive>
inc/drive_mecanum.hpp  MecanumDrive（4×3 J，forward 伪逆展开）
inc/drive_omni.hpp     OmniDrive（N×3 J，forward 3 轮特例 γ=0）
inc/chassis.hpp        对外聚合入口（用户只 include 这一个）
```

- `DiffDrive : public Kinematics<DiffDrive>` 读作"差速底盘**具备运动学能力**"（Comparable/Drawable 模式）
- **chassis（应用）在上，kinematics（数学）在下**——应用层名字暴露给应用
- 数学核心独立可复用：将来导航模块只要 jacobian，`#include "kinematics.hpp"` 即可
- 文档盲点承认：DESIGN.md 定义 Chassis 时只想着"底盘家族"，没考虑文件组织语义

---

## 三、讨论收获

### 1. forward 也有翻译官，但先手写
```
inverse:  wheel = J       × cmd   →  jacobian_apply(J, cmd)
forward:  cmd   = pinv(J) × wheel →  jacobian_apply(pinvJ, wheel)（将来）
```
- 对称性成立，但伪逆符号藏在矩阵里错了极难查 → **先显式手写公式证明，再抽矩阵**
- 抽象永远在正确之后；先有重复再收敛

### 2. 测试锚点期望值用有理数分数，不用四舍五入小数
```cpp
CHECK(close(s1.values_[0], 50.0f/3.0f));    // ✅ 16.6666... 精确表达
CHECK(close(s1.values_[0], 16.6667f));      // ❌ 与真实值差 3.3e-5 > 容差 1e-5 → 误报 FAIL
```
手算锚点写成精确分数，独立渠道更纯。

### 3. example vs test 定位
| | test | example |
|---|---|---|
| 读者 | 机器（退出码） | 人（API 调用示范） |
| 价值 | 证明"对不对" | 证明"怎么调" |
- 用户反馈：example 是静态的，只能验证数学/教调用；**更想要仿真演示（L3：2D 位姿积分）**——记录为阶段 3+ 需求

### 4. 模板实例化心智模型（再次确认）
- 模板未调用 = 未真正编译；错误在实例化点（调用处）爆发，找 `required from here`
- 新文件写完第一件事：让它被真实调用（test 或 example）

---

## 四、阶段 3 开工自查

- [ ] SpeedLimiter：**第一个有状态类**（成员 prev_ 速度/时间戳），方法非 const，与纯函数底盘形成对照
- [ ] 新增文件后立即实例化验证（防 E2 假绿）
- [ ] 锚点手算：加速度限幅的斜坡响应（如 0→0.5m/s，acc=1 → 0.5s 到达）
- [ ] 仿真演示需求（用户提出）：2D 位姿积分仿真——把 inverse 轮速积分成 (x,y,θ) 轨迹，可视化验证"看得见"
- [ ] 交叉编译冒烟（arm-none-eabi -fsyntax-only）可提前做，验证嵌入式兼容性
