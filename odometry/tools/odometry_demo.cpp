// tools/odometry_demo.cpp — odometry 仿真 + CSV 导出（开发/分析工具，非库示例）
// ============================================================================
// 用途：让作者亲眼看见 odometry 跑出来的轨迹，并把每帧数据落成 CSV
//       （CSV 是打滑检测、底盘建模、复盘分析的原料 —— 见 docs/TODO.md P19）
//
// 用法：./odometry_demo [输出文件]     默认 run.csv
// 配套：python3 tools/plot_odometry.py run.csv run.png
//
// 注意：这是【工具】，不是库的示例。它依赖 inc/odometry.hpp，但 odometry.hpp
//       本身不依赖本文件 —— 库零依赖的性质不受影响。
// ============================================================================

#include "odometry.hpp"

#include <cstdio>
#include <cstring>

// 命名空间（2026-09-17）：全仓类型收进 lunokhod::（规则见 AGENTS.md §3）
using namespace lunokhod;
using namespace lunokhod::odometry;


namespace {

// sink 实现：ctx 带的是 FILE*
void csv_sink(void* ctx, const OdometrySample& s) {
    std::fprintf(static_cast<FILE*>(ctx),
                 "%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                 "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                 s.tick_, s.dt_,
                 s.ws_.values_[0], s.ws_.values_[1], s.ws_.values_[2], s.ws_.values_[3],
                 s.cmd_.vx_, s.cmd_.vy_, s.cmd_.wz_,
                 s.twist_.vx_, s.twist_.vy_, s.twist_.wz_,
                 s.pose_.x_, s.pose_.y_, s.pose_.yaw_);
}

struct Leg {
    const char* name_;
    float vx_, vy_, wz_, seconds_;
    float slip_;          // 0 = 无打滑；0.4 = 实际只有期望的 60%
};

}  // namespace

int main(int argc, char** argv) {
    const char* out_path = (argc > 1) ? argv[1] : "run.csv";

    FILE* fp = std::fopen(out_path, "w");
    if (fp == nullptr) {
        std::fprintf(stderr, "打不开输出文件: %s\n", out_path);
        return 1;
    }
    std::fprintf(fp, "tick,dt,ws0,ws1,ws2,ws3,cmd_vx,cmd_vy,cmd_wz,"
                     "twist_vx,twist_vy,twist_wz,x,y,yaw\n");

    // 四段场景：直行 → 圆弧 → 麦轮横移（注入打滑）→ 原地转
    const Leg legs[] = {
        {"straight", 0.50f, 0.00f, 0.00f, 2.0f, 0.0f},
        {"arc",      0.40f, 0.00f, 0.50f, 3.0f, 0.0f},
        {"slide",    0.00f, 0.30f, 0.00f, 2.0f, 0.4f},   // ← 注入 40% 打滑
        {"spin",     0.00f, 0.00f, 1.00f, 1.5f, 0.0f},
    };

    const float dt = 0.01f;              // 100 Hz
    Odometry    odom;                    // twist_scale = 1.0
    odom.set_sink(csv_sink, fp);

    WheelSpeeds ws{4, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
    uint32_t    tick = 0;

    std::printf("%-10s %8s %8s %8s\n", "段", "时长(s)", "打滑", "起点(x,y)");
    for (const Leg& L : legs) {
        const Pose before = odom.pose();
        std::printf("%-10s %8.1f %8.0f%% (%6.3f,%6.3f)\n",
                    L.name_, L.seconds_, L.slip_ * 100.0f, before.x_, before.y_);
        const Twist cmd{L.vx_, L.vy_, L.wz_};
        // "实际"= 期望 × (1 − 打滑率)；轮速记录用实际体速度（真实编码器读数）
        const Twist actual{L.vx_ * (1.0f - L.slip_),
                           L.vy_ * (1.0f - L.slip_),
                           L.wz_};
        ws.values_[0] = actual.vx_; ws.values_[1] = actual.vx_;
        ws.values_[2] = actual.vx_; ws.values_[3] = actual.vx_;

        const int n = static_cast<int>(L.seconds_ / dt);
        for (int i = 0; i < n; ++i) {
            odom.update(actual, dt, ++tick, cmd, &ws);
        }
    }
    std::fclose(fp);

    const Pose p = odom.pose();
    std::printf("\n终点 (%.4f, %.4f)  yaw=%.4f（wrap）  连续 yaw=%.4f（%.2f 圈）\n",
                p.x_, p.y_, p.yaw_, odom.yaw_continuous(),
                odom.yaw_continuous() / 6.28318530718f);
    std::printf("已写出 %s：%u 帧 × 15 列\n", out_path, tick);
    std::printf("画图： python3 tools/plot_odometry.py %s run.png\n", out_path);
    return 0;
}
