---
class: fact
generated: false
---
> **类：B 事实** —— **唯一来源**：别处只许链接，不许复制；改决策只改这里。 全局 git 工作流（2026-09-13 从 control/wheel/docs/ 上移，内容未改，其中 wheel 为示例）。
> 文档体系与写作规则：README.md

# wheel 开发工作流：SemVer + 渐进稳定分支

> 目标：**跟着本文档走完每一步，就是一个清晰、可照做的 git branching 流程。**
> 适用：个人组件库（wheel），未来要拖进 STM32 工程、被别人/未来的你使用。

## 1. 总览：稳定性阶梯

```
最不稳定                             最稳定
feature/xxx → main → release/v0.x → tag vX.Y.Z
  开发中      集成线   冻结只修bug     发布点(不可变)
```

**代码只能单向向右流动**（合并方向），反向只有一种合法操作：cherry-pick（把 release 上修的 bug 摘回 main）。

## 2. 铁律（就 4 条）

1. **main 是集成线**：功能开发都在 main 上，每个 commit 后必须能编译 + 测试全绿（锚点测试是守门员）
2. **release/v0.x 是冻结线**：功能攒够一个里程碑 → 从 main 切出 release 分支，**只修 bug，不加功能**
3. **tag 不可变**：发布点打 tag，出了 bug 只能发下一个 PATCH（v0.1.1），永远不改 v0.1.0
4. **版本号单一来源**：只维护在 `CMakeLists.txt` 的 `project(... VERSION x.y.z)`，发布时同步 + CHANGELOG 记一笔

## 3. 一次性修复（首次启用工作流时做，做完即忘）

> 场景：首次 commit 时 `.idea/` 被误提交（原因：.gitignore 行尾注释无效）+ 分支名是 master 而非 main。

```bash
cd /home/wilson/Dev/Workspace/lunokhod/control/wheel

# ① 确认 .gitignore 内容正确（注释独占一行，.idea/ 单独一行）
cat .gitignore

# ② 把 .idea/ 从版本库移除（--cached：只出库，工作区文件保留）
git rm -r --cached .idea/

# ③ 分支改名 master → main（现在没有远端，正是改名的唯一无痛时机）
git branch -m main

# ④ 提交修复
git add -A
git commit -m "chore: .idea/ 移出版本库（IDE 文件不进库）；分支改名 main"
git status --short        # 应只剩代码文件，.idea/ 不再出现
```

## 4. 日常开发（90% 的时间在这）

```bash
git checkout main
# ...写代码、跑测试，全绿后...
git add -A
git commit -m "feat(wheel): 完整描述改动"     # 一条 commit 一个完整主题
git push origin main                          # 首次 push 用 -u 记住上游
```

**commit 信息规范**（对齐 kinematics 历史）：
- `feat(x): ...` 新功能
- `fix(x): ...` 修 bug
- `refactor(x): ...` 重构（行为不变）
- `chore: ...` 杂项（gitignore、构建等）
- `docs: ...` 文档
- 括号里写组件名：`feat(pid): 积分分离阈值接入 Config`

## 5. 发布流程（里程碑完成时）

> 触发条件：一个可交付的状态（如：PID + Wheel 完成、测试全绿、example 闭环仿真通过）

```bash
# ① main 全绿确认
./build/test_wheel && echo "ALL PASS"

# ② 冻结：切 release 分支
git checkout -b release/v0.1 main

# ③ 版本号：CMakeLists.txt 写 project(lunokhod_wheel VERSION 0.1.0)
#    写 CHANGELOG.md（第一条：v0.1.0 有什么）

# ④ 提交发布准备
git add -A
git commit -m "chore(release): v0.1.0"

# ⑤ 打 tag 并推送（tag = 不可变发布点）
git tag v0.1.0
git push origin main release/v0.1 v0.1.0

# ⑥ 回到 main 继续开发新功能
git checkout main
```

## 6. bug 修复流程（发布后发现 bug 时）

```bash
# ① 在 release 分支上修（不污染 main 的开发线）
git checkout release/v0.1
# ...修 bug，测试全绿...
git commit -am "fix: 描述 bug"

# ② 版本号升 PATCH：CMakeLists 0.1.0 → 0.1.1 + CHANGELOG 追加
git commit -am "chore(release): v0.1.1"

# ③ 打新 tag
git tag v0.1.1
git push origin release/v0.1 v0.1.1

# ④ cherry-pick 回 main（关键！防止 main 永远缺这个修复）
git checkout main
git cherry-pick <修复的 commit hash>
git push origin main
```

## 7. 版本号怎么升（SemVer 决策表）

| 变更类型 | 例子 | 版本 |
|---|---|---|
| PATCH：修 bug（API 不变） | 积分分离阈值 bug | 0.1.0 → **0.1.1** |
| MINOR：加功能（API 兼容） | 新增 feedforward 参数 | 0.1.1 → **0.2.0** |
| MAJOR：破坏性变更 | 改函数签名 / 删接口 | 0.2.0 → **1.0.0** |

**0.x 特殊约定**：MAJOR=0 期间，MINOR 允许破坏性变更（0.1 → 0.2 可以改 API）——这正是"还没承诺稳定"的信号。**1.0.0 之后**：破坏性变更必须升 MAJOR，且每个 MINOR 分支维护到明确声明退役为止。

## 8. 命令速查表

| 想做什么 | 命令 |
|---|---|
| 日常提交 | `git add -A && git commit -m "..."` |
| 推送 | `git push origin main` |
| 冻结发布 | `git checkout -b release/v0.1 main` |
| 打 tag | `git tag v0.1.0` |
| 看当前分支 | `git branch` |
| 看 tag | `git tag` |
| 看历史 | `git log --oneline --graph` |
| 修完摘回 main | `git checkout main && git cherry-pick <hash>` |
| 反悔（未 push） | `git reset --soft HEAD~1`（保留改动） |

## 9. 检查单（每次 commit 前）

- [ ] `cmake --build build` 零错误零警告（-Werror 已开）
- [ ] `./build/test_wheel` 输出 ALL PASS
- [ ] 版本号没被动过（除非在发布流程中）
- [ ] `.idea/`、`build/`、`cmake-build-debug/` 不在 `git status` 里
