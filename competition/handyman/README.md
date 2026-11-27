# Handyman 比赛程序维护入口

维护源码在此目录，`build/handyman-competition` 是生成的本机运行包，不再纳入 Git，也不要直接修改其中的程序。

- `session1/`：六轮固定/动态任务、导航抓取放置、协议、掉落重拾、时钟桥源码以及 `tests/`。
- `tools/`：搬运和视觉公共实现。
- `support/`：原 runtime 中的辅助源码、配置、几何与历史回放输入。保留这些输入以避免丢失旧测试依赖；不是实时运行日志目录。
- `config/`：物品抓法、桶口、房间物品绑定等配置。
- `launch/`：启动、清理、自检脚本、任务配置及使用说明。
- `project_overrides/`、`workspace_sources/`：已验证比赛环境所需的源码/地图副本。与仓库根目录一致的文件由清单直接引用，不重复复制；不同副本保留为明确的比赛专用版本，不能直接覆盖另一个项目的 ROS 实现。
- `package_manifest.json`：维护源码到运行包路径的显式映射。新增文件时也需添加映射；已有文件修改不需要更新历史校验值。

ROS 主工程仍位于仓库根目录 `src/`。独立 Python 比赛调度不属于独立 ROS 包，放在 `competition/handyman`。

## 开发与生成

代码中的 `/__HANDYMAN_PACKAGE__` 是构建时路径标记，不能直接作为运行路径使用。修改本目录源码，然后生成新包验证；不要在维护源码目录直接运行比赛。

从仓库根目录生成源码包（不具备模型和运行环境，启动会被标记阻止）：

```bash
python3 competition/handyman/build_package.py --output build/handyman-source-check
```

利用已验证的本机运行依赖生成独立完整包并自动自检：

```bash
python3 competition/handyman/build_package.py \
  --runtime-base build/handyman-competition \
  --output build/handyman-competition-next
```

输出目录必须不存在，脚本不会覆盖旧包。模型权重、Python 环境、ROS 安装产物及编译时钟桥由 `--runtime-base` 提供，不纳入 Git。脚本复制这些依赖、重定位路径、覆盖正式源码并执行 `check_competition.sh`；自检失败保留 `PACKAGING_IN_PROGRESS` 阻止启动。离线自检不发布机器人控制指令。此工具不是从零安装系统依赖的安装器；只有 Git 克隆不足以创建完整运行环境。

检查通过后按新包中的 `使用说明.md` 启动。替换正式包须先正常退出比赛进程，再重命名旧包留作备份，随后将新包移到正式路径并再次自检。

## 本次验证

本次迁移前六轮任务与恢复逻辑已有 645 项离线回归测试通过。源码迁移不改变抓取、导航、投放策略。完整包生成验证结果记录在新包 `package-check.log`；历史运行和备份仍留在原本机包的 `runs/`、`audit/`。

迁移验证结果（2026-11-27）：在 `build/handyman-competition-next` 独立路径重新生成完整运行包，自检通过，645 项比赛回归测试通过；构建器另有 3 项测试通过。原运行包未替换，没有发送机器人控制指令。Git 当前版本不再跟踪 `build/`；之前的提交历史保留，不进行强制改写。
