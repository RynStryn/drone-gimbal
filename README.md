# 无人机云台：控制调试与架构重整

以已有云台框架为基础，保留姿态、电机、停火逻辑、前馈及重力补偿的调试记录，追踪 beta 到 clean 的重构过程。

**当前分支**：`archive/stable-backup`。Stable 命名的历史工作树。

**留存状态**：重构工作树保留。

[查看项目时间线](docs/TIMELINE.md)

## 来源

README 记载基于江文涛框架；框架来源与个人改动需要分别署名。

个人职责待补充。保留原工程中的来源说明和许可证；本次归档没有进行构建、烧录或实机验收。

## 代码与历史分支

| 分支 | 内容 |
| --- | --- |
| `main` | 项目代表工作树，包含本次保存的现存源码。 |
| [`archive/beta-working-tree`](https://github.com/RynStryn/drone-gimbal/tree/archive/beta-working-tree) | beta 控制工作树 |
| [`archive/framework-working-tree`](https://github.com/RynStryn/drone-gimbal/tree/archive/framework-working-tree) | 完整框架工作树 |
| [`archive/stable-backup`](https://github.com/RynStryn/drone-gimbal/tree/archive/stable-backup) | Stable 命名的历史工作树 |

## 时间口径

历史研发／修改日期写在时间线中。本次于 2026-11-04 整理并提交现存源码；新快照提交使用实际提交日期。

有原 Git 历史的工作树以原 HEAD 为父提交保留历史；无 Git 历史的树以独立根提交保存，不补造历史提交。工作区中的源码变动一并保存在本次快照。

本次快照排除了构建输出、IDE 会话文件和通用整理资料。历史提交保持原样，其中可能保留以前已跟踪的工具状态或构建文件。

本分支的原始 Git 检查点：[ `09471f9` ](https://github.com/RynStryn/drone-gimbal/commit/09471f9a75e758e9f135c800b56a21a3843c946e)。

[原工程 README](docs/README.original.md)
