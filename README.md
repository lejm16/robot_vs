# robot_vs

本仓库实现了多机器人红蓝对抗系统，采用 **Manager + Car Agent + Skill** 三层架构：
- **Manager 层**（`scripts/manager/`）：感知全局战场状态，调用 LLM 规划战术，通过 `TaskCommand` 消息向各小车下发任务。
- **Car Agent 层**（`scripts/car/`）：每辆小车运行一个独立的 `car_node.py`，接收任务并通过技能（Skill）执行动作，同时将 `RobotState` 反馈给 Manager。
- **Skill 系统**（`scripts/car/skills/`）：GoToSkill（导航）、StopSkill（刹车）、AttackSkill（追击开火）、RotateSkill（原地转向）、RetreatSkill（撤退），实现任务的原子化执行。
- **裁判系统**（`scripts/manager/referee_node.py`）：全局唯一，负责命中判定、扣血、可见敌人计算与比赛胜负判定。

详细架构说明与数据流图请参阅 → **[技术原理文档](TECHNICAL.md)**

---

## 演示 / Demo

> 🎬 演示图/视频即将更新，敬请期待……

---

## 功能特性

- 支持 **仿真环境** 与 **现实环境** 下的多机器人独立运行
- 采用 **红方 / 蓝方 两个阵营** 的对抗结构
  - 每个阵营有一个 Manager 节点负责 LLM 决策与任务分配
  - 每辆小车运行一个 Car Agent，通过技能系统执行 GOTO / STOP / ATTACK / ROTATE / RETREAT 五类动作
  - 小车携带 `mode` 字段区分待机 / 巡逻 / 攻击模式
- **规则博弈兜底**：LLM 不可用时自动走内置规则（角色分配 + 集火目标 + 攻守战术 + 低血撤退）
- **裁判判定**：射线命中判定、掩体遮挡、可见敌人视野计算、一方全灭 / 超时判负
- **比赛状态机**：`IDLE / PLAYING / FINISHED` 三态，支持手动 `start` / `stop` / `reset` 或启动即开赛
- 基于 **命名空间 + TF 前缀** 实现多机话题隔离，防止冲突
- `TaskCommand` / `RobotState` 消息形成完整的任务下发与状态反馈闭环
- 仿真与现实话题结构保持一致，便于算法迁移
- 提供编辑好的 Rviz 可视化界面

---

## 快速开始

详细的环境搭建与部署步骤请参考 → **[环境配置文档](INSTALL.md)**

```bash
# 1. 克隆项目到 ROS 工作空间
cd ~/catkin_ws/src
git clone https://github.com/Xqrion/robot_vs.git

# 2. 编译
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 3. 一键启动 3V3 仿真
#    （Gazebo 6 车 + 裁判 + 红蓝两个 Manager + 6 个 Car Agent）
roslaunch robot_vs simulation/3v3vs_simulation.launch

# 4. 另开一个终端：开始比赛（比赛默认处于 IDLE，必须手动 start，否则 Manager 只会空转）
rostopic pub -1 /game/command std_msgs/String "data: 'start'"
```

> 想省掉第 4 步可以加 `auto_start:=true`：
> `roslaunch robot_vs simulation/3v3vs_simulation.launch auto_start:=true`
>
> 比赛结束后可复位再来一局：`rostopic pub -1 /game/command std_msgs/String "data: 'reset'"`
>
> 若只想起单条链路做调试：`roslaunch robot_vs managers.launch`（红蓝两个 Manager 都会启动）
> 加 `roslaunch robot_vs cars.launch`（红蓝各一辆小车）。
>
> 各 launch 会自动加载对应的 YAML 配置，无需手动传参；巡逻点、队伍颜色、命中判定等参数
> 请直接编辑 `config/` 下对应的 YAML 文件。

---

## 地图

`maps/world0.pgm / world0.png / world0.yaml` 是由脚本从 `worlds/world0.world`
离线生成的，和仿真场地（8.25 m × 4.15 m 围墙 + 4 根障碍柱）完全一致。
改了世界文件后重新生成一次即可：

```bash
python3 scripts/world_to_map.py --world worlds/world0.world --out maps/world0
```

> 仓库里原有的 `maps/map_simulation*.png` 只是占位图（整张空白 + 中间一个黑方块），
> 和 world0.world 对不上，仿真 launch 已不再使用它们。
>
> 脚本会把场地边界一并算好写进 yaml 的 `origin`，不要手改；
> 需要调整小车活动范围时改 `config/manager/*_manager.yaml` 里的
> `arena_min_x / arena_max_x / arena_min_y / arena_max_y`（所有下发目标点都会被夹进这个矩形）。

---

## 仿真跑通检查清单

仿真里有 **地图 → 定位 → 规划 → 决策** 四条链路，任何一条断了都会表现成
“车不动 / 打不起来”。按顺序确认：

| 检查项 | 命令 | 期望结果 |
|--------|------|----------|
| 地图 | `rostopic echo -n1 /map/info` | `width: 363  height: 199  resolution: 0.025` |
| 激光 | `rostopic hz /robot_red1/scan` | 稳定出数据 |
| 里程计 | `rostopic hz /robot_red1/odom` | 稳定出数据 |
| 定位 | `rostopic hz /robot_red1/amcl_pose` | 稳定出数据 |
| TF 链 | `rosrun tf tf_echo map robot_red1/base_footprint` | 能打变换 |
| 任务下发 | `rostopic echo /robot_red1/car_task` | 出现 GOTO / ATTACK |
| 机器人上报 | `rostopic echo -n1 /robot_red1/robot_state` | 坐标不是 (0,0) 且随车变化 |
| 比赛状态 | `rostopic echo /game/state` | `status: "PLAYING"` |

常见现象与处理：

| 现象 | 多半是 | 处理 |
|------|--------|------|
| 车完全不动，日志里出现 `GoToSkill: ... move_base 没有返回结果、车也没有移动` | 地图 / 定位 / 话题名断了 | 按上表逐项排查；应急可在 `config/car/*.yaml` 里把 `use_move_base` 设为 `false`（纯 cmd_vel 直行，无避障，仅验证用） |
| 所有车坐标都是 (0,0)，裁判永远打不中 | `/<ns>/odom`、`/<ns>/amcl_pose` 收不到数据 | 确认 gazebo 插件的话题是不是带命名空间（`rostopic list \| grep -E 'cmd_vel\|odom\|scan'`） |
| 车往场外/墙里开 | 目标点在场地外 | 检查 `arena_*` 参数；`TaskDispatcher` 会自动夹目标点并打印 `超出场地，已夹到` |
| 双方互相看不见 | 视野太小 | `config/manager/referee.yaml` 的 `vision_range`（默认 3.5 m）与 `fov_deg` |

---

## 文档索引

| 文档 | 内容 |
|------|------|
| [环境配置](INSTALL.md) | 虚拟机搭建、ROS 安装、项目部署全流程 |
| [技术原理](TECHNICAL.md) | 系统架构、Manager/Car/Skill 详解、ROS 消息流、数据流图、比赛状态机与裁判判定 |

---

## 项目状态

| 模块 | 状态 |
|------|------|
| 仿真环境（Gazebo + Rviz） | ✅ 已完成 |
| 红蓝阵营 Manager 框架 | ✅ 已完成 |
| 真机局域网下通信测试 | ✅ 已完成 |
| Car Agent + Skill 系统 | ✅ 已完成 |
| 裁判系统对接 | ✅ 基础版（命中/遮挡/视野/胜负判定） |
| 大模型接入 | 🚧 规划服务已实现，默认关闭（`llm.enabled: false`） |
| 现实环境部署 | 🚧 进行中 |
