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
> 比赛结束后**默认会自动复位并开下一局**（`config/manager/referee.yaml` 的 `auto_reset: true`，
> 停留 6 秒再复位）。想手动控制就发：
> `rostopic pub -1 /game/command std_msgs/String "data: 'reset'"`（复位）
> 或 `"data: 'stop'"`（停到 IDLE）。
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

### 场地障碍布局

`world0.world` 里的障碍是**整体 180° 旋转对称**的，保证红蓝完全公平：

| 障碍 | 尺寸 | 位置 | 作用 |
|------|------|------|------|
| `Obstacle_Center` | 1.80 × 0.30 | (0, 0) | 挡住中路，双方中路出生时互相看不见 |
| `Obstacle_Left/Right_Top` | 0.90 × 0.30 | (±2.5, +0.7) | 挡住左右出生通道的视线 |
| `Obstacle_Left/Right_Bot` | 0.90 × 0.30 | (±2.5, -0.7) | 同上（蓝方一侧） |

设计意图：三条出生通道的正前方视线都被遮住，双方**出生时互相看不见**，
必须开出去侦察才能交火；同时各通道之间留了 1 m 以上的通行口，
小车（含 move_base 的膨胀层）能顺利绕行。
改完世界文件后必须重新生成地图，否则地图和场地会对不上。

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
| TF 前缀 | 启动日志里的 `[tf_bridge/robot_red1] 实际坐标系: ...` | 看 gazebo 插件用的是带前缀还是不带前缀的坐标系 |
| 任务下发 | `rostopic echo /robot_red1/car_task` | 出现 GOTO / ATTACK |
| 机器人上报 | `rostopic echo -n1 /robot_red1/robot_state` | 坐标不是 (0,0) 且随车变化 |
| 比赛状态 | `rostopic echo /game/state` | `status: "PLAYING"` |

常见现象与处理：

| 现象 | 多半是 | 处理 |
|------|--------|------|
| 车完全不动，日志里出现 `GoToSkill: ... move_base 没有返回结果、车也没有移动` | 地图 / 定位 / 话题名断了 | 按上表逐项排查；应急可在 `config/car/*.yaml` 里把 `use_move_base` 设为 `false`（纯 cmd_vel 直行，无避障，仅验证用） |
| **车只在原地打转、不往前走** | move_base 全局规划一直失败，进入 recovery 的"原地自转"（最常见是 amcl 没定位成功 → 缺 `map→odom` 的 TF） | 现在已经会自动降级：日志里会出现 `本车接下来会自动改用自带激光绕障直行`。若降级后还不走，看 `tf_bridge` 打印的实际坐标系名字 |
| 所有车坐标都是 (0,0)，裁判永远打不中 | `/<ns>/odom`、`/<ns>/amcl_pose` 收不到数据 | 确认 gazebo 插件的话题是不是带命名空间（`rostopic list \| grep -E 'cmd_vel\|odom\|scan'`） |
| 车往场外/墙里开 | 目标点在场地外 | 检查 `arena_*` 参数；`TaskDispatcher` 会自动夹目标点并打印 `超出场地，已夹到` |
| 双方互相看不见 | 视野太小 | `config/manager/referee.yaml` 的 `vision_range`（默认 3.5 m）与 `fov_deg` |

### 定位链路与两条导航路线

每台车的 GOTO 有两条路线，代码会自动选：

| 路线 | 触发条件 | 依赖 |
|------|----------|------|
| move_base 规划 | `use_move_base: true`（默认） | `map_server` + `amcl` + `/scan` + TF 全链路 |
| 自带激光绕障直行（`skills/steering.py`） | `use_move_base: false`，或 move_base 连续失败自动降级 | 只需要 `/odom` 和 `/scan` |

move_base 失败时会**自动降级 60 秒**，所以定位坏掉也不会让整场比赛卡死，日志会说明原因。

另外为定位链路加了 `scripts/car/tf_bridge.py`：多机仿真里每台车的 TF 必须带前缀
（`robot_red1/odom`、`robot_red1/base_footprint` …），而 turtlebot3 原版 URDF 的
gazebo 插件可能用的是**不带前缀**的 `odom / base_footprint / base_scan`。
该节点会读 `/odom` 与 `/scan` 报文里真实的坐标系名字并打印出来，且**只在不匹配时**
补一条 identity 静态变换把两边接起来（匹配时不发，避免 TF 成环）。

---

## 文档索引

| 文档 | 内容 |
|------|------|
| [环境配置](INSTALL.md) | 虚拟机搭建、ROS 安装、项目部署全流程 |
| [技术原理](TECHNICAL.md) | 系统架构、Manager/Car/Skill 详解、ROS 消息流、数据流图、比赛状态机与裁判判定 |

---

## 观战（RViz）

3V3 仿真启动时会自动打开 `rviz/multi_robot.rviz`，里面已经配好：

- `/map` 全局地图（Fixed Frame = `map`）
- 6 台车的 `RobotModel` 与 `LaserScan`（`robot_red1..3` / `robot_blue1..3`）
- `/health_markers`（血条）、`/chassis_markers`（底盘颜色）、`/trajectory_markers`（弹道）

> 之前这份配置用的是旧的四车命名空间（`robot_red` / `robot_blue`），
> 在 3V3 里车模型和雷达都不显示，已对齐到六车。
> 想加回 move_base 的代价地图/路径显示时，照着现有的
> `Global Map [Red1]` 块复制，把话题里的 `robot_red1` 换成目标车即可。

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
