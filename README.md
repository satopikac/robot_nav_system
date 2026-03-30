# Robot Navigation System

集成自主探索与 LLM 驱动任务执行的机器人导航系统。

系统启动后进入自主探索模式，使用 explore-lite 进行前沿探索，同时 DMROS 实时处理 RGBD 数据构建语义地图；当收到自然语言指令后，系统自动从探索切换为任务执行模式，完成导航任务后自动恢复探索。

---

## 目录

- [系统架构](#系统架构)
- [核心特性](#核心特性)
- [多机地图合并](#多机地图合并)
- [项目结构](#项目结构)
- [依赖环境](#依赖环境)
- [安装](#安装)
- [配置系统](#配置系统)
- [三种部署场景](#三种部署场景)
- [使用说明](#使用说明)
- [系统状态机](#系统状态机)
- [数据流](#数据流)
- [模块详解](#模块详解)
- [异常处理](#异常处理)
- [常见问题](#常见问题)

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                     SystemOrchestrator                       │
│                                                              │
│  ┌─────────────┐  ┌──────────────────┐  ┌────────────────┐  │
│  │ StateMachine │  │ ExploreController│  │ TaskController  │  │
│  │ (模式切换)   │  │ (explore-lite)   │  │ (Agent管线)    │  │
│  └──────┬──────┘  └────────┬─────────┘  └───────┬────────┘  │
│         │                  │                     │           │
│  ┌──────┴──────────────────┴─────────────────────┴───────┐  │
│  │                  NavigationAgent                       │  │
│  │  LLMClient → TaskPlan → Allocator → Navigator         │  │
│  └──────────────────────────┬────────────────────────────┘  │
│                             │                                │
│  ┌──────────────────────────┴────────────────────────────┐  │
│  │               Perception Pipeline                      │  │
│  │  DMROS → MapWatcher → MapConverter → SemanticMap       │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

---

## 核心特性

### 两条管线

| 管线 | 说明 |
|------|------|
| **单机任务执行** | 一台机器人按顺序执行导航子任务 |
| **多机协同执行** | 多台机器人并行执行，支持 Greedy/Hungarian 任务分配和 DAG 依赖同步 |

### 两种模式

| 模式 | 说明 |
|------|------|
| **探索模式** | explore-lite 自主前沿探索 + DMROS 实时语义建图 |
| **任务执行模式** | 接收自然语言指令后，停止探索，规划并执行导航任务 |

### 三种部署场景

| 场景 | 配置文件 | ROS | 探索 | DMROS | Navigator |
|------|----------|-----|------|-------|-----------|
| TurtleBot3 Gazebo 仿真 | `turtlebot3_sim.json` | 需要 | explore-lite | 开启 | ROSMultiNavigator |
| 手动输入模拟 | `manual_sim.json` | 不需要 | 无 | 关闭 | SimulatedNavigator |
| 真机部署 | `real_robot.json` | 需要 | explore-lite | 开启 | ROSSingleNavigator |

---

## 多机地图合并

### 概述

在多机模式（`ros_multi`）下，每台机器人独立进行探索和建图：

- **几何地图**：每台机器人运行独立的 SLAM（gmapping），产生各自的 occupancy grid，由 `multirobot_map_merge` ROS 包实时合并为全局几何地图
- **语义地图**：每台机器人运行独立的 DMROS 实例，处理各自的 RGBD 数据，产生各自的 `semantic_map.json`，由 `MultiRobotMapManager` 实时合并为全局语义地图

### 架构

```
Robot_0                        Robot_1                        Robot_2
  │                              │                              │
  ├─ SLAM_0 ──┐                  ├─ SLAM_1 ──┐                  ├─ SLAM_2 ──┐
  │           │                  │           │                  │           │
  │     occupancy_grid_0         │     occupancy_grid_1         │     occupancy_grid_2
  │           │                  │           │                  │           │
  │           └────────────────> multirobot_map_merge <─────────┘           │
  │                              │ (全局几何地图)                            │
  │                              │                                          │
  ├─ DMROS_0 ──┐                 ├─ DMROS_1 ──┐                 ├─ DMROS_2 ──┐
  │            │                 │            │                 │            │
  │  output/tb3_0/semantic/      │  output/tb3_1/semantic/      │  output/tb3_2/semantic/
  │  semantic_map.json           │  semantic_map.json           │  semantic_map.json
  │            │                 │            │                 │            │
  │            └─────────────────┴────────────┴─────────────────┘            │
  │                              │                                          │
  │                   MultiRobotMapManager                                  │
  │                     │                                                   │
  │                     ├─ 轮询每台机器人的语义地图变化                        │
  │                     ├─ SemanticMapMerger 去重合并                        │
  │                     ├─ MapConverter 格式转换                             │
  │                     └─ SemanticMap 热重载                               │
  │                              │                                          │
  │                   全局语义地图 (data/semantic_map.json)                   │
  │                              │                                          │
  └──────────────────> NavigationAgent <────────────────────────────────────┘
```

### 语义地图合并算法

`SemanticMapMerger` 使用基于空间距离的去重策略：

1. **收集**：加载所有机器人的本地语义地图
2. **按类分组**：按 `class_name` 对物体分组
3. **空间去重**：同类物体中，距离 ≤ `merge_distance`（默认 1.0m）的视为同一物理物体
4. **位置融合**：合并后的物体位置取所有观测的加权平均
   - x, y: 算术平均
   - yaw: 圆周平均（`atan2(Σsin, Σcos)`）
5. **包围盒选择**：取面积最大的观测包围盒
6. **来源追踪**：记录每个合并物体的来源机器人列表和观测次数

```
示例：
  tb3_0 观测到 chair 在 (3.0, 1.0)
  tb3_1 观测到 chair 在 (3.1, 0.9)   距离 = 0.14m < 1.0m → 合并
  tb3_2 观测到 chair 在 (10.0, 10.0) 距离 >> 1.0m → 视为不同椅子

  合并结果:
    chair_01: (3.05, 0.95), 观测 2 次, 来源 [tb3_0, tb3_1]
    chair_02: (10.0, 10.0), 观测 1 次, 来源 [tb3_2]
```

### 配置

在 `turtlebot3_sim.json` 中：

```jsonc
{
  "dmros": {
    "enabled": true,
    "output_dir": "output",            // 基础输出目录
                                       // 每台机器人输出到 output/{robot_name}/semantic/
    "merge_distance": 1.0,             // 合并距离阈值 (米)
    "poll_interval": 3.0,              // 轮询间隔 (秒)
    "per_robot_topics": {              // 每台机器人的 RGBD 话题
      "tb3_0": {
        "rgb": "/tb3_0/camera/rgb/image_raw",
        "depth": "/tb3_0/camera/depth/image_raw",
        "odom": "/tb3_0/odom",
        "camera_info": "/tb3_0/camera/rgb/camera_info"
      },
      "tb3_1": { ... },
      "tb3_2": { ... }
    }
  }
}
```

### 数据流

```
每台机器人的 RGBD 传感器
    │
    ▼ (命名空间隔离)
/{robot}/camera/rgb/image_raw
/{robot}/camera/depth/image_raw
/{robot}/odom
    │
    ▼
DMROS_{robot} (独立实例)
    │
    ▼
output/{robot}/semantic/semantic_map.json (DMROS 格式, 每台独立)
    │
    ▼ (MultiRobotMapManager 后台轮询)
    │
    ├─ 检测任一机器人的地图文件变化
    ├─ 加载所有机器人的最新地图
    ├─ SemanticMapMerger.merge() 去重合并
    ├─ 写入 output/merged/semantic_map.json (全局 DMROS 格式)
    ├─ MapConverter.convert_fast() 格式转换
    └─ SemanticMap.reload() 热重载
    │
    ▼
data/semantic_map.json (全局 nav 格式, NavigationAgent 使用)
```

### 文件输出结构

```
output/
├── tb3_0/
│   ├── semantic/
│   │   └── semantic_map.json       # tb3_0 的本地语义地图 (DMROS 格式)
│   └── map/
│       └── ...                     # tb3_0 的点云等 (DMROS 输出)
├── tb3_1/
│   └── semantic/
│       └── semantic_map.json       # tb3_1 的本地语义地图
├── tb3_2/
│   └── semantic/
│       └── semantic_map.json       # tb3_2 的本地语义地图
└── merged/
    └── semantic_map.json           # 合并后的全局语义地图 (DMROS 格式)
                                    # 包含 metadata.source_robots, observation_count 等
```

### REPL 状态查看

在多机模式下，`status` 命令会额外显示每台机器人的地图状态：

```
你> status

  state: exploring
  profile: turtlebot3_sim
  runtime_mode: ros_multi
  semantic_map_objects: 12
  multi_robot_maps:
    tb3_0: 5 objects
    tb3_1: 4 objects
    tb3_2: 6 objects
  merge_count: 8
```

---

## 项目结构

```
robot_nav_system/
├── robot_nav_system/                      # ROS catkin 包
│   ├── CMakeLists.txt                     # catkin 构建配置
│   ├── package.xml                        # ROS 包描述
│   ├── setup.py                           # catkin Python 安装
│   │
│   ├── config/
│   │   ├── profiles/                      # 部署场景配置
│   │   │   ├── turtlebot3_sim.json        #   TB3 Gazebo 多机仿真
│   │   │   ├── manual_sim.json            #   手动输入模拟（无 ROS）
│   │   │   └── real_robot.json            #   真机单机部署
│   │   └── dmros/                         # DMROS 话题配置
│   │       ├── turtlebot3_sim.yaml        #   TB3 仿真 RGBD 话题
│   │       └── real_robot.yaml            #   真机 RGBD 话题
│   │
│   ├── launch/
│   │   ├── bringup/
│   │   │   └── gazebo_world.launch        # Gazebo 世界 + 机器人生成
│   │   ├── slam/
│   │   │   ├── slam.launch                # 单机 gmapping SLAM
│   │   │   └── multi_slam.launch          # 多机 SLAM
│   │   ├── navigation/
│   │   │   ├── move_base.launch           # 单机 move_base
│   │   │   └── multi_move_base.launch     # 多机 move_base (DWA)
│   │   ├── exploration/
│   │   │   ├── explore.launch             # 单机 explore-lite
│   │   │   └── multi_explore.launch       # 多机 explore-lite
│   │   ├── perception/
│   │   │   ├── dmros.launch               # 单机 DMROS 语义建图
│   │   │   └── multi_dmros.launch         # 多机 DMROS（每机独立实例）
│   │   └── system/
│   │       ├── full_system.launch         # 完整系统一键启动
│   │       └── nav_agent_only.launch      # 仅 Agent 节点
│   │
│   ├── scripts/                           # 可执行入口
│   │   ├── system_orchestrator_node.py    # 主入口（ROS 节点 / 独立运行）
│   │   └── dmros_bridge_node.py           # DMROS 进程桥接
│   │
│   ├── src/robot_nav_system/              # Python 核心代码
│   │   ├── core/                          # 系统核心
│   │   │   ├── orchestrator.py            #   顶层编排器
│   │   │   ├── state_machine.py           #   有限状态机
│   │   │   └── mode_controller.py         #   探索控制器 + 任务控制器
│   │   ├── agent/                         # 导航 Agent
│   │   │   ├── agent.py                   #   NavigationAgent 主类
│   │   │   ├── llm_client.py              #   LLM 客户端（DeepSeek/OpenAI）
│   │   │   ├── memory.py                  #   对话记忆
│   │   │   ├── models.py                  #   数据模型
│   │   │   ├── task_allocator.py          #   任务分配（Greedy/Hungarian）
│   │   │   └── task_manager.py            #   任务生命周期管理
│   │   ├── navigation/                    # 导航后端
│   │   │   ├── base.py                    #   BaseNavigator 抽象基类
│   │   │   ├── simulated.py               #   手动输入模拟器
│   │   │   ├── ros_single.py              #   单机 ROS move_base
│   │   │   ├── ros_multi.py               #   多机 ROS move_base
│   │   │   └── factory.py                 #   导航器工厂
│   │   ├── perception/                    # 感知管线
│   │   │   ├── semantic_map.py            #   语义地图（热重载 + 模糊匹配）
│   │   │   ├── map_converter.py           #   DMROS → nav 格式转换
│   │   │   ├── map_watcher.py             #   文件监听器（单机模式）
│   │   │   ├── map_merger.py              #   多机语义地图合并器
│   │   │   └── multi_robot_map_manager.py #   多机地图管理器（轮询+合并+转换）
│   │   ├── config/                        # 配置系统
│   │   │   ├── config.py                  #   Config 类（Profile 加载）
│   │   │   └── defaults.py                #   默认配置常量
│   │   ├── exceptions.py                  #   异常体系
│   │   └── logging_setup.py               #   日志配置
│   │
│   ├── srv/                               # ROS 服务定义
│   │   ├── SwitchMode.srv                 #   模式切换
│   │   ├── ExecuteTask.srv                #   提交任务
│   │   └── GetSystemStatus.srv            #   状态查询
│   └── msg/
│       └── SystemState.msg                # 系统状态消息
│
├── third_party/
│   └── DMROS/                             # DMROS 语义建图子模块
├── data/
│   └── semantic_map.json                  # 运行时语义地图
├── tests/                                 # 测试用例
├── requirements.txt                       # Python 依赖
└── setup.py                               # pip 安装
```

---

## 依赖环境

### 基础环境

- Python 3.8+
- pip

### Python 依赖

```
openai>=1.0.0
numpy>=1.20.0
scipy>=1.7.0
```

### ROS 环境（仿真与真机部署需要）

- ROS1 Noetic
- TurtleBot3 功能包：`turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_navigation`, `turtlebot3_description`
- `explore_lite`
- `gmapping`
- `move_base`
- `multirobot_map_merge`（多机地图合并，可选）

### DMROS 环境（语义建图需要）

- CUDA GPU（推荐）
- DMROS 自身的依赖（YOLO、FastSAM、SAM、CLIP 等，参见 `third_party/DMROS/` 文档）

---

## 安装

### 1. 克隆项目

```bash
cd /mnt/storage/ubuntu/workspace
git clone <repo_url> robot_nav_system
cd robot_nav_system
```

### 2. 安装 Python 依赖

```bash
pip install -r requirements.txt
```

### 3. 初始化 DMROS 子模块（如需语义建图）

```bash
cd third_party
git submodule add <dmros_repo_url> DMROS
cd ..
```

### 4. catkin 构建（如需 ROS 功能）

```bash
# 将项目链接到 catkin 工作空间
ln -s /mnt/storage/ubuntu/workspace/robot_nav_system/robot_nav_system ~/catkin_ws/src/robot_nav_system
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 配置系统

系统采用 **Profile 配置机制**：每种部署场景对应一个 JSON 配置文件，与内置默认值深度合并。

### 配置文件位置

```
robot_nav_system/config/profiles/
├── turtlebot3_sim.json    # TurtleBot3 Gazebo 仿真
├── manual_sim.json        # 手动输入模拟
└── real_robot.json        # 真机部署
```

### 关键配置项

```jsonc
{
  "profile": "turtlebot3_sim",          // Profile 名称

  "runtime": {
    "mode": "ros_multi",                // simulation | ros_single | ros_multi
    "use_ros": true,                    // 是否使用 ROS
    "auto_explore_on_startup": true,    // 启动后自动进入探索模式
    "resume_explore_after_task": true   // 任务完成后自动恢复探索
  },

  "robots": {
    "names": ["tb3_0", "tb3_1", "tb3_2"],
    "global_frame": "map",
    "base_frame_suffix": "base_footprint",
    "goal_timeout": 300.0,              // 导航目标超时 (秒)
    "server_timeout": 10.0,             // move_base 服务连接超时
    "default_positions": {              // 初始位置 (仿真模式使用)
      "tb3_0": [-7.0, -1.0],
      "tb3_1": [7.0, -1.0],
      "tb3_2": [0.5, 3.0]
    }
  },

  "llm": {
    "base_url": "https://api.deepseek.com",
    "api_key": "",                      // 填入 API Key 启用 LLM 规划
    "model": "deepseek-chat",
    "temperature": 0.2,
    "max_tokens": 1024,
    "timeout": 45
  },

  "dmros": {
    "enabled": true,                    // 是否启用 DMROS 语义建图
    "output_dir": "output/semantic",    // DMROS 输出目录
    "poll_interval": 3.0,               // 地图变化检测间隔 (秒)
    "use_llm_for_metadata": true        // 使用 LLM 生成中文元数据
  },

  "exploration": {
    "enabled": true,                    // 是否启用 explore-lite
    "launch_file": "exploration/multi_explore.launch"
  },

  "semantic_map": {
    "path": "data/semantic_map.json",   // 语义地图文件路径
    "auto_reload": true                 // 自动热重载
  }
}
```

### LLM 配置

系统支持任何 OpenAI 兼容 API。未配置 API Key 时自动启用 **fallback 关键词匹配模式**，无需网络即可运行基本的任务规划。

```bash
# 方式一：直接写入配置文件
# 编辑对应 profile JSON，设置 llm.api_key

# 方式二：环境变量（需自行在代码中适配）
export DEEPSEEK_API_KEY="sk-your-key-here"
```

---

## 三种部署场景

### 场景一：手动输入模拟（无 ROS）

适用于开发调试、演示、无 ROS 环境的场景。导航结果由用户手动输入 `success` / `fail`。

```bash
cd /mnt/storage/ubuntu/workspace/robot_nav_system
PYTHONPATH=robot_nav_system/src:$PYTHONPATH \
  python3 robot_nav_system/scripts/system_orchestrator_node.py --profile manual_sim
```

交互示例：

```
==================================================
  Robot Navigation System
  Profile: manual_sim
  Mode:    simulation
  Robots:  robot_0
  Map:     3 objects
==================================================
输入自然语言指令，或输入 help 查看帮助。

你> 去咖啡机拿杯咖啡送到沙发

[SIM-SINGLE] 步骤 1/2: navigate -> 咖啡机
  目标坐标: x=2.100, y=5.400, yaw=1.570
  所在房间: 茶水间  |  类别: 电器
  请输入结果 [success/fail]: success

[SIM-SINGLE] 步骤 2/2: navigate -> 沙发
  目标坐标: x=-3.200, y=1.500, yaw=0.000
  所在房间: 客厅  |  类别: 家具
  请输入结果 [success/fail]: success

Agent> 任务结束（单机）：共 2 步，成功 2 步，失败 0 步。
```

### 场景二：TurtleBot3 Gazebo 仿真

完整系统一键启动：Gazebo + SLAM + move_base + DMROS + Agent。

```bash
# 1. 设置环境
export TURTLEBOT3_MODEL=waffle_pi
source ~/catkin_ws/devel/setup.bash

# 2. 一键启动
roslaunch robot_nav_system full_system.launch

# 3. 或者分步启动（调试用）
# 终端 1: Gazebo + 机器人
roslaunch robot_nav_system gazebo_world.launch

# 终端 2: SLAM
roslaunch robot_nav_system multi_slam.launch

# 终端 3: Navigation
roslaunch robot_nav_system multi_move_base.launch

# 终端 4: Agent 节点
roslaunch robot_nav_system nav_agent_only.launch profile:=turtlebot3_sim
```

启动后系统自动进入探索模式，机器人开始自主探索环境。通过 ROS 服务或 REPL 提交任务：

```bash
# 通过 rostopic 查看系统状态
rostopic echo /system_orchestrator/system_state
```

### 场景三：真机部署

适用于搭载 RGBD 相机的自定义机器人平台。

**步骤 1：修改配置**

编辑 `config/profiles/real_robot.json`，修改以下关键项：

```jsonc
{
  "robots": {
    "names": ["robot"],           // 改为你的机器人名称
    "base_frame_suffix": "base_link"  // 改为你的 base frame
  },
  "ros": {
    "base_frame": "base_link"    // 与上面一致
  }
}
```

编辑 `config/dmros/real_robot.yaml`，修改 RGBD 话题名称：

```yaml
ros_topics:
  rgb_image: "/camera/color/image_raw"              # 你的 RGB 话题
  depth_image: "/camera/aligned_depth_to_color/image_raw"  # 你的深度话题
  camera_info: "/camera/color/camera_info"           # 你的相机参数话题
  odom: "/odom"                                      # 你的里程计话题
```

**步骤 2：启动**

```bash
# 确保机器人驱动、SLAM、move_base 已运行
roslaunch robot_nav_system nav_agent_only.launch profile:=real_robot
```

---

## 使用说明

### REPL 交互命令

| 命令 | 说明 |
|------|------|
| `<自然语言指令>` | 提交导航任务，自动判断单机/多机模式 |
| `explore` / `探索` | 启动自主探索模式 |
| `stop` / `停止` | 停止探索，回到空闲 |
| `status` / `状态` | 查看系统当前状态 |
| `interrupt` / `中断` | 中断当前正在执行的任务 |
| `help` / `帮助` | 显示帮助信息 |
| `exit` / `quit` / `结束` | 退出系统 |

### 自然语言指令示例

**单机任务**（一个机器人按顺序执行）：

```
你> 去咖啡机拿杯咖啡送到沙发
你> 依次经过电视柜和沙发
```

**多机任务**（多个机器人并行执行）：

```
你> 让三个机器人分别去咖啡机、沙发和电视柜
你> 同时派机器人去茶水间和客厅
```

**带依赖的任务**（部分子任务有先后顺序）：

```
你> 先去咖啡机接咖啡，完成后再送到沙发，同时另一个机器人去拿杯子
```

### LLM 自动规划

当配置了 API Key 后，LLM 会自动：

1. 分析指令，决定使用单机还是多机模式
2. 拆解为子任务列表
3. 设置子任务之间的依赖关系
4. 选择合适的任务分配策略
5. 执行完成后生成中文总结

未配置 API Key 时，使用基于关键词匹配的 fallback 规划器，能处理简单的直接目标导航。

---

## 系统状态机

系统运行时由有限状态机管理模式切换：

```
                          ┌──────────────┐
                          │ UNINITIALIZED│
                          └──────┬───────┘
                                 │ startup()
                                 ▼
                 ┌──────────►┌──────┐◄──────────┐
                 │           │ IDLE │            │
                 │           └──┬───┘            │
                 │              │                │
          error  │   ┌──────────┼──────────┐     │ completed
        handled  │   ▼          ▼          │     │
                 │ EXPLORING  TASK_RECEIVED│     │
                 │   │          │          │     │
                 │   │ task()   │ (auto)   │     │
                 │   └────►  PLANNING      │     │
                 │              │           │     │
                 │           EXECUTING      │     │
                 │              │           │     │
                 │           COMPLETED ─────┘     │
                 │              │                  │
                 └───── ERROR ◄┘                   │
                          │                        │
                          └────────────────────────┘

                    任意状态 ──── SHUTTING_DOWN (信号/退出)
```

**关键转换说明**：

- **EXPLORING → TASK_RECEIVED**：收到自然语言指令时触发。自动执行：
  1. 取消所有 move_base 导航目标
  2. 杀掉 explore-lite 进程
  3. 等待机器人停止运动
- **COMPLETED → EXPLORING**：若任务执行前处于探索状态，且配置了 `resume_explore_after_task: true`，则自动恢复探索
- **ERROR → IDLE**：任何错误自动恢复到空闲状态，不会导致系统崩溃

---

## 数据流

### 感知管线（探索时持续运行）

```
RGBD 传感器
    │
    ▼
DMROS Core (检测 + 3D 语义地图)
    │
    ▼
semantic_map.json (DMROS 格式)
    │  ┌─────────────────┐
    └─►│  MapWatcher      │ 后台轮询 (每 2-3 秒)
       │  检测文件变化     │
       └────────┬────────┘
                ▼
       ┌────────────────┐
       │  MapConverter   │ DMROS → nav 格式转换
       │  (快速/完整)    │
       └────────┬───────┘
                ▼
       semantic_map.json (nav 格式)
                │
                ▼
       ┌────────────────┐
       │  SemanticMap    │ 线程安全热重载
       │  (模糊匹配)    │
       └────────────────┘
```

### 任务执行管线

```
用户自然语言指令
    │
    ▼
┌───────────┐    ┌──────────┐    ┌────────────┐    ┌───────────┐
│ LLMClient │───►│ TaskPlan │───►│ Allocator  │───►│ Navigator │
│ (规划)    │    │ (子任务) │    │ (Greedy/   │    │ (执行)    │
│           │    │          │    │  Hungarian)│    │           │
└───────────┘    └──────────┘    └────────────┘    └─────┬─────┘
                                                         │
                                                    move_base
                                                         │
                                                    机器人运动
```

### 格式转换

**DMROS 输出格式** → **Nav Agent 格式**

```
DMROS:                              Nav Agent:
{                                   {
  "objects": [{                       "objects": [{
    "instance_id": "uuid",              "id": "chair_01",
    "class_name": "chair",              "name": "椅子",
    "bbox_2d": {                        "aliases": ["座椅", "坐椅"],
      "center": [3.2, 0.6],            "category": "家具",
      "extent": [1.0, 0.7]             "room": "客厅",
    },                                  "position": {
    "bbox_3d": {...}                      "x": 2.82,
  }]                                      "y": 1.59,
}                                         "yaw": -1.2
                                        },
                                        "description": "供人坐的家具"
                                      }]
                                    }
```

转换过程：

1. 从 DMROS 的 `bbox_2d.center` 提取物体位置
2. 计算无碰撞的机器人停靠位姿（考虑物体包围盒和已放置的停靠位）
3. （完整模式）通过 LLM 生成中文名称、别名、类别、房间推断
4. （快速模式）直接使用英文 class_name，跳过 LLM 调用

---

## 模块详解

### core/orchestrator.py — SystemOrchestrator

系统的顶层控制器，持有所有子组件：

| 方法 | 说明 |
|------|------|
| `startup()` | 初始化所有子系统（语义地图、LLM、导航器、Agent、感知管线、控制器） |
| `shutdown()` | 优雅关闭（停止探索、取消目标、停止监听） |
| `submit_task(instruction)` | 提交自然语言任务，自动处理模式切换 |
| `request_explore()` | 进入探索模式 |
| `request_idle()` | 停止探索，进入空闲 |
| `get_status()` | 返回系统状态快照 |
| `run_repl()` | 启动交互式 REPL |
| `handle_command(text)` | 路由命令到对应处理器 |

### core/state_machine.py — SystemStateMachine

线程安全的有限状态机，支持：

- 合法转换校验（非法转换抛出 `StateTransitionError`）
- `on_enter(state, callback)` — 进入状态时触发回调
- `on_exit(state, callback)` — 离开状态时触发回调
- `on_transition(from, to, callback)` — 特定转换时触发回调

### core/mode_controller.py — ExploreController

动态管理 explore-lite 的生命周期：

- `start_exploration()` — 优先使用 `roslaunch.parent.ROSLaunchParent` Python API 动态启动 launch 文件；不可用时回退到 `subprocess`
- `stop_exploration()` — 杀掉 explore-lite 进程 + 取消所有 move_base 目标

这是实现探索/任务模式无缝切换的关键设计：explore-lite **不在** `full_system.launch` 中静态包含，而是由 ExploreController 按需启停。

### agent/agent.py — NavigationAgent

核心 Agent，完整管线：

```
run_instruction(instruction)
    ├── plan_new_task()         # LLM 规划 → TaskPlan
    │   ├── llm.plan_tasks()   # 调用 LLM 或 fallback
    │   └── _validate_plan_with_map()  # 语义地图校验
    ├── execute_plan()          # 执行
    │   ├── _execute_single()   # 单机顺序执行
    │   └── _execute_multi()    # 多机并行执行
    │       ├── allocator.allocate()  # 任务分配
    │       └── navigator.execute_multi()  # 并行导航
    └── llm.summarize_task()    # 结果总结
```

### agent/task_allocator.py — 任务分配器

| 分配器 | 算法 | 复杂度 | 说明 |
|--------|------|--------|------|
| `GreedyAllocator` | 贪心最近优先 | O(n*m) | 默认策略，每个任务分配给最近的空闲机器人 |
| `HungarianAllocator` | 匈牙利算法 | O(n³) | 全局最优分配，最小化总距离 |

两者都支持 LLM 预分配（`assigned_robot` 字段）。

### navigation/ — 导航后端

| 后端 | 使用场景 | ROS | 特点 |
|------|---------|-----|------|
| `SimulatedNavigator` | 开发调试 | 不需要 | 用户手动输入 success/fail |
| `ROSSingleNavigator` | 真机单机 | 需要 | TF + move_base actionlib |
| `ROSMultiNavigator` | 仿真多机 | 需要 | 命名空间隔离 + TF2 + 多线程并行 + 依赖同步 |

所有导航器都实现 `cancel_all_goals()` 方法，用于探索→任务的模式切换。

`NavigatorFactory` (`factory.py`) 根据配置中的 `runtime.mode` 自动选择对应的导航器。

### perception/semantic_map.py — 语义地图

增强版语义地图，支持：

- **线程安全热重载**：`reload()` 方法可在后台线程中安全调用
- **版本追踪**：`version` 属性单调递增，便于检测变化
- **空地图支持**：`SemanticMap.empty()` 创建空地图，探索过程中逐步填充
- **模糊匹配**：支持精确匹配、子串匹配、SequenceMatcher 相似度、分词匹配

### perception/map_converter.py — 格式转换器

两种转换模式：

| 方法 | LLM | 速度 | 输出质量 | 适用场景 |
|------|-----|------|---------|---------|
| `convert_fast()` | 不使用 | 快 | 英文名称 | 探索过程中实时转换 |
| `convert()` | 使用 | 慢 | 中文名称+别名+类别+房间 | 探索结束后精细转换 |

### perception/map_watcher.py — 文件监听器

后台守护线程，周期性轮询 DMROS 输出目录：

1. 检测 `semantic_map.json` 的修改时间变化
2. 触发 `MapConverter.convert_fast()` 格式转换
3. 调用 `SemanticMap.reload()` 热重载

采用轮询而非 inotify 的原因：DMROS 使用原子写入（临时文件 + rename），轮询方式更简单可靠。

### perception/map_merger.py — 多机语义地图合并器

`SemanticMapMerger` 负责将多台机器人的本地语义地图去重合并为全局地图：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `merge_distance` | 1.0m | 同类物体视为同一物理实体的最大距离 |

核心类 `ObjectCluster` 管理对同一物理物体的多次观测：
- 动态合并新观测（距离检查）
- 位置融合（算术平均 + 圆周平均角度）
- 追踪来源机器人和观测次数

### perception/multi_robot_map_manager.py — 多机地图管理器

`MultiRobotMapManager` 是多机模式的核心感知组件，替代单机模式的 `MapWatcher`：

```python
MultiRobotMapManager(
    robots=["tb3_0", "tb3_1", "tb3_2"],
    base_output_dir=Path("output"),         # 每机器人: output/{robot}/semantic/
    global_dmros_map_path=Path("output/merged/semantic_map.json"),
    nav_map_path=Path("data/semantic_map.json"),
    converter=MapConverter(...),
    semantic_map=SemanticMap(...),
    merge_distance=1.0,
    poll_interval=3.0,
)
```

后台线程工作流程：

```
每 3 秒轮询
    │
    ├─ 检查每台机器人的 semantic_map.json 修改时间
    │   如果任一变化 →
    │
    ├─ 加载所有机器人的最新地图
    ├─ SemanticMapMerger.merge() 去重合并
    ├─ 写入 output/merged/semantic_map.json
    ├─ MapConverter.convert_fast() → data/semantic_map.json
    └─ SemanticMap.reload()
```

在 `ros_multi` 模式下，`SystemOrchestrator` 自动选择 `MultiRobotMapManager` 而非 `MapWatcher`。

---

## 异常处理

### 异常层级

```
RobotNavError (基类)
├── ConfigError              # 配置文件缺失或格式错误
├── SemanticMapError         # 语义地图加载/匹配失败
├── LLMError                 # LLM API 调用或响应解析失败
├── TaskPlanningError        # 任务规划失败
├── NavigationError          # 导航目标执行失败
├── AllocationError          # 任务分配失败
├── StateTransitionError     # 非法状态转换
├── ExplorationError         # explore-lite 启停失败
└── MapConversionError       # 格式转换失败
```

### 各组件异常处理策略

| 组件 | 故障 | 处理方式 |
|------|------|---------|
| **LLM API** | 超时/网络错误/格式异常 | 自动使用 fallback 关键词匹配模式 |
| **语义地图** | 文件不存在 | 创建空地图，探索后自动填充 |
| **语义地图** | reload 失败 | 保留上一版本，记录警告日志 |
| **地图转换** | DMROS 输出格式异常 | 跳过本次转换周期，保留旧版本 |
| **explore-lite** | 启动失败 | 转入 ERROR 状态，用户可重试 |
| **explore-lite** | 杀死失败 | 视为成功（幂等操作） |
| **move_base** | 目标超时 | 记录失败，继续下一子任务 |
| **move_base** | 服务不可用 | 标记所有任务为 SERVER_TIMEOUT |
| **TF 查找** | 变换不可用 | 返回空位置字典 |
| **状态转换** | 非法转换 | 拒绝转换，记录日志，保持当前状态 |
| **编排器** | 未捕获异常 | 顶层 try/except 捕获，尝试优雅关闭 |

### 设计原则

1. **系统永不崩溃**：所有异常都在合适的层级被捕获处理
2. **自动恢复**：错误后自动转入 ERROR → IDLE，用户可重试
3. **保守策略**：感知管线出错时保留上一版本数据，不会清空语义地图
4. **部分成功**：多步任务中某步失败不影响其他步骤执行，最终汇总所有结果

---

## 常见问题

### Q: 没有 API Key 能用吗？

可以。系统会自动启用 fallback 关键词匹配模式。它通过匹配指令中的物体名称/别名/类别/房间来生成子任务。适合简单的直接目标导航，复杂的多步任务建议配置 API Key。

### Q: 如何添加新的物体到语义地图？

**方式一（自动）**：启动探索模式，DMROS 会实时检测环境中的物体并更新语义地图。

**方式二（手动）**：直接编辑 `data/semantic_map.json`：

```json
{
  "objects": [
    {
      "id": "my_object_01",
      "name": "我的物体",
      "aliases": ["别名1", "别名2"],
      "category": "家具",
      "room": "客厅",
      "position": {"x": 1.0, "y": 2.0, "yaw": 0.0},
      "description": "简短描述"
    }
  ]
}
```

### Q: 如何适配其他机器人平台？

1. 复制 `config/profiles/real_robot.json` 并修改
2. 修改 `robots.base_frame_suffix` 为你的 base frame 名称
3. 修改 `config/dmros/real_robot.yaml` 中的 RGBD 话题名称
4. 修改 `converter.robot_length` 和 `converter.robot_width` 为你的机器人尺寸
5. 如果使用不同的 SLAM 或导航栈，调整对应的 launch 文件

### Q: explore-lite 为什么不在 full_system.launch 中？

这是核心设计决策。explore-lite 和任务导航都通过 move_base 发送目标，如果同时运行会产生冲突。因此 explore-lite 由 `ExploreController` **动态启停**：

- 探索模式：ExploreController 启动 explore-lite 进程
- 收到任务：ExploreController 杀掉 explore-lite + 取消所有 move_base 目标
- 任务完成：ExploreController 重新启动 explore-lite

### Q: 多机器人的任务依赖如何工作？

LLM 在规划时会自动检测子任务之间的逻辑依赖，并通过 `depends_on` 字段表示（值为前置任务的索引）。执行时：

- `ROSMultiNavigator` 为每个依赖关系创建 `threading.Event`
- 前置任务完成后 `.set()` 事件
- 后续任务通过 `.wait()` 阻塞等待
- 无依赖的任务之间完全并行

### Q: 语义地图的模糊匹配是如何工作的？

匹配算法综合使用四种策略，取最高分：

| 策略 | 分数 | 说明 |
|------|------|------|
| 精确匹配 | 1.0 | query == candidate（忽略大小写） |
| 子串匹配 | 0.9 | query 是 candidate 的子串，或反之 |
| 分词匹配 | 0.72 | query 的任一词在某个 candidate 中出现 |
| 序列匹配 | 0.0-1.0 | SequenceMatcher 相似度 |

默认阈值 0.48，低于此分数视为不匹配。匹配范围包括物体的 name、aliases、category、room。
