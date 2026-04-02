# 基于大语言模型的多机器人自主探索与协同导航系统

## 摘要

本文提出了一种集成自主探索与大语言模型（LLM）驱动任务执行的多机器人导航系统。系统采用"探索—执行"双模态架构：在探索模式下，多台机器人基于前沿探索算法自主构建环境的几何地图与语义地图；在接收到自然语言指令后，系统通过有限状态机无缝切换至任务执行模式，利用 LLM 进行任务分解、多机任务分配与依赖调度。本文重点阐述了系统的核心算法设计，包括基于空间距离的多源语义地图融合算法、贪心与匈牙利多机任务分配策略、基于模糊字符串匹配的语义目标解析机制，以及无碰撞停靠位姿生成算法。系统支持 TurtleBot3 仿真、手动模拟和真机部署三种运行场景，具备完善的异常处理与自动恢复能力。实验表明，该系统能够在未知环境中实现高效的自主探索与准确的自然语言导航任务执行。

**关键词：** 多机器人系统；大语言模型；语义导航；任务分配；自主探索；语义地图融合

---

## 1 引言

### 1.1 研究背景

随着移动机器人技术的不断发展，机器人在服务、物流、搜救等领域的应用日益广泛。在实际部署中，机器人面临的核心挑战之一是如何在未知环境中自主构建对环境的语义理解，并根据用户的自然语言指令完成复杂的导航任务。

传统的机器人导航系统通常采用预建地图加目标点导航的方式，需要人工标注导航目标。这种方式存在两个显著局限：第一，地图的构建与目标的标注需要大量人工参与；第二，用户只能通过坐标或预定义的目标名称下达指令，缺乏自然交互能力。

近年来，大语言模型（Large Language Model, LLM）的快速发展为机器人自然语言交互提供了新的可能。LLM 具备强大的自然语言理解、任务分解和推理能力，能够将用户的模糊指令转化为具体的机器人操作序列。然而，如何将 LLM 的规划能力与机器人的感知、建图和导航执行系统有效集成，仍然是一个值得深入研究的问题。

### 1.2 研究内容

本文设计并实现了一个完整的多机器人导航系统，其核心贡献包括：

1. **双模态系统架构**：提出探索模式与任务执行模式的双模态切换机制，通过有限状态机实现无缝模式转换，解决了自主探索与任务导航的 move_base 资源冲突问题。

2. **多源语义地图融合算法**：设计了基于空间距离聚类的多机器人语义地图实时融合方法，支持增量更新与去重合并。

3. **LLM 驱动的任务规划与分配**：利用 LLM 实现自然语言指令到导航子任务的自动分解，结合贪心与匈牙利算法完成多机任务的最优分配。

4. **层次化异常处理机制**：构建了从 LLM 回退到导航恢复的多层容错体系，确保系统在各类故障条件下的鲁棒运行。

### 1.3 论文组织

本文其余部分组织如下：第 2 节介绍系统整体架构；第 3 节详细描述核心算法设计；第 4 节阐述多机地图融合机制；第 5 节介绍 LLM 驱动的任务规划与分配；第 6 节讨论系统状态机与模式切换；第 7 节介绍系统部署与实验；第 8 节总结全文。

---

## 2 系统架构

### 2.1 总体架构

系统采用分层架构设计，自底向上分为感知层、导航层、Agent 层和编排层，如图 1 所示。

```
┌────────────────────────────────────────────────────────────────┐
│                    编排层 (Orchestration Layer)                  │
│   SystemOrchestrator · SystemStateMachine · ModeControllers    │
├────────────────────────────────────────────────────────────────┤
│                    Agent 层 (Agent Layer)                       │
│   NavigationAgent · LLMClient · TaskAllocator · TaskManager    │
├────────────────────────────────────────────────────────────────┤
│                    导航层 (Navigation Layer)                    │
│   SimulatedNavigator · ROSSingleNavigator · ROSMultiNavigator  │
├────────────────────────────────────────────────────────────────┤
│                    感知层 (Perception Layer)                    │
│   DMROS · SemanticMap · MapConverter · MapMerger · MapWatcher  │
└────────────────────────────────────────────────────────────────┘
```

**图 1** 系统分层架构

各层职责如下：

- **感知层**：负责环境的语义理解，包括 RGBD 数据处理（DMROS）、语义地图的存储与检索（SemanticMap）、多源地图格式转换（MapConverter）与实时融合（MapMerger）。
- **导航层**：封装底层导航执行，通过抽象接口 `BaseNavigator` 支持模拟执行、ROS 单机导航和 ROS 多机并行导航三种后端。
- **Agent 层**：核心决策层，集成 LLM 任务规划（LLMClient）、语义目标匹配、多机任务分配（TaskAllocator）和执行状态跟踪（TaskManager）。
- **编排层**：系统的顶层控制器，管理探索与任务执行两种运行模式的切换（SystemStateMachine），协调各子系统的生命周期。

### 2.2 数据模型

系统定义了如下核心数据结构：

**定义 1（语义物体）**：语义物体 $o$ 定义为七元组：

$$o = (id, name, A, cat, room, \mathbf{p}, desc)$$

其中 $id$ 为唯一标识符，$name$ 为显示名称，$A = \{a_1, a_2, \ldots, a_k\}$ 为别名集合，$cat$ 为类别，$room$ 为所在房间，$\mathbf{p} = (x, y, \theta) \in \mathbb{R}^2 \times [-\pi, \pi)$ 为位姿（平面坐标与朝向角），$desc$ 为描述文本。

**定义 2（子任务）**：子任务 $s$ 定义为六元组：

$$s = (action, target, reason, obj\_id, dep, robot)$$

其中 $action$ 为动作类型（如 navigate），$target$ 为目标名称，$reason$ 为 LLM 给出的执行原因，$obj\_id$ 为经语义地图验证后的目标物体 ID，$dep \in \mathbb{Z}_{\geq 0} \cup \{\varnothing\}$ 为依赖的前置子任务索引，$robot \in \mathcal{R} \cup \{\varnothing\}$ 为预分配的机器人。

**定义 3（任务计划）**：任务计划 $P$ 定义为五元组：

$$P = (inst, mode, \mathbf{S}, strat, notes)$$

其中 $inst$ 为原始自然语言指令，$mode \in \{single, multi\}$ 为执行模式，$\mathbf{S} = (s_1, s_2, \ldots, s_n)$ 为有序子任务列表，$strat \in \{greedy, hungarian\}$ 为分配策略，$notes$ 为补充说明。

**定义 4（执行记录）**：执行记录 $r$ 定义为七元组：

$$r = (step, action, target, obj\_id, success, msg, robot)$$

记录每一步导航的结果，$success \in \{True, False\}$ 指示是否成功。

### 2.3 软件模块组成

系统共包含 18 个 Python 模块，按功能分为四个包。表 1 列出了各模块的类名、代码行数与核心职责。

**表 1** 系统模块构成

| 包 | 模块 | 核心类 | 职责 |
|----|------|--------|------|
| core | orchestrator.py | SystemOrchestrator | 系统生命周期管理与顶层协调 |
| core | state_machine.py | SystemStateMachine | 有限状态机，模式切换与回调 |
| core | mode_controller.py | ExploreController, TaskController | 探索与任务模式的具体控制 |
| agent | agent.py | NavigationAgent | 核心 Agent，集成规划—分配—执行—总结管线 |
| agent | llm_client.py | LLMClient | LLM API 调用与响应解析 |
| agent | task_allocator.py | GreedyAllocator, HungarianAllocator | 多机任务分配算法 |
| agent | task_manager.py | TaskManager | 任务生命周期跟踪 |
| agent | memory.py | DialogueMemory | 有界对话记忆缓冲 |
| agent | models.py | SemanticObject, SubTask, TaskPlan 等 | 数据模型定义 |
| navigation | base.py | BaseNavigator | 导航器抽象基类 |
| navigation | simulated.py | SimulatedNavigator | 手动输入模拟导航 |
| navigation | ros_single.py | ROSSingleNavigator | 单机 ROS move_base 导航 |
| navigation | ros_multi.py | ROSMultiNavigator | 多机 ROS 并行导航 |
| navigation | factory.py | build_navigator() | 导航器工厂函数 |
| perception | semantic_map.py | SemanticMap | 语义地图存储、检索与热重载 |
| perception | map_converter.py | MapConverter | DMROS 格式到 Agent 格式的转换 |
| perception | map_merger.py | SemanticMapMerger | 多源语义地图融合 |
| perception | multi_robot_map_manager.py | MultiRobotMapManager | 多机地图管理与融合调度 |

---

## 3 核心算法设计

### 3.1 语义目标模糊匹配算法

用户通过自然语言指令指定导航目标时，目标名称可能与语义地图中的标准名称存在差异（如"咖啡机"与"咖啡机台"，"sofa"与"沙发"）。为此，系统设计了多策略融合的模糊匹配算法。

设用户查询字符串为 $q$，语义地图中物体 $o_i$ 的关键词集合为 $K_i = \{name_i\} \cup A_i \cup \{cat_i, room_i\}$。对于 $q$ 与候选关键词 $c \in K_i$，定义字符串相似度函数：

$$\sigma(q, c) = \begin{cases} 1.0, & \text{if } q^L = c^L \\ 0.9, & \text{if } q^L \subseteq c^L \text{ or } c^L \subseteq q^L \\ \text{SequenceMatcher}(q^L, c^L), & \text{otherwise} \end{cases}$$

其中上标 $L$ 表示转换为小写形式，$\subseteq$ 表示子串关系，SequenceMatcher 为 Ratcliff/Obershelp 序列相似度算法，其值域为 $[0, 1]$。

对于物体 $o_i$，其关键词匹配得分为：

$$\sigma_{\max}(q, o_i) = \max_{c \in K_i} \sigma(q, c)$$

进一步引入分词匹配奖励：将查询 $q$ 按空格分词得到词元集合 $T_q = \{t_1, t_2, \ldots\}$，若存在词元 $t \in T_q$ 使得 $t \subseteq c^L$ 对某 $c \in K_i$ 成立，则：

$$\sigma_{\text{tok}}(q, o_i) = 0.72$$

最终匹配得分为：

$$S(q, o_i) = \max\{\sigma_{\max}(q, o_i), \sigma_{\text{tok}}(q, o_i)\}$$

系统返回得分最高的物体，当且仅当 $S(q, o^*) \geq \tau$（默认阈值 $\tau = 0.48$）时视为匹配成功：

$$o^* = \arg\max_{o_i \in \mathcal{O}} S(q, o_i), \quad \text{match} \Leftrightarrow S(q, o^*) \geq \tau$$

该算法的时间复杂度为 $O(|\mathcal{O}| \cdot |K_{\max}|)$，其中 $|\mathcal{O}|$ 为语义物体总数，$|K_{\max}|$ 为单物体最大关键词数。

### 3.2 无碰撞停靠位姿生成算法

当 DMROS 检测到环境中的物体后，需要为机器人计算一个距物体适当距离、无碰撞且面向物体的停靠位姿（stop pose）。

设物体中心坐标为 $(x_o, y_o)$，朝向为 $\theta_o$，包围盒半长为 $(e_x, e_y)$，机器人底盘尺寸为长 $L$、宽 $W$，安全间距为 $m$。已放置停靠位的集合为 $\mathcal{P} = \{(p_1^x, p_1^y), \ldots\}$。

**基础偏移距离**：

$$d_0 = \frac{1}{2}\max(e_x, e_y) + \frac{1}{2}\max(L, W) + m$$

**候选位置生成**：在 $N_r = 8$ 个半径层和 $N_a = 8$ 个角度方向上生成候选点：

$$r_k = d_0 + 0.2k, \quad k = 0, 1, \ldots, N_r - 1$$

$$\alpha_j \in \left\{\theta_o + \pi, \theta_o + \frac{\pi}{2}, \theta_o - \frac{\pi}{2}, \theta_o, \theta_o + \frac{3\pi}{4}, \theta_o - \frac{3\pi}{4}, \theta_o + \frac{\pi}{4}, \theta_o - \frac{\pi}{4}\right\}$$

对于每个候选点 $(c_x, c_y) = (x_o + r_k \cos\alpha_j, y_o + r_k \sin\alpha_j)$，执行两项冲突检测：

**检测 1（已有停靠位冲突）**：定义机器人包络圆半径 $R = \frac{1}{2}\sqrt{L^2 + W^2}$，最小间距 $d_{\min} = 2R + m$。若存在 $(p^x, p^y) \in \mathcal{P}$ 使得：

$$\sqrt{(c_x - p^x)^2 + (c_y - p^y)^2} < d_{\min}$$

则该候选位置存在冲突，予以排除。

**检测 2（物体包围盒重叠）**：采用轴对齐包围盒（AABB）重叠检测：

$$|c_x - x_o| \leq \frac{e_x}{2} + \frac{L}{2} + m \quad \wedge \quad |c_y - y_o| \leq \frac{e_y}{2} + \frac{W}{2} + m$$

若上式成立，则候选位置与物体重叠，予以排除。

**朝向计算**：对于通过检测的候选位置 $(c_x, c_y)$，令机器人面向物体：

$$\theta_{\text{face}} = \text{atan2}(y_o - c_y, x_o - c_x)$$

算法返回第一个通过所有检测的三元组 $(c_x, c_y, \theta_{\text{face}})$。若所有候选位置均被排除，则使用回退策略：

$$(c_x^{\text{fb}}, c_y^{\text{fb}}) = (x_o + d_0 + 1.0, y_o), \quad \theta^{\text{fb}} = \text{atan2}(y_o - c_y^{\text{fb}}, x_o - c_x^{\text{fb}})$$

### 3.3 对话记忆机制

为使 LLM 具备上下文感知能力，系统维护一个有界环形缓冲区 $\mathcal{M}$，容量为 $2H$ 条消息（$H$ 为最大对话轮数，默认 $H = 8$）。每次 LLM 调用时，将 $\mathcal{M}$ 中的最近消息作为对话历史传入，使 LLM 能够理解上下文语境中的指代、省略等语言现象。

$$\mathcal{M} = \text{deque}(\text{maxlen}=2H), \quad |\mathcal{M}| \leq 2H$$

当缓冲区满时，最早的消息自动被丢弃（FIFO 策略）。

---

## 4 多机语义地图融合

### 4.1 问题定义

在多机器人协同探索场景中，$N$ 台机器人 $\{R_1, R_2, \ldots, R_N\}$ 各自携带 RGBD 传感器独立探索环境，每台机器人通过 DMROS 系统产生各自的局部语义地图。设第 $i$ 台机器人在时刻 $t$ 的局部语义地图为 $\mathcal{M}_i^t = \{o_{i,1}^t, o_{i,2}^t, \ldots\}$，其中每个物体 $o_{i,j}^t$ 包含类别名 $c_{i,j}$、位置 $\mathbf{p}_{i,j} = (x_{i,j}, y_{i,j}, \theta_{i,j})$ 和包围盒尺寸 $\mathbf{e}_{i,j} = (e_x, e_y)$。

地图融合的目标是将所有局部地图合并为全局语义地图：

$$\mathcal{G}^t = \text{Merge}(\mathcal{M}_1^t, \mathcal{M}_2^t, \ldots, \mathcal{M}_N^t)$$

使得：（1）同一物理物体在全局地图中仅出现一次（去重）；（2）物体位置为多次观测的最优估计（融合）；（3）融合过程为增量式，支持实时更新。

### 4.2 基于空间距离聚类的融合算法

#### 4.2.1 算法概述

本文提出的语义地图融合算法基于"同类别 + 空间邻近"的双重准则进行去重：当且仅当两个物体属于同一类别且空间距离不超过阈值 $d_{\text{merge}}$ 时，视为同一物理实体的不同观测。

#### 4.2.2 物体聚类

设 $\mathcal{C}$ 为当前聚类集合，初始为空。对来自所有机器人的每个物体观测 $o = (c, \mathbf{p}, \mathbf{e})$，执行以下操作：

**步骤 1**：在 $\mathcal{C}$ 中查找同类别 $c$ 的所有聚类 $\mathcal{C}_c = \{C \in \mathcal{C} \mid C.\text{class} = c\}$。

**步骤 2**：对每个聚类 $C \in \mathcal{C}_c$，检查 $o$ 是否可以合并入 $C$。合并条件为存在 $C$ 中的某个已有观测 $(r', o')$ 使得：

$$d(\mathbf{p}, \mathbf{p}') = \sqrt{(x - x')^2 + (y - y')^2} \leq d_{\text{merge}}$$

**步骤 3**：若找到满足条件的聚类 $C^*$，则将 $o$ 加入 $C^*$；否则创建新聚类 $\{o\}$ 并加入 $\mathcal{C}$。

**算法 1** 多源语义地图融合

```
输入: robot_maps = {R_i: [o_{i,1}, ..., o_{i,n_i}]}, d_merge
输出: merged_objects

1:  clusters ← 按类别索引的空字典
2:  for each (R_i, objects) in robot_maps do
3:      for each o in objects do
4:          c ← o.class_name
5:          merged ← false
6:          for each cluster C in clusters[c] do
7:              if C.try_merge(o, R_i, d_merge) then
8:                  merged ← true
9:                  break
10:         if not merged then
11:             clusters[c].append(new_cluster(c, o, R_i))
12: merged_objects ← []
13: for each (c, cluster_list) in clusters do
14:     for each C in cluster_list do
15:         p̄ ← C.merged_position()     // 公式 (1)-(3)
16:         ē ← C.best_extent()          // 取面积最大的包围盒
17:         merged_objects.append((c, p̄, ē, C.sources))
18: return merged_objects
```

#### 4.2.3 位置融合

对于聚类 $C$ 中的 $M$ 个观测 $\{(r_1, o_1), (r_2, o_2), \ldots, (r_M, o_M)\}$，每个观测的位置为 $\mathbf{p}_j = (x_j, y_j, \theta_j)$。融合位置 $\bar{\mathbf{p}} = (\bar{x}, \bar{y}, \bar{\theta})$ 的计算方式如下：

**平面坐标取算术均值**：

$$\bar{x} = \frac{1}{M}\sum_{j=1}^{M} x_j, \quad \bar{y} = \frac{1}{M}\sum_{j=1}^{M} y_j \tag{1}$$

**朝向角取圆周均值**。由于角度具有周期性（$-\pi$ 与 $\pi$ 表示相同方向），直接算术平均会导致错误结果。采用基于正弦/余弦分量的圆周均值：

$$\bar{\theta} = \text{atan2}\left(\frac{1}{M}\sum_{j=1}^{M}\sin\theta_j, \quad \frac{1}{M}\sum_{j=1}^{M}\cos\theta_j\right) \tag{2}$$

**包围盒选择**：取面积最大的观测包围盒，以获得对物体尺寸的最保守估计：

$$\bar{\mathbf{e}} = \arg\max_{j \in \{1,\ldots,M\}} (e_{x,j} \cdot e_{y,j}) \tag{3}$$

#### 4.2.4 复杂度分析

设 $N$ 台机器人共产生 $n$ 个物体观测，共形成 $K$ 个类别，类别 $c$ 中有 $n_c$ 个观测和 $k_c$ 个聚类。合并检测的总次数为：

$$T = \sum_{c} \sum_{j=1}^{n_c} k_c^{(j)} \leq \sum_{c} n_c \cdot k_c$$

最坏情况下 $k_c = n_c$（所有物体互不重复），则 $T = O(n^2/K)$。实际场景中聚类数远小于观测数，算法效率较高。

### 4.3 几何地图融合

几何地图（occupancy grid）的融合由 ROS `multirobot_map_merge` 功能包完成。该功能包基于特征匹配方法对各机器人独立构建的占用栅格地图进行对齐与合并，生成全局几何地图。本系统通过 launch 文件配置实现自动集成。

### 4.4 融合管线架构

完整的多机地图融合管线如图 2 所示。

```
Robot_0                      Robot_1                      Robot_2
  │                            │                            │
  ├── SLAM ────────┐           ├── SLAM ────────┐           ├── SLAM ────────┐
  │  occupancy_grid│           │  occupancy_grid│           │  occupancy_grid│
  │                ▼           │                ▼           │                ▼
  │        multirobot_map_merge (ROS)                       │
  │                │                                        │
  │           全局几何地图                                    │
  │                                                         │
  ├── DMROS ───┐               ├── DMROS ───┐               ├── DMROS ───┐
  │  local     │               │  local     │               │  local     │
  │  semantic  │               │  semantic  │               │  semantic  │
  │  map       │               │  map       │               │  map       │
  │            ▼               │            ▼               │            ▼
  │     output/R_0/            │     output/R_1/            │     output/R_2/
  │     semantic_map.json      │     semantic_map.json      │     semantic_map.json
  │            │               │            │               │            │
  │            └───────────────┴────────────┴───────────────┘            │
  │                            │                                        │
  │                 MultiRobotMapManager                                │
  │                   │ (每 T_poll 秒轮询)                               │
  │                   ├── 检测文件修改时间变化                             │
  │                   ├── 加载所有局部语义地图                             │
  │                   ├── SemanticMapMerger.merge()                     │
  │                   ├── MapConverter.convert_fast()                   │
  │                   └── SemanticMap.reload()                          │
  │                            │                                        │
  │                   全局语义地图                                        │
  │                   data/semantic_map.json                             │
  └──────────────────► NavigationAgent ◄────────────────────────────────┘
```

**图 2** 多机地图融合管线架构

`MultiRobotMapManager` 以后台守护线程运行，以固定周期 $T_{\text{poll}}$（默认 3 秒）轮询各机器人的局部语义地图文件。当检测到任一文件的修改时间戳发生变化时，触发完整的融合—转换—重载流程。

---

## 5 LLM 驱动的任务规划与分配

### 5.1 任务规划

#### 5.1.1 LLM 规划接口

当用户输入自然语言指令 $inst$ 后，系统构造包含以下信息的 prompt 发送至 LLM：

- 可用机器人列表 $\mathcal{R} = \{R_1, R_2, \ldots, R_N\}$
- 语义地图中的物体摘要 $\{(id_i, name_i, aliases_i, cat_i, room_i)\}$
- 当前任务进度快照（已完成 / 进行中 / 待执行）
- 最近 $2H$ 条对话历史

LLM 被要求输出严格 JSON 格式的任务计划 $P$，包含执行模式（单机/多机）、分配策略、子任务列表及依赖关系。

#### 5.1.2 LLM 决策规则

LLM 通过系统提示词中的如下规则进行模式判断：

1. 单一机器人按顺序访问多个地点 → $mode = single$
2. 多台机器人各自前往不同地点或并行工作 → $mode = multi$
3. 用户明确使用"多机器人""分别""并行"等关键词 → $mode = multi$
4. 用户指定具体机器人执行具体任务 → $mode = multi$ 且设置 $assigned\_robot$

#### 5.1.3 回退规划器

当 LLM API 不可用时（网络中断或未配置 API Key），系统自动启用基于关键词匹配的回退规划器。该规划器通过检测指令中的多机关键词集合 $\mathcal{K}_{\text{multi}} = \{$"多机", "多个机器人", "分别", "并行", "同时"$\}$ 判断模式，并将指令与语义地图中所有物体的 $name$、$aliases$、$category$、$room$ 进行子串匹配，筛选出目标物体生成子任务。

### 5.2 任务计划验证

LLM 生成的原始计划需经语义地图验证。对于计划中的每个子任务 $s_i$，使用模糊匹配算法（第 3.1 节）在语义地图中查找最佳匹配物体：

$$o_i^*, S_i = \text{match}(s_i.target, \mathcal{O})$$

若 $S_i \geq \tau$，则将 $s_i$ 的目标绑定到 $o_i^*$；否则，在严格模式（$strict\_map\_only = True$）下丢弃该子任务。

### 5.3 多机任务分配

#### 5.3.1 问题形式化

给定 $n$ 个已验证的子任务 $\mathbf{S} = (s_1, \ldots, s_n)$、$m$ 台可用机器人 $\mathcal{R} = \{R_1, \ldots, R_m\}$，以及各机器人的当前位置 $\{(x_{R_i}, y_{R_i})\}$ 和各子任务目标位置 $\{(x_{s_j}, y_{s_j})\}$，定义代价矩阵 $\mathbf{C} \in \mathbb{R}^{m \times n}$：

$$C_{ij} = \sqrt{(x_{R_i} - x_{s_j})^2 + (y_{R_i} - y_{s_j})^2} \tag{4}$$

任务分配的目标是找到分配映射 $\phi: \{1,\ldots,n\} \rightarrow \{1,\ldots,m\}$，优化特定目标函数。

#### 5.3.2 贪心分配算法

贪心分配器采用最近优先策略，按任务顺序逐一分配：

**算法 2** 贪心任务分配

```
输入: 子任务集 S, 目标集 T, 机器人位置 Pos, 预分配集 Pre
输出: 分配结果 φ

1:  φ ← {R_i: [] for R_i in R}
2:  remaining ← []
3:  for i = 1 to n do
4:      if s_i.robot ∈ R then
5:          φ[s_i.robot].append(i)      // 尊重 LLM 预分配
6:      else
7:          remaining.append(i)
8:  available ← 未被预分配占用的机器人集合
9:  for each i in remaining do
10:     R* ← arg min_{R ∈ available} C(R, s_i)
11:     φ[R*].append(i)
12:     available.remove(R*)
13:     if available = ∅ then
14:         available ← R              // 循环复用
15: return φ
```

该算法的时间复杂度为 $O(n \cdot m)$。

#### 5.3.3 匈牙利最优分配算法

匈牙利分配器通过求解最小权完美匹配获得全局最优分配：

$$\phi^* = \arg\min_{\phi} \sum_{j=1}^{n} C_{\phi(j), j} \tag{5}$$

求解过程：

1. 处理 LLM 预分配的子任务（不参与优化）
2. 对剩余 $n'$ 个子任务和 $m$ 个机器人，构建 $\max(n', m) \times \max(n', m)$ 的代价矩阵（以 $10^6$ 填充）
3. 调用 `scipy.optimize.linear_sum_assignment` 求解
4. 提取有效分配对（排除填充行/列）

匈牙利算法的时间复杂度为 $O(\max(n,m)^3)$，保证找到使总移动距离最小的最优分配。

#### 5.3.4 两种策略的对比

**表 2** 分配策略比较

| 维度 | 贪心 (Greedy) | 匈牙利 (Hungarian) |
|------|:---:|:---:|
| 时间复杂度 | $O(n \cdot m)$ | $O(\max(n,m)^3)$ |
| 最优性 | 局部最优 | 全局最优 |
| 额外依赖 | 无 | numpy, scipy |
| 适用场景 | 实时性要求高 | 任务数较多时 |
| 预分配支持 | 支持 | 支持 |

### 5.4 依赖调度

对于多机任务执行，子任务之间可能存在先后依赖关系（如"先拿咖啡，再送到沙发"）。LLM 在规划时通过 `depends_on` 字段描述依赖关系，形成有向无环图（DAG）。

系统使用线程同步原语 `threading.Event` 实现依赖调度：

1. 为每个子任务 $s_i$ 创建完成事件 $E_i$
2. 各机器人在独立线程中执行分配到的任务
3. 若子任务 $s_j$ 依赖 $s_k$（即 $s_j.dep = k$），则执行 $s_j$ 前先调用 $E_k.\text{wait}()$
4. 子任务 $s_i$ 完成后调用 $E_i.\text{set}()$，释放所有等待者

这种机制允许无依赖关系的子任务完全并行执行，同时保证有依赖关系的子任务按正确顺序执行。

---

## 6 系统状态机与模式切换

### 6.1 状态定义

系统运行时状态由有限状态机（FSM）管理。定义状态集合 $\mathcal{Q} = \{q_0, q_1, \ldots, q_8\}$：

**表 3** 系统状态定义

| 状态 | 符号 | 语义 |
|------|------|------|
| UNINITIALIZED | $q_0$ | 系统未初始化 |
| IDLE | $q_1$ | 空闲，等待指令 |
| EXPLORING | $q_2$ | 自主探索中（explore-lite 运行） |
| TASK_RECEIVED | $q_3$ | 已接收自然语言指令 |
| PLANNING | $q_4$ | LLM 正在规划任务 |
| EXECUTING | $q_5$ | 导航任务执行中 |
| COMPLETED | $q_6$ | 任务执行完毕 |
| ERROR | $q_7$ | 发生错误 |
| SHUTTING_DOWN | $q_8$ | 系统关闭中 |

### 6.2 状态转换函数

状态转换关系 $\delta \subseteq \mathcal{Q} \times \mathcal{Q}$ 定义如下：

$$\delta = \{(q_0, q_1), (q_1, q_2), (q_1, q_3), (q_2, q_3), (q_2, q_1), (q_3, q_4),$$
$$(q_4, q_5), (q_4, q_7), (q_5, q_6), (q_5, q_7), (q_6, q_1), (q_6, q_2), (q_7, q_1)\}$$
$$\cup \{(q, q_8) \mid q \in \mathcal{Q}\}$$

状态转换函数为线程安全操作，使用互斥锁保护状态读写：

$$\text{transition}(q_{\text{target}}): \begin{cases} q_{\text{current}} \leftarrow q_{\text{target}}, & \text{if } (q_{\text{current}}, q_{\text{target}}) \in \delta \\ \text{raise StateTransitionError}, & \text{otherwise} \end{cases}$$

### 6.3 回调机制

状态机支持三类回调：

1. **进入回调** $f_{\text{enter}}(q)$：进入状态 $q$ 时触发
2. **离开回调** $f_{\text{exit}}(q)$：离开状态 $q$ 时触发
3. **转换回调** $f_{\text{trans}}(q_{\text{from}}, q_{\text{to}})$：特定转换时触发

回调的执行在锁外进行，避免死锁。

### 6.4 模式切换关键路径

**探索 → 任务切换**是系统中最关键的状态转换路径。当系统处于 $q_2$（EXPLORING）状态接收到自然语言指令时：

1. 触发 $q_2 \rightarrow q_3$ 转换
2. 离开 $q_2$ 时执行 `ExploreController.stop_exploration()`：
   - 向所有机器人的 `move_base` 发送取消目标指令
   - 终止 explore-lite 进程
   - 等待机器人停止运动（0.5 秒缓冲）
3. 进入 $q_3$ 后自动转换至 $q_4$（PLANNING）
4. LLM 生成任务计划
5. 转换至 $q_5$（EXECUTING），导航器接管 `move_base`

**任务完成后**（$q_6 \rightarrow q_2$）：若配置了 `resume_explore_after_task = true` 且此前处于探索状态，系统自动恢复探索：

1. 触发 $q_6 \rightarrow q_2$ 转换
2. 进入 $q_2$ 时执行 `ExploreController.start_exploration()`

这种设计确保了 explore-lite 与 NavigationAgent 对 `move_base` 的互斥访问。

---

## 7 系统部署与运行

### 7.1 部署场景

系统支持三种部署场景，通过配置 Profile 切换，如表 4 所示。

**表 4** 部署场景配置

| 场景 | Profile | runtime.mode | Navigator | DMROS | 探索 | 机器人数 |
|------|---------|-------------|-----------|-------|------|---------|
| Gazebo 仿真 | turtlebot3_sim | ros_multi | ROSMultiNavigator | 每机独立实例 | explore-lite | 3 |
| 手动模拟 | manual_sim | simulation | SimulatedNavigator | 关闭 | 无 | 1-3 |
| 真机部署 | real_robot | ros_single | ROSSingleNavigator | 单实例 | explore-lite | 1 |

### 7.2 配置系统

系统采用层次化配置合并机制：

$$\text{Config}_{\text{final}} = \text{DeepMerge}(\text{Defaults}, \text{Profile}, \text{Overrides})$$

其中 `DeepMerge` 为递归字典合并操作，右侧参数的值覆盖左侧。Profile 文件只需指定与默认值不同的配置项。

配置项通过点分路径访问：`config.get("robots.names")` → `["tb3_0", "tb3_1", "tb3_2"]`。

### 7.3 任务执行全流程

以多机器人 Gazebo 仿真场景为例，完整的系统运行流程如下：

**阶段 1：系统启动**
1. 加载 `turtlebot3_sim` 配置 Profile
2. 初始化语义地图（空或从文件加载）
3. 构建 LLMClient、ROSMultiNavigator、NavigationAgent
4. 启动 MultiRobotMapManager（后台轮询线程）
5. 状态机转换：$q_0 \rightarrow q_1$（IDLE）
6. 自动进入探索：$q_1 \rightarrow q_2$（EXPLORING）

**阶段 2：自主探索**
- explore-lite 为每台机器人生成前沿探索目标
- SLAM 实时构建各机器人的占用栅格地图
- `multirobot_map_merge` 合并全局几何地图
- DMROS 实时处理各机器人的 RGBD 数据，输出局部语义地图
- MultiRobotMapManager 以 3 秒周期检测变化并执行融合

**阶段 3：接收自然语言指令**
- 用户输入："让三个机器人分别去咖啡机、沙发和电视柜"
- 状态转换：$q_2 \rightarrow q_3 \rightarrow q_4$
- explore-lite 被终止，move_base 目标被取消

**阶段 4：LLM 规划**
- LLM 输出：$mode = multi$，3 个子任务，$strat = greedy$
- 语义地图验证：确认所有目标存在
- 状态转换：$q_4 \rightarrow q_5$

**阶段 5：多机执行**
- 获取 3 台机器人当前位置（TF2 查询）
- 贪心分配：每台机器人分配到最近的目标
- 3 个线程并行发送 move_base 目标
- 等待所有导航完成

**阶段 6：任务完成**
- 收集执行记录，LLM 生成中文总结
- 状态转换：$q_5 \rightarrow q_6 \rightarrow q_2$（自动恢复探索）

---

## 8 异常处理机制

### 8.1 异常层次结构

系统定义了层次化异常体系，所有自定义异常继承自基类 `RobotNavError`：

```
RobotNavError
├── ConfigError              // 配置文件缺失或格式错误
├── SemanticMapError         // 地图加载、解析或匹配失败
├── LLMError                 // LLM API 调用或响应解析失败
├── TaskPlanningError        // 任务规划失败
├── NavigationError          // 导航目标执行失败
├── AllocationError          // 任务分配失败
├── StateTransitionError     // 非法状态转换
├── ExplorationError         // explore-lite 启停失败
└── MapConversionError       // 格式转换失败
```

### 8.2 多层容错策略

**表 5** 各组件异常处理策略

| 组件 | 故障类型 | 处理策略 | 恢复方式 |
|------|---------|---------|---------|
| LLMClient | API 超时/网络错误 | 切换至 fallback 关键词规划器 | 自动降级 |
| LLMClient | 响应格式异常 | 花括号定位 + 二次 JSON 解析 | 自动修复 |
| SemanticMap | 文件不存在 | 创建空地图 | 探索后填充 |
| SemanticMap | reload 失败 | 保留上一版本 | 静默恢复 |
| MapConverter | DMROS 格式异常 | 跳过本次转换周期 | 下次重试 |
| MapMerger | 部分机器人地图缺失 | 仅使用已有机器人数据 | 降级合并 |
| ExploreController | 启动失败 | 转入 ERROR 状态 | 用户重试 |
| ROSNavigator | move_base 目标超时 | 记录失败，继续下一子任务 | 部分成功 |
| ROSNavigator | move_base 服务不可用 | 标记所有任务为 SERVER_TIMEOUT | 状态通知 |
| StateMachine | 非法转换 | 拒绝并记录日志 | 保持当前态 |
| Orchestrator | 未捕获异常 | 顶层 try/except 捕获 | 尝试优雅关闭 |

核心设计原则为：**系统永不崩溃**。所有异常在适当层级被捕获处理，通过 ERROR 状态自动恢复到 IDLE，用户可随时重试。

---

## 9 总结与展望

### 9.1 工作总结

本文提出并实现了一个完整的多机器人自主探索与协同导航系统，其主要技术贡献包括：

1. **双模态架构设计**：通过有限状态机实现探索模式与任务执行模式的无缝切换，解决了 explore-lite 与 NavigationAgent 对 move_base 的资源竞争问题。

2. **多源语义地图融合算法**：基于"同类别 + 空间邻近"的双重准则，采用圆周均值进行方位角融合，支持增量式实时更新。

3. **LLM 集成任务规划**：将 LLM 的自然语言理解能力与机器人的环境感知相结合，支持自动模式判断、子任务分解和依赖关系识别，并提供无 API 依赖的回退规划器保障可用性。

4. **双策略任务分配**：提供贪心与匈牙利两种分配算法，适应不同场景的实时性与最优性需求。

5. **多层容错机制**：从 LLM 降级到导航恢复的完整异常处理链路，确保系统在各类故障条件下的稳定运行。

### 9.2 不足与展望

当前系统仍存在以下可改进之处：

1. **动态环境适应**：当前语义地图为静态物体建模，未考虑物体的移动或消失。后续可引入物体状态跟踪与地图老化机制。

2. **任务分配的动态调整**：当前任务分配在规划阶段一次性完成。对于长时间任务，可引入在线重规划（re-planning）机制，根据执行情况动态调整分配方案。

3. **多模态指令理解**：当前仅支持文本输入。后续可扩展至语音指令、手势指向等多模态交互方式。

4. **大规模场景扩展**：当物体数量或机器人数量显著增加时，语义匹配和任务分配的计算开销将上升。可考虑引入空间索引（如 R-tree）和近似求解算法。

---

## 参考文献格式说明

本文引用的关键技术及其对应模块如下，供后续补充正式参考文献时使用：

- **explore-lite**：基于前沿（frontier）的自主探索算法，用于探索模式下的导航目标生成。
- **gmapping**：基于粒子滤波的 SLAM 算法，用于构建占用栅格地图。
- **move_base**：ROS Navigation Stack 的导航控制器，集成全局规划与局部避障。
- **DWA (Dynamic Window Approach)**：局部路径规划算法，用于 move_base 的实时避障。
- **multirobot_map_merge**：基于特征匹配的多机器人地图合并方法。
- **DMROS (DualMap_ROS)**：基于 YOLO/FastSAM/SAM/CLIP 的实时语义建图系统。
- **DeepSeek**：兼容 OpenAI 接口的大语言模型，用于任务规划与结果总结。
- **匈牙利算法**：求解二部图最小权匹配的经典算法，时间复杂度 $O(n^3)$。
- **Ratcliff/Obershelp 算法**：Python `difflib.SequenceMatcher` 所使用的序列相似度算法。

---

## 附录 A：核心数据结构定义

### A.1 语义地图 JSON 格式

```json
{
  "objects": [
    {
      "id": "coffee_machine_01",
      "name": "咖啡机",
      "aliases": ["咖啡机台", "coffee machine"],
      "category": "电器",
      "room": "茶水间",
      "position": {"x": 2.1, "y": 5.4, "yaw": 1.57},
      "description": "自动咖啡机"
    }
  ]
}
```

### A.2 LLM 任务规划输出格式

```json
{
  "mode": "multi",
  "alloc_strategy": "greedy",
  "subtasks": [
    {
      "action": "navigate",
      "target": "咖啡机",
      "reason": "用户要求前往咖啡机",
      "depends_on": null,
      "assigned_robot": null
    },
    {
      "action": "navigate",
      "target": "沙发",
      "reason": "送达目的地",
      "depends_on": 0,
      "assigned_robot": "tb3_0"
    }
  ],
  "notes": "任务1和任务2存在依赖关系"
}
```

### A.3 合并后全局语义地图格式（DMROS 格式）

```json
{
  "metadata": {
    "total_objects": 5,
    "source_robots": ["tb3_0", "tb3_1", "tb3_2"],
    "merge_distance": 1.0
  },
  "objects": [
    {
      "class_name": "chair",
      "bbox_2d": {
        "center": [3.05, 0.95, 0.0],
        "extent": [0.6, 0.5]
      },
      "observation_count": 2,
      "source_robots": ["tb3_0", "tb3_1"]
    }
  ]
}
```

### A.4 配置 Profile 示例（turtlebot3_sim）

```json
{
  "profile": "turtlebot3_sim",
  "runtime": {
    "mode": "ros_multi",
    "use_ros": true,
    "auto_explore_on_startup": true,
    "resume_explore_after_task": true
  },
  "robots": {
    "names": ["tb3_0", "tb3_1", "tb3_2"],
    "global_frame": "map",
    "base_frame_suffix": "base_footprint",
    "goal_timeout": 300.0,
    "default_positions": {
      "tb3_0": [-7.0, -1.0],
      "tb3_1": [7.0, -1.0],
      "tb3_2": [0.5, 3.0]
    }
  },
  "dmros": {
    "enabled": true,
    "merge_distance": 1.0,
    "poll_interval": 3.0,
    "per_robot_topics": {
      "tb3_0": {"rgb": "/tb3_0/camera/rgb/image_raw", "...": "..."},
      "tb3_1": {"rgb": "/tb3_1/camera/rgb/image_raw", "...": "..."},
      "tb3_2": {"rgb": "/tb3_2/camera/rgb/image_raw", "...": "..."}
    }
  }
}
```

---

## 附录 B：算法符号表

| 符号 | 含义 |
|------|------|
| $\mathcal{O}$ | 语义地图中的物体集合 |
| $o_i$ | 第 $i$ 个语义物体 |
| $\mathbf{p} = (x, y, \theta)$ | 物体/机器人位姿 |
| $K_i$ | 物体 $o_i$ 的关键词集合 |
| $\sigma(q, c)$ | 查询 $q$ 与候选关键词 $c$ 的字符串相似度 |
| $S(q, o_i)$ | 查询 $q$ 与物体 $o_i$ 的最终匹配得分 |
| $\tau$ | 匹配得分阈值（默认 0.48） |
| $\mathcal{R}$ | 可用机器人集合 |
| $\mathbf{C} \in \mathbb{R}^{m \times n}$ | 任务分配代价矩阵 |
| $\phi$ | 任务分配映射 |
| $d_{\text{merge}}$ | 语义地图融合距离阈值（默认 1.0m） |
| $\mathcal{M}_i^t$ | 机器人 $R_i$ 在时刻 $t$ 的局部语义地图 |
| $\mathcal{G}^t$ | 时刻 $t$ 的全局融合语义地图 |
| $T_{\text{poll}}$ | 地图变化轮询周期（默认 3s） |
| $H$ | 最大对话记忆轮数（默认 8） |
| $\mathcal{Q}$ | 状态机状态集合 |
| $\delta$ | 合法状态转换关系 |
| $d_0$ | 停靠位姿基础偏移距离 |
| $E_i$ | 子任务 $s_i$ 的完成事件（线程同步） |
