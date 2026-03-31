# 具身智能学习指南

> 从零掌握具身智能所需的全部知识体系，每个模块附带推荐资料和对应的 Demo 代码。

<p align="center">
  <img src="../assets/knowledge_tree.png" width="500" alt="Knowledge Tree"/>
</p>

## 知识体系总览

```
具身智能 = 机器人学基础 + 控制理论 + 强化学习 + 计算机视觉 + 系统工程(ROS 2)
              │              │            │              │              │
           理解身体        控制身体      学会技能       看见世界       连接一切
```

| 模块 | 对应 Demo | 核心问题 | 学习时间 |
|------|----------|---------|---------|
| 1. 机器人学基础 | Demo 1, 4 | 机器人怎么动？ | 1-2 周 |
| 2. 控制理论 | Demo 1, 4 | 怎么让机器人精确地动？ | 1-2 周 |
| 3. 强化学习 | Demo 2, 3 | 怎么让机器人自己学会动？ | 3-4 周 |
| 4. 计算机视觉 | (进阶) | 怎么让机器人看见世界？ | 2-3 周 |
| 5. ROS 2 系统 | (进阶) | 怎么把一切连起来？ | 2-3 周 |

---

## 模块 1: 机器人学基础

> **核心问题**：一个有 N 个关节的机器人，怎么从 A 点移动到 B 点？

### 你需要理解的概念

```
关节空间 (Joint Space)              笛卡尔空间 (Cartesian Space)
[θ1, θ2, θ3, ..., θ7]              [x, y, z, roll, pitch, yaw]
 每个关节的角度                       末端在三维空间的位置和姿态

              正运动学 (FK)
  关节角度 ──────────────────→ 末端位置
            给定角度，算位置

              逆运动学 (IK)
  末端位置 ──────────────────→ 关节角度
            给定位置，算角度（Demo 1 用的就是这个）
```

**URDF 模型**：用 XML 描述机器人的"骨架"——每个关节的类型、位置、运动范围、质量、惯性。
```xml
<!-- 这就是机器人的"身份证" -->
<joint name="shoulder" type="revolute">
  <parent link="base"/>
  <child link="upper_arm"/>
  <limit lower="-3.14" upper="3.14" effort="200"/>
</joint>
```

**自由度 (DOF)**：机器人能独立运动的维度。Kuka 7-DOF = 7个旋转关节，G1 = 29个驱动器。

### 本项目中的对应

| 概念 | Demo 代码位置 |
|------|-------------|
| URDF 加载 | `Demo 1: p.loadURDF("kuka_iiwa/model.urdf")` |
| 逆运动学 | `Demo 1: p.calculateInverseKinematics(robot, 6, target)` |
| 关节映射 | `Demo 4: L_HIP_PITCH, L_KNEE, ...` (G1 的 29 个关节) |
| 自由度 | `Demo 4: model.nq=36, model.nu=29` |

### 推荐学习资料

| 资料 | 类型 | 说明 |
|------|------|------|
| [Robotics 1 - De Luca (Sapienza)](https://www.diag.uniroma1.it/~deluca/rob1_en.php) | 视频课 | 最经典的机器人学入门课，涵盖运动学和动力学 |
| [Modern Robotics (Northwestern)](https://modernrobotics.org/) | 教材+视频 | Kevin Lynch 的免费教材，配套 Coursera 课程和 Python 代码 |
| [URDF Tutorial (ROS Wiki)](http://wiki.ros.org/urdf/Tutorials) | 教程 | 学习怎么读懂和编写 URDF 模型 |
| [MuJoCo 文档 - 建模](https://mujoco.readthedocs.io/en/stable/modeling.html) | 文档 | 理解 MuJoCo 中的关节、驱动器、碰撞体定义 |
| [3Blue1Brown - 线性代数](https://www.3blue1brown.com/topics/linear-algebra) | 视频 | 机器人学的数学基础（矩阵变换、旋转） |

---

## 模块 2: 控制理论

> **核心问题**：知道了目标角度，怎么让电机精确、快速、稳定地转到那个角度？

<p align="center">
  <img src="../assets/pid_control.png" width="450" alt="PID Control"/>
</p>

### 你需要理解的概念

**PD/PID 控制**（Demo 4 的核心）：
```python
# 这就是控制理论的核心公式
error = target_angle - current_angle        # 误差
torque = Kp * error - Kd * current_velocity  # PD 控制输出

# Kp (比例增益): 越大响应越快，但太大会震荡
# Kd (微分增益): 阻尼项，防止震荡
# Ki (积分增益): 消除稳态误差（PID才有，PD没有）
```

**位置控制 vs 力矩控制**：
```
位置控制 (Demo 1):  "关节请转到 0.5 rad"  → 电机自己算力矩
力矩控制 (Demo 4):  "关节请施加 50 N·m"   → 你自己算力矩（更灵活但更难）
```

**为什么 Demo 4 的剧烈动作会摔倒？**
- PD 控制只追踪目标角度，不考虑全身平衡
- 重心超出支撑面就会倒
- 需要更高级的控制：全身动力学控制（Whole-Body Control）或 RL

### 本项目中的对应

| 概念 | Demo 代码位置 |
|------|-------------|
| 位置控制 | `Demo 1: p.setJointMotorControl2(..., POSITION_CONTROL)` |
| PD 力矩控制 | `Demo 4: torque = kp * (target - q) - kd * dq` |
| 增益调参 | `Demo 4: kp=200(腿), kp=40(臂)` — 腿需要更大力矩支撑体重 |

### 推荐学习资料

| 资料 | 类型 | 说明 |
|------|------|------|
| [控制工程导论 (Brian Douglas YouTube)](https://www.youtube.com/playlist?list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk) | 视频 | 最好的控制理论入门视频系列，直觉性极强 |
| [PID Without a PhD](https://www.wescottdesign.com/articles/pid/pidWithoutAPhD.pdf) | 文章 | 不用数学公式也能理解 PID 的经典文章 |
| [MuJoCo - Actuation](https://mujoco.readthedocs.io/en/stable/computation/index.html#actuation) | 文档 | 理解 MuJoCo 中力矩控制的物理模型 |
| [Underactuated Robotics (MIT)](https://underactuated.csail.mit.edu/) | 教材+视频 | Russ Tedrake 的课程，深入控制与优化，偏进阶 |

---

## 模块 3: 强化学习

> **核心问题**：不写规则，让机器人通过试错自己学会走路/抓东西/保持平衡。

<p align="center">
  <img src="../assets/rl_loop.png" width="400" alt="RL Loop"/>
</p>

### 你需要理解的概念

**RL 核心循环**（Demo 2 实现了这个）：
```
                    ┌─────────────┐
        action      │             │  observation
Agent ─────────────→│ Environment │─────────────→ Agent
(神经网络)           │ (仿真器)    │  + reward      (更新权重)
                    └─────────────┘
```

**关键术语对照**：

| RL 术语 | 在 Demo 2 中 | 含义 |
|--------|-------------|------|
| State/Observation | `[7个关节角, 3个末端坐标, 3个目标坐标]` | AI 能看到的信息 |
| Action | `7个关节的角速度增量` | AI 输出的控制指令 |
| Reward | `-distance + 到达奖励` | 离目标越近奖励越高 |
| Policy | `PolicyNetwork (3层MLP)` | AI 的决策函数：obs → action |
| Episode | `一次从reset到done的尝试` | 一轮试错 |

**奖励设计是 RL 最重要的部分**（比算法选择更重要）：
```python
# Demo 2 的奖励设计
reward = -distance                    # 基础：离目标越近越好
if distance < 0.05: reward += 10.0    # 到了附近：额外奖励
if distance < 0.02: reward += 50.0    # 非常精确：超大奖励
```

**常用 RL 算法进阶路径**：
```
Demo 2 (策略梯度)  →  PPO (最常用)  →  SAC (连续控制最强)
     入门                主流               进阶
```

### 本项目中的对应

| 概念 | Demo 代码位置 |
|------|-------------|
| 环境定义 | `Demo 2: class RobotArmEnv (reset/step/obs)` |
| 策略网络 | `Demo 2: class PolicyNetwork (3层MLP, 19K参数)` |
| 奖励设计 | `Demo 2: reward = -distance + 到达奖励` |
| 训练循环 | `Demo 2: for episode → collect → update` |
| 模型保存 | `Demo 2: torch.save(policy, "trained_policy.pt")` |
| 模型部署 | `Demo 3: torch.load("trained_policy.pt")` |

### 推荐学习资料

| 资料 | 类型 | 优先级 | 说明 |
|------|------|--------|------|
| [Spinning Up in Deep RL (OpenAI)](https://spinningup.openai.com/) | 教程+代码 | **必读** | 最好的 RL 入门教程，含 PPO/SAC 实现 |
| [RL Course (Hugging Face)](https://huggingface.co/learn/deep-rl-course) | 在线课 | **必读** | 免费互动课程，从零到PPO，有实操环境 |
| [David Silver RL Course](https://www.davidsilver.uk/teaching/) | 视频课 | 推荐 | DeepMind 首席科学家的经典 RL 课 |
| [CleanRL](https://github.com/vwxyzjn/cleanrl) | 代码 | 推荐 | 单文件 RL 算法实现，PPO/SAC 各 ~300 行 |
| [Gymnasium 文档](https://gymnasium.farama.org/) | 文档 | 推荐 | 理解标准 RL 环境接口 |
| [Sutton & Barto 教材](http://incompleteideas.net/book/the-book-2nd.html) | 教材 | 深入 | RL 圣经，免费电子版 |
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | 代码 | 实战 | 宇树官方 RL 训练框架，直接训练 G1 走路 |

---

## 模块 4: 计算机视觉（进阶）

> **核心问题**：让机器人用"眼睛"（相机）看见世界，而不是依赖预先告知的坐标。

### 你需要理解的概念

```
Demo 1-4 的感知:                    进阶视觉感知:
关节角度 + 坐标值 (数值)              RGB 图像 + 深度图 → 神经网络 → 语义理解
简单但需要人工提供目标位置              复杂但可以自主发现和理解目标
```

**视觉策略 vs 状态策略**：

| 维度 | 状态策略 (本项目) | 视觉策略 (进阶) |
|------|-----------------|----------------|
| 输入 | `[关节角度, 末端坐标, 目标坐标]` | `RGB图像 (480x640x3)` |
| 网络 | MLP (全连接) | CNN / ViT (卷积/Transformer) |
| 优势 | 简单、训练快 | 不需要人工标注目标位置 |
| 劣势 | 需要完美的位置信息 | 训练慢、需要大量数据 |

**VLA 模型**（2025年最前沿）：
```
输入: 相机图像 + 语言指令 "把红色杯子放到桌上"
  │
  ▼
Vision-Language-Action 模型 (如 OpenVLA, pi0)
  │
  ▼
输出: 机器人关节动作序列
```

### 推荐学习资料

| 资料 | 类型 | 说明 |
|------|------|------|
| [CS231n (Stanford)](http://cs231n.stanford.edu/) | 视频课 | 经典计算机视觉课（CNN/ViT） |
| [PyTorch 视觉教程](https://pytorch.org/tutorials/beginner/transfer_learning_tutorial.html) | 教程 | 迁移学习快速上手 |
| [OpenCV Python 教程](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) | 教程 | 图像处理基础 |
| [OpenVLA 论文](https://openvla.github.io/) | 论文 | 开源 VLA 模型，理解视觉-语言-动作架构 |
| [RT-2 论文](https://robotics-transformer2.github.io/) | 论文 | Google 的 VLA 先驱工作 |

---

## 模块 5: ROS 2 系统工程（进阶）

> **核心问题**：怎么把感知、决策、控制各个模块连接成一个完整的机器人系统？

### 你需要理解的概念

```
ROS 2 的核心就是4个概念:

Node (节点)     = 一个独立程序（感知节点、控制节点、AI节点...）
Topic (话题)    = 节点之间传数据的管道（发布/订阅模式）
Service (服务)  = 一问一答的通信（类似 HTTP API）
Action (动作)   = 长时间任务（导航到某点，带进度反馈）
```

**为什么 Demo 1-4 没用 ROS 2？**
- Demo 是单进程程序，感知/决策/执行在同一个 Python 文件里
- 真实机器人上，这些是不同的进程/不同的电脑，需要 ROS 2 通信

### 推荐学习资料

| 资料 | 类型 | 说明 |
|------|------|------|
| [ROS 2 官方教程](https://docs.ros.org/en/jazzy/Tutorials.html) | 教程 | **必读**，从零学 ROS 2（建议 Docker 运行 Ubuntu） |
| [ROS 2 for Beginners (Udemy)](https://www.udemy.com/course/ros2-for-beginners/) | 视频课 | Edouard Renard 的付费课，极好的入门课 |
| [The Construct](https://www.theconstructsim.com/) | 在线平台 | 在线 ROS 2 学习环境，不需要本地安装 |
| [Navigation2 文档](https://docs.nav2.org/) | 文档 | 机器人自主导航框架 |
| [MoveIt 2 教程](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html) | 教程 | 机械臂运动规划框架 |

---

## 数学基础（按需补充）

> 不需要从头学完所有数学，遇到不懂的再查。

| 数学领域 | 用在哪里 | 最低要求 | 推荐资料 |
|---------|---------|---------|---------|
| **线性代数** | 运动学变换、神经网络 | 矩阵乘法、逆矩阵、特征值 | [3Blue1Brown 线性代数](https://www.3blue1brown.com/topics/linear-algebra) |
| **微积分** | 反向传播、优化 | 偏导数、链式法则、梯度 | [3Blue1Brown 微积分](https://www.3blue1brown.com/topics/calculus) |
| **概率论** | RL 策略、贝叶斯滤波 | 概率分布、期望、条件概率 | [StatQuest YouTube](https://www.youtube.com/c/joshstarmer) |
| **优化理论** | 训练神经网络 | 梯度下降、SGD、Adam | [Optimizer 可视化](https://github.com/Jaewan-Yun/optimizer-visualization) |

---

## 推荐学习顺序

```
第 1 周: 机器人学基础 + 控制理论
  ├── 看 Modern Robotics 前 4 章（正/逆运动学）
  ├── 看 Brian Douglas 的 PID 视频 (3-4集)
  ├── 修改 Demo 1 的目标位置，观察逆运动学结果
  └── 修改 Demo 4 的 Kp/Kd 参数，观察 G1 的抖动/稳定

第 2-3 周: 强化学习
  ├── 读完 Spinning Up 的 Part 1 (关键概念)
  ├── 跑通 Hugging Face RL Course 的前 4 单元
  ├── 修改 Demo 2 的奖励函数，观察训练效果变化
  └── 尝试用 CleanRL 的 PPO 替换 Demo 2 的策略梯度

第 4 周: 宇树 RL 实战
  ├── 跑通 unitree_rl_gym 的 Go2/G1 训练
  ├── 理解 Domain Randomization 代码
  └── 尝试修改奖励函数，训练不同步态

第 5-6 周: 视觉 + ROS 2（可选）
  ├── CS231n 前 5 讲（CNN 基础）
  ├── ROS 2 官方教程 Beginner 部分
  └── Docker 运行 Gazebo + Nav2
```

---

## 一句话总结每个模块

| 模块 | 一句话 |
|------|-------|
| 机器人学 | "给定目标位置，算出每个关节该转多少度" |
| 控制理论 | "知道了目标角度，用 PD 控制让电机精确转过去" |
| 强化学习 | "不告诉机器人怎么做，让它自己试错学会" |
| 计算机视觉 | "让机器人用相机看见世界，而不是靠人告知坐标" |
| ROS 2 | "把感知、决策、控制各个模块用管道连起来" |
