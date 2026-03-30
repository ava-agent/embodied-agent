# embodied-agent

> 具身智能学习与仿真 Demo — 没有真实机器人也能学机器人 AI

<p align="center">
  <img src="assets/sim_to_real.png" width="600" alt="Sim-to-Real"/>
</p>

本项目通过 **PyBullet 物理仿真器**，在你的电脑上运行完整的机器人「感知 → 决策 → 执行」控制流程。训练出的 AI 模型（`.pt` 文件）可直接部署到真实机器人上。

## 运行效果

Kuka iiwa 7自由度机械臂在仿真环境中抓取红色方块：

| 初始状态 | 接近目标 | 到达抓取位置 |
|:---:|:---:|:---:|
| ![initial](assets/screenshot_initial.png) | ![reaching](assets/screenshot_reaching.png) | ![grasp](assets/screenshot_grasp.png) |
| 机械臂直立，方块在右前方 | 机械臂弯曲伸向目标 | 末端到达方块上方，误差 < 2cm |

## 快速开始

```bash
git clone https://github.com/ava-agent/embodied-agent.git
cd embodied-agent

# 安装依赖
pip install -r requirements.txt

# Demo 1: 机械臂抓取（弹出3D窗口）
python demos/01_robot_arm_grasp.py

# Demo 2: 强化学习训练AI
python demos/02_rl_training.py

# Demo 3: 部署训练好的模型
python demos/03_deploy_model.py
```

## Demo 说明

### Demo 1: 机械臂抓取 — 传统控制

用**逆运动学**（给定目标位置 → 计算关节角度）控制机械臂分三步抓取方块。

```
阶段1: 移动到目标上方  →  误差 0.055m
阶段2: 下降接近目标    →  误差 0.018m
阶段3: 到达抓取位置    →  误差 0.019m
```

核心概念：URDF 模型加载、逆运动学、位置控制

### Demo 2: 强化学习训练 — AI 控制

让**神经网络**通过反复试错学会控制机械臂。训练完成后输出 `trained_policy.pt` 模型文件。

```python
# AI 控制的核心就是这3行
obs = env.get_observation()               # 感知
action = policy_network(obs)              # 决策（神经网络推理）
env.step(action)                          # 执行
```

核心概念：Gymnasium 环境、策略网络、奖励设计、策略梯度

### Demo 3: 模型部署 — Sim vs Real

加载训练好的 `.pt` 模型，在仿真中运行，并展示仿真代码与真机代码的对比。

```python
# 仿真（本项目）                        # 真机（买了机器人后）
obs = env.step(action)                   obs = ros_node.get_obs()
action = policy(obs)  # ← 完全一样 →    action = policy(obs)
env.step(action)                         ros_node.send_cmd(action)
```

## 架构

<p align="center">
  <img src="assets/architecture.png" width="500" alt="Architecture"/>
</p>

```
┌──────────────────────────────────────────────┐
│  感知 Perception                              │
│  获取关节角度 + 末端位置 + 目标位置              │
│  (仿真: PyBullet API / 真机: ROS 2 Topic)     │
├──────────────────────────────────────────────┤
│  决策 Decision                                │
│  神经网络推理: 观测 → 动作                      │
│  (PyTorch, trained_policy.pt)                 │
├──────────────────────────────────────────────┤
│  执行 Action                                  │
│  设置关节目标角度                               │
│  (仿真: PyBullet 电机 / 真机: CAN总线→电机)     │
└──────────────────────────────────────────────┘
```

## 技术栈

| 组件 | 技术 | 作用 |
|------|------|------|
| 物理仿真 | **PyBullet** (Bullet 引擎) | 重力、碰撞、摩擦、力矩模拟 |
| AI 框架 | **PyTorch** | 策略网络定义、训练、推理 |
| RL 接口 | **Gymnasium** | 标准 obs/action/reward 接口 |
| 机器人模型 | **URDF** (Kuka iiwa) | 描述关节、质量、形状的 XML |
| 语言 | **Python 3.9+** | 全栈开发 |

### 与生产级技术栈对比

| 维度 | 本项目（入门） | 生产级 |
|------|-------------|-------|
| 仿真器 | PyBullet | Isaac Sim (NVIDIA) / MuJoCo |
| AI 模型 | 3层MLP, 19K参数 | VLA 大模型 (pi0/OpenVLA), 数十亿参数 |
| 感知输入 | 关节角度+位置 | RGB相机图像+深度图+点云 |
| 中间件 | 直接 PyBullet API | ROS 2 + ros2_control + MoveIt 2 |
| 部署硬件 | 你的电脑 | NVIDIA Jetson Orin |

## 学习路线

<p align="center">
  <img src="assets/roadmap.png" width="600" alt="Learning Roadmap"/>
</p>

| 阶段 | 内容 | 时间 |
|------|------|------|
| **1. 本项目** | 运行 3 个 Demo，理解感知→决策→执行循环 | 1-2 周 |
| **2. 进阶仿真** | MuJoCo + PPO/SAC 算法 + 图像输入 | 2-4 周 |
| **3. ROS 2** | Docker 运行 ROS 2，Gazebo 仿真 | 2-4 周 |
| **4. 真机部署** | 入门级机械臂 + Sim-to-Real 迁移 | 持续 |

## 上真机需要什么

| 条件 | 最低方案 | 推荐方案 |
|------|---------|---------|
| 机械臂 | 舵机臂套件 (~800元) | myCobot 280 (~3,000元) |
| 主控 | 你的电脑 (USB) | Jetson Orin Nano (~3,000元) |
| 传感器 | 关节编码器（臂自带） | + Intel RealSense 相机 |
| 中间件 | pymycobot SDK | ROS 2 + MoveIt 2 |

## 项目结构

```
embodied-agent/
├── README.md
├── requirements.txt
├── assets/                          # 图片资源
│   ├── architecture.png             # 架构图
│   ├── sim_to_real.png              # Sim-to-Real 对比图
│   ├── roadmap.png                  # 学习路线图
│   ├── screenshot_initial.png       # 仿真截图：初始状态
│   ├── screenshot_reaching.png      # 仿真截图：接近目标
│   └── screenshot_grasp.png         # 仿真截图：抓取位置
├── demos/
│   ├── 01_robot_arm_grasp.py        # Demo 1: 逆运动学抓取
│   ├── 02_rl_training.py            # Demo 2: 强化学习训练
│   ├── 03_deploy_model.py           # Demo 3: 模型部署
│   └── rl_training.py               # 共享模块（环境+模型）
└── docs/
    └── tech_stack.md                # 技术栈详细文档
```

## 相关资源

- [PyBullet Quickstart](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) | [MoveIt 2](https://moveit.picknik.ai/) | [Nav2](https://docs.nav2.org/)
- [MuJoCo](https://mujoco.readthedocs.io/) | [Isaac Sim](https://developer.nvidia.com/isaac-sim)
- [Open X-Embodiment](https://robotics-transformer-x.github.io/) | [Gymnasium Robotics](https://robotics.farama.org/)
