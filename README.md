# embodied-agent

具身智能学习与仿真 Demo — 从零开始学习机器人 AI

> 没有真实机器人也能学具身智能。本项目通过物理仿真器，在你的电脑上运行完整的「感知 → 决策 → 执行」机器人控制流程。

## 架构

```
你的电脑（Mac/Linux/Windows）
┌─────────────────────────────────────────────────┐
│                                                 │
│  PyBullet 物理仿真器                              │
│  ├── 虚拟世界（重力、碰撞、摩擦）                  │
│  ├── 机械臂模型（Kuka iiwa, 7自由度）             │
│  └── 目标物体（红色方块）                          │
│                                                 │
│  PyTorch AI 模型                                 │
│  ├── 策略网络（观测 → 动作）                       │
│  └── 强化学习训练循环                              │
│                                                 │
│  控制流程（和真实机器人完全一致）                    │
│  ├── 感知：获取关节角度 + 末端位置                  │
│  ├── 决策：AI模型 / 逆运动学计算                   │
│  └── 执行：设置电机目标角度                        │
│                                                 │
└─────────────────────────────────────────────────┘
```

## 快速开始

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 运行机械臂抓取演示（弹出3D窗口）
python demos/01_robot_arm_grasp.py

# 3. 训练AI控制机械臂（强化学习）
python demos/02_rl_training.py

# 4. 部署训练好的模型
python demos/03_deploy_model.py
```

## Demo 说明

| Demo | 文件 | 内容 | 核心概念 |
|------|------|------|---------|
| 01 | `demos/01_robot_arm_grasp.py` | 机械臂3D可视化抓取 | URDF模型、逆运动学、位置控制 |
| 02 | `demos/02_rl_training.py` | 强化学习训练AI | 仿真环境、策略网络、奖励设计 |
| 03 | `demos/03_deploy_model.py` | 部署AI模型到机器人 | 模型加载、推理、仿真 vs 真机对比 |

## 技术栈

| 组件 | 技术 | 作用 |
|------|------|------|
| 物理仿真 | PyBullet | 模拟真实物理世界（重力、碰撞、摩擦） |
| AI 框架 | PyTorch | 神经网络定义、训练、推理 |
| RL 接口 | Gymnasium | 标准化的观测/动作/奖励接口 |
| 机器人模型 | URDF | 描述机器人关节、质量、形状的XML格式 |

## 核心概念

### 从仿真到真机：代码对比

```python
# === 仿真环境（本项目）===
obs = env.step(action)          # 从 PyBullet 获取数据
action = policy(obs)            # AI 推理（完全一样）
env.step(action)                # 发给 PyBullet 虚拟电机

# === 真实机器人 ===
obs = ros_node.get_obs()        # 从 ROS 2 传感器获取数据
action = policy(obs)            # AI 推理（完全一样）
ros_node.send_cmd(action)       # 通过 ROS 2 发给真实电机
```

AI 模型（`policy`）不需要任何修改，变的只是数据来源和执行目标。

### 具身智能控制循环

```
感知 ──→ 决策 ──→ 执行
 │        │        │
 │        │        └── 设置关节角度（仿真电机 / 真实电机）
 │        └─────────── AI模型推理 / 逆运动学
 └──────────────────── 读取传感器（仿真API / ROS Topic）
```

## 进阶路线

```
当前（本项目）                     下一步
├── PyBullet 入门仿真        →   MuJoCo / Isaac Sim 专业仿真
├── 3层MLP策略网络           →   VLA大模型（pi0 / OpenVLA）
├── 简单策略梯度             →   PPO / SAC 算法
├── 关节角度输入             →   RGB相机图像输入
├── 直接PyBullet API         →   ROS 2中间件
└── 纯仿真                   →   Sim-to-Real真机部署
```

## 上真机需要什么

| 条件 | 最低方案 | 推荐方案 |
|------|---------|---------|
| 机械臂 | 舵机臂套件 (~800元) | myCobot 280 (~3000元) |
| 主控电脑 | 你的Mac (USB连接) | Jetson Orin Nano (~3000元) |
| 传感器 | 关节编码器（臂自带） | + Intel RealSense 相机 |
| 中间件 | pymycobot SDK | ROS 2 + MoveIt 2 |
| 系统 | macOS / Windows | Ubuntu 22.04 |

## 相关资源

- [PyBullet 官方文档](https://pybullet.org/)
- [ROS 2 Jazzy 文档](https://docs.ros.org/en/jazzy/)
- [MoveIt 2](https://moveit.picknik.ai/)
- [MuJoCo](https://mujoco.readthedocs.io/)
- [Open X-Embodiment](https://robotics-transformer-x.github.io/)
- [Gymnasium Robotics](https://robotics.farama.org/)
