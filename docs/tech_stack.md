# 具身智能技术栈全景

## 本项目使用的技术栈

```
┌───────────────────────────────────────────┐
│  语言: Python 3.9+                         │
│  物理仿真: PyBullet (Bullet 物理引擎)       │
│  AI 框架: PyTorch                          │
│  RL 接口: Gymnasium                        │
│  机器人模型: URDF (Kuka iiwa 7-DOF)        │
│  产出: trained_policy.pt (可部署的AI模型)   │
└───────────────────────────────────────────┘
```

## 生产级具身智能技术栈

### 仿真层

| 平台 | 物理精度 | 渲染质量 | 适用场景 |
|------|---------|---------|---------|
| **PyBullet** | 中 | 低 | 入门学习、快速原型 |
| **MuJoCo** (DeepMind) | 高 | 中 | RL研究、控制算法 |
| **Isaac Sim** (NVIDIA) | 极高 | 极高(RTX) | 工业级仿真、合成数据 |
| **Gazebo** | 中高 | 中 | ROS 2 开发、多机器人 |

### AI 模型层

| 模型 | 参数量 | 输入 | 特点 |
|------|--------|------|------|
| MLP 策略网络 (本项目) | 19K | 关节角度+位置 | 入门级 |
| EEGNet / CNN | 100K-1M | 图像 | 视觉感知 |
| RT-2 (Google) | 55B | 图像+语言 | VLA先驱 |
| OpenVLA (Stanford) | 7B | 图像+语言 | 开源VLA |
| pi0 (Physical Intelligence) | - | 图像+语言 | 当前最强 |
| GR00T N2 (NVIDIA) | - | 图像+语言 | 人形机器人专用 |

### 中间件层

| 组件 | 作用 |
|------|------|
| **ROS 2** | 节点通信框架（Topic/Service/Action） |
| **ros2_control** | 硬件抽象 + 实时电机控制 |
| **MoveIt 2** | 运动规划（无碰撞轨迹） |
| **Nav2** | 自主导航（路径规划 + 避障） |

### 硬件层

| 组件 | 入门方案 | 专业方案 |
|------|---------|---------|
| 机械臂 | myCobot 280 (¥3K) | Franka Panda (¥20万) |
| 主控 | Jetson Orin Nano (¥3K) | Jetson AGX Orin (¥8K) |
| 相机 | USB摄像头 | Intel RealSense D435 |
| 电机驱动 | Arduino/STM32 | EtherCAT伺服驱动 |

## 从本项目到真机的改造路径

```
本项目 (仿真)                         真机部署
──────────────                        ──────────
PyBullet API                    →     ROS 2 Topic
p.getLinkState()                →     /joint_states 订阅
p.setJointMotorControl2()       →     /arm/commands 发布
p.stepSimulation()              →     删除（真实世界自动运行）
env.reset()                     →     人工复位
trained_policy.pt               →     直接复用（不修改）
```

## 学习路线

```
第1阶段: 本项目 (1-2周)
├── 运行3个Demo，理解感知→决策→执行循环
├── 修改奖励函数，观察训练效果变化
└── 尝试不同的网络结构

第2阶段: 进阶仿真 (2-4周)
├── 安装MuJoCo，用Gymnasium Robotics环境
├── 实现PPO/SAC算法
└── 加入相机图像作为观测输入

第3阶段: ROS 2 (2-4周)
├── Docker运行ROS 2
├── 学习Node/Topic/Launch
└── 在Gazebo中仿真完整机器人

第4阶段: 真机 (持续)
├── 购入入门级机械臂
├── ROS 2驱动对接
└── Sim-to-Real迁移
```
