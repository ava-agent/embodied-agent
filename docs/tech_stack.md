# 具身智能技术栈全景

## 本项目技术栈

```
Python 3.9+ ─── PyBullet (物理仿真) ─── PyTorch (AI) ─── Gymnasium (RL接口)
                    │                        │
                    │                        └── trained_policy.pt (可部署模型)
                    │
                    ├── Bullet 物理引擎 (碰撞/重力/摩擦/力矩)
                    ├── URDF 机器人模型 (Kuka iiwa 7-DOF)
                    └── OpenGL/Metal 3D 渲染 (GUI 模式)
```

## 生产级技术栈全景

### 仿真层

| 平台 | 物理精度 | 渲染质量 | GPU加速 | 适用场景 |
|------|---------|---------|--------|---------|
| **PyBullet** | 中 | 低 | 无 | 入门学习、快速原型 |
| **MuJoCo** (DeepMind) | 高 | 中 | 支持 | RL 研究、控制算法 |
| **Isaac Sim** (NVIDIA) | 极高 | 极高(RTX) | 原生 | 工业级仿真、合成数据、数字孪生 |
| **Gazebo** | 中高 | 中 | 有限 | ROS 2 开发、多机器人协作 |

### AI 模型层

| 模型 | 参数量 | 输入 | 输出 | 定位 |
|------|--------|------|------|------|
| **MLP 策略网络** (本项目) | 19K | 关节角度+位置(13维) | 关节角速度(7维) | 入门学习 |
| **EEGNet / CNN** | 100K-1M | RGB 图像 | 动作 | 视觉感知 |
| **RT-2** (Google) | 55B | 图像+语言 | 动作token | VLA 先驱 |
| **OpenVLA** (Stanford) | 7B | 图像+语言 | 动作 | 开源 VLA |
| **pi0** (Physical Intelligence) | 未公开 | 图像+语言 | 连续动作 | 当前最强 VLA |
| **GR00T N2** (NVIDIA) | 未公开 | 图像+语言 | 动作 | 人形机器人专用 |

### 中间件层

| 组件 | 作用 | 备注 |
|------|------|------|
| **ROS 2** | 节点通信 (Topic/Service/Action) | 机器人软件的 "Android" |
| **ros2_control** | 硬件抽象 + 实时电机控制 (1kHz) | C++ 实现 |
| **MoveIt 2** | 运动规划 (无碰撞轨迹) | 替代手写逆运动学 |
| **Nav2** | 自主导航 (路径规划+避障) | 移动机器人必备 |

### 硬件层

| 组件 | 入门 | 科研级 | 工业级 |
|------|------|--------|-------|
| **机械臂** | myCobot 280 (¥3K) | Franka Panda (¥20万) | KUKA/UR (¥30万+) |
| **主控** | Jetson Orin Nano (¥3K) | Jetson AGX Orin (¥8K) | 工控机 |
| **相机** | USB 摄像头 | RealSense D435 (¥2K) | 工业相机 |
| **电机驱动** | Arduino/STM32 | 内置驱动 | EtherCAT 伺服 |

## 从仿真到真机：代码改造清单

| 仿真代码 (PyBullet) | 真机代码 (ROS 2) | 说明 |
|---------------------|-----------------|------|
| `p.connect(p.DIRECT)` | `rclpy.init()` | 初始化方式不同 |
| `p.getLinkState(robot, 6)` | 订阅 `/joint_states` | 数据来源：仿真器 → 编码器 |
| `p.setJointMotorControl2()` | 发布 `/arm/commands` | 指令目标：仿真器 → CAN总线 |
| `p.stepSimulation()` | 删除 | 真实世界自动运行 |
| `env.reset()` | 人工复位 | 真实世界没有重置按钮 |
| `trained_policy.pt` | **直接复用** | AI 模型完全不变 |

## 关键概念索引

| 概念 | Demo | 说明 |
|------|------|------|
| **URDF** | Demo 1 | XML 格式描述机器人的关节、质量、形状 |
| **逆运动学 (IK)** | Demo 1 | 给定目标位置 → 计算关节角度 |
| **位置控制** | Demo 1 | 设置电机目标角度，PID 自动跟踪 |
| **Gymnasium 环境** | Demo 2 | 标准的 obs/action/reward/done 接口 |
| **策略网络** | Demo 2 | 神经网络：观测 → 动作 |
| **奖励设计** | Demo 2 | 距离越近奖励越高，到达给额外奖励 |
| **策略梯度** | Demo 2 | 用回报加权更新网络参数 |
| **模型部署** | Demo 3 | 加载 .pt 文件，推理模式运行 |
| **Sim-to-Real** | Demo 3 | 仿真训练 → 真机运行，模型不变 |
