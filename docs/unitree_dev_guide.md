# 宇树 (Unitree) 机器人开发指南

## 环境搭建

### 前置条件

```bash
# 安装 Miniforge (Apple Silicon Mac)
curl -fsSL -o /tmp/miniforge.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh
bash /tmp/miniforge.sh -b -p ~/miniforge3

# 创建开发环境
conda create -n unitree python=3.11 -y
conda activate unitree

# 安装核心依赖
pip install mujoco numpy torch

# 编译安装 CycloneDDS（Mac ARM 需要源码编译）
conda install cmake -y
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git --branch 0.10.5 --depth 1 /tmp/cyclonedds
cd /tmp/cyclonedds && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DBUILD_EXAMPLES=OFF
make -j$(sysctl -n hw.ncpu) && make install
```

### 安装宇树 SDK

```bash
# Python SDK（核心）
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
CYCLONEDDS_HOME=$CONDA_PREFIX pip install -e .

# MuJoCo 仿真器
git clone https://github.com/unitreerobotics/unitree_mujoco.git
pip install mujoco pygame

# 强化学习训练（可选）
git clone https://github.com/unitreerobotics/unitree_rl_gym.git
```

## 可用机器人模型

| 型号 | DOF | 驱动器数 | 仿真模型路径 |
|------|-----|---------|-------------|
| Go2 | 19 | 12 | `unitree_robots/go2/scene.xml` |
| G1 | 36 | 29 | `unitree_robots/g1/scene.xml` |
| H1 | 27 | 20 | `unitree_robots/h1/scene.xml` |
| H1_2 | - | - | `unitree_robots/h1_2/scene.xml` |
| B2 | - | - | `unitree_robots/b2/scene.xml` |

## 控制模式

### 高层控制（推荐入门）

发送速度指令，机器人自主规划步态：

```python
from unitree_sdk2py.core.channel import ChannelFactory, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeCmd_

# 仿真用 "lo"（本地回环），实机用网卡名如 "eth0"
ChannelFactory.Instance().Init(0, "lo")

pub = ChannelPublisher("rt/sportmode/cmd", SportModeCmd_)
pub.Init()

cmd = SportModeCmd_()
cmd.velocity = [0.5, 0.0, 0.0]  # 前进 0.5 m/s
pub.Write(cmd)
```

### 低层控制（进阶）

直接控制每个关节的位置/力矩：

```python
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_

cmd = LowCmd_()
# 关节 0 的 PD 控制
cmd.motor_cmd[0].q = 0.5       # 目标角度 (rad)
cmd.motor_cmd[0].kp = 50.0     # 比例增益
cmd.motor_cmd[0].kd = 3.0      # 微分增益
cmd.motor_cmd[0].tau = 0.0     # 前馈力矩
pub.Write(cmd)
```

> 使用低层控制前必须通过手机 App 关闭 `sport_mode`，防止指令冲突。

## 仿真 → 实机 迁移

```
仿真代码                              实机代码
──────                               ──────
ChannelFactory.Init(0, "lo")    →    ChannelFactory.Init(0, "eth0")
                                     其他代码完全不变
```

这就是宇树仿真最大的优势：**API 完全一致，改一个参数上真机**。

## 开发资源

| 资源 | 链接 |
|------|------|
| 官方 SDK | [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
| 官方仿真 | [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) |
| RL 训练 | [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) |
| 模仿学习 | [unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot) |
| Isaac Lab | [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) |
| ROS 2 | [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) |
| 社区索引 | [awesome-unitree-robots](https://github.com/shaoxiang/awesome-unitree-robots) |
| 官方文档 | [support.unitree.com](https://support.unitree.com/home/zh/developer) |

## 购买参考

| 目标 | 推荐型号 | 价格 |
|------|---------|------|
| 纯仿真学习 | 不需要买 | 免费 |
| 四足开发 | Go2 EDU | ~¥85,000 |
| 人形入门 | G1 基础版 | ~¥72,000 |
| 人形研究 | G1 EDU | ¥130,000-540,000 |
