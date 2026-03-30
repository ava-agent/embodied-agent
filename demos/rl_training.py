"""
示例2: 用强化学习训练一个AI控制机器人
====================================

这个程序展示了具身智能的核心训练流程：
1. 创建一个仿真环境（机械臂 + 目标）
2. AI（神经网络）通过反复试错学习控制策略
3. 训练完成后保存模型权重文件
4. 这个权重文件可以直接部署到真实机器人上运行

整个训练过程不需要真实机器人——全部在你的电脑上完成。
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import pybullet as p
import pybullet_data
import time
from collections import deque


# =============================================================
# 第一部分：仿真环境（模拟真实世界）
# =============================================================

class RobotArmEnv:
    """机械臂抓取环境

    这就是一个"虚拟世界"：
    - 有一个7关节机械臂
    - 有一个随机放置的目标物体
    - AI的任务是控制机械臂到达目标
    - 到达目标获得奖励，远离则受到惩罚

    这和 Gymnasium 的标准环境接口一致。
    真实机器人也会包装成同样的接口。
    """

    def __init__(self, render=False):
        # 选择是否显示3D窗口
        if render:
            self.client = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.3])
        else:
            self.client = p.connect(p.DIRECT)  # 无头模式，训练时用这个（更快）

        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.robot_id = None
        self.target_pos = None
        self.step_count = 0
        self.max_steps = 200

        # 观测空间维度：7个关节角度 + 3个末端位置 + 3个目标位置 = 13
        self.obs_dim = 13
        # 动作空间维度：7个关节的角速度
        self.action_dim = 7

    def reset(self):
        """重置环境——开始新的一轮尝试"""
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        # 加载地面和机械臂
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(
            "kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

        # 随机放置目标（每次位置不同，迫使AI学到泛化的策略）
        self.target_pos = np.array([
            0.4 + np.random.uniform(-0.15, 0.15),
            np.random.uniform(-0.2, 0.2),
            0.1 + np.random.uniform(0, 0.3)
        ])

        # 创建目标球体（绿色）
        visual = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 1, 0, 0.7])
        p.createMultiBody(0, -1, visual, self.target_pos)

        self.step_count = 0
        return self._get_observation()

    def step(self, action):
        """执行一步动作

        这是仿真环境的核心：
        1. 接收AI输出的动作（关节角速度）
        2. 在物理引擎中执行这个动作
        3. 返回新的观测、奖励、是否结束

        同样的接口，也可以接收真实机器人的传感器数据。
        """
        # 将AI的动作缩放到合理范围
        action = np.clip(action, -1, 1) * 0.1

        # 获取当前关节角度
        joint_positions = []
        for i in range(7):
            state = p.getJointState(self.robot_id, i)
            joint_positions.append(state[0])

        # 设置新的关节目标 = 当前角度 + 动作增量
        for i in range(7):
            target = joint_positions[i] + action[i]
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL,
                targetPosition=target, force=200)

        # 物理引擎前进多步（模拟真实物理）
        for _ in range(10):
            p.stepSimulation()

        self.step_count += 1

        # 获取新状态
        obs = self._get_observation()

        # 计算奖励
        ee_pos = self._get_ee_pos()
        distance = np.linalg.norm(ee_pos - self.target_pos)

        # 奖励设计：离目标越近奖励越高
        reward = -distance  # 基础奖励：负的距离
        if distance < 0.05:
            reward += 10.0   # 到达目标：大奖励！
        if distance < 0.02:
            reward += 50.0   # 非常精确：超大奖励！

        # 判断是否结束
        done = (distance < 0.02) or (self.step_count >= self.max_steps)

        return obs, reward, done, {"distance": distance}

    def _get_observation(self):
        """获取当前观测（AI的"眼睛看到的信息"）"""
        joint_angles = []
        for i in range(7):
            state = p.getJointState(self.robot_id, i)
            joint_angles.append(state[0])

        ee_pos = self._get_ee_pos()

        # 观测 = [7个关节角度, 末端xyz, 目标xyz]
        obs = np.concatenate([joint_angles, ee_pos, self.target_pos])
        return obs.astype(np.float32)

    def _get_ee_pos(self):
        """获取末端执行器位置"""
        state = p.getLinkState(self.robot_id, 6)
        return np.array(state[0])

    def close(self):
        p.disconnect()


# =============================================================
# 第二部分：AI模型（神经网络）
# =============================================================

class PolicyNetwork(nn.Module):
    """策略网络——AI的"大脑"

    输入：当前观测（关节角度 + 末端位置 + 目标位置）
    输出：每个关节应该怎么动（角速度）

    就是一个普通的全连接神经网络。
    训练完成后保存为 .pt 文件，可以部署到任何设备上。
    """

    def __init__(self, obs_dim=13, action_dim=7, hidden_dim=128):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()  # 输出范围 [-1, 1]
        )

    def forward(self, obs):
        return self.network(obs)


# =============================================================
# 第三部分：训练（让AI通过试错学习）
# =============================================================

def train():
    print("=" * 60)
    print("  强化学习训练：教AI控制机械臂")
    print("  (全部在仿真中完成，不需要真实机器人)")
    print("=" * 60)
    print()

    # 创建仿真环境（无头模式，速度更快）
    env = RobotArmEnv(render=False)

    # 创建AI模型
    policy = PolicyNetwork()
    optimizer = optim.Adam(policy.parameters(), lr=1e-3)

    # 简单的进化策略（ES）训练
    # 这比标准RL更简单，适合演示
    best_reward = -float('inf')
    reward_history = deque(maxlen=50)

    num_episodes = 200
    print(f"开始训练 {num_episodes} 轮...\n")

    for episode in range(num_episodes):
        # 重置环境（目标位置随机变化）
        obs = env.reset()
        total_reward = 0
        episode_distances = []

        # 一轮尝试
        for step in range(200):
            # AI根据观测决定动作
            obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
            with torch.no_grad():
                action = policy(obs_tensor).squeeze(0).numpy()

            # 训练时添加探索噪声（鼓励AI尝试新动作）
            noise_scale = max(0.3 * (1 - episode / num_episodes), 0.05)
            action = action + np.random.normal(0, noise_scale, size=action.shape)

            # 执行动作，获取反馈
            obs, reward, done, info = env.step(action)
            total_reward += reward
            episode_distances.append(info["distance"])

            if done:
                break

        reward_history.append(total_reward)
        avg_reward = np.mean(reward_history)
        min_dist = min(episode_distances)

        # 如果这轮表现更好，用这轮的经验更新模型
        if total_reward > best_reward:
            best_reward = total_reward
            # 保存最佳模型
            torch.save(policy.state_dict(), "best_policy.pt")

        # 简单的策略梯度更新
        obs = env.reset()
        log_probs = []
        rewards = []

        for step in range(200):
            obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
            action = policy(obs_tensor).squeeze(0)

            # 添加噪声并记录概率
            noise = torch.randn_like(action) * noise_scale
            noisy_action = action + noise
            log_prob = -0.5 * (noise ** 2).sum()
            log_probs.append(log_prob)

            obs, reward, done, info = env.step(noisy_action.detach().numpy())
            rewards.append(reward)

            if done:
                break

        # 策略梯度更新
        returns = []
        R = 0
        for r in reversed(rewards):
            R = r + 0.99 * R
            returns.insert(0, R)
        returns = torch.FloatTensor(returns)
        if len(returns) > 1:
            returns = (returns - returns.mean()) / (returns.std() + 1e-8)

        loss = 0
        for log_prob, R in zip(log_probs, returns):
            loss -= log_prob * R

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # 打印训练进度
        if (episode + 1) % 10 == 0:
            print(f"  轮次 {episode + 1:3d}/{num_episodes} | "
                  f"奖励: {total_reward:8.1f} | "
                  f"平均: {avg_reward:8.1f} | "
                  f"最近距离: {min_dist:.4f}m | "
                  f"最佳: {best_reward:8.1f}")

    env.close()

    # 保存最终模型
    model_path = "trained_policy.pt"
    torch.save(policy.state_dict(), model_path)

    print()
    print("=" * 60)
    print(f"  训练完成！模型已保存到: {model_path}")
    print()
    print("  这个 .pt 文件就是AI学到的'技能'。")
    print("  它可以被加载到：")
    print("    - 另一个仿真环境中运行")
    print("    - 真实机器人的 Jetson 电脑上运行")
    print("    - 任何有 PyTorch 的设备上运行")
    print()
    print("  运行 demo_deploy_model.py 来看训练效果")
    print("=" * 60)


if __name__ == "__main__":
    train()
