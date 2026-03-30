"""
宇树 G1 人形机器人动作演示
=========================

让 G1 依次完成：站立 → 挥手打招呼 → 鞠躬 → 出拳 → 抬腿 → 太极起势

无需 Linux，直接在 macOS 上用 MuJoCo 运行。

用法:
    conda activate unitree
    cd ~/projects/ios/unitree_mujoco
    python demo_g1_actions.py
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# ====================================================================
# G1 关节索引映射（29个驱动器）
# ====================================================================
# 腿部 (0-11)
L_HIP_PITCH, L_HIP_ROLL, L_HIP_YAW = 0, 1, 2
L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL = 3, 4, 5
R_HIP_PITCH, R_HIP_ROLL, R_HIP_YAW = 6, 7, 8
R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL = 9, 10, 11

# 腰部 (12-14)
WAIST_YAW, WAIST_ROLL, WAIST_PITCH = 12, 13, 14

# 左臂 (15-21)
L_SHOULDER_PITCH, L_SHOULDER_ROLL, L_SHOULDER_YAW = 15, 16, 17
L_ELBOW = 18
L_WRIST_ROLL, L_WRIST_PITCH, L_WRIST_YAW = 19, 20, 21

# 右臂 (22-28)
R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_SHOULDER_YAW = 22, 23, 24
R_ELBOW = 25
R_WRIST_ROLL, R_WRIST_PITCH, R_WRIST_YAW = 26, 27, 28

NUM_ACTUATORS = 29


# ====================================================================
# PD 控制器：将目标关节角度转换为力矩指令
# ====================================================================
class PDController:
    """PD 位置控制器

    真实机器人的 ros2_control 做的也是同样的事：
    tau = Kp * (q_target - q_current) - Kd * dq_current
    """

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.target = np.zeros(NUM_ACTUATORS)

        # PD 增益（腿部需要更大的力矩来支撑体重）
        self.kp = np.zeros(NUM_ACTUATORS)
        self.kd = np.zeros(NUM_ACTUATORS)

        # 腿部关节：高刚度
        for i in range(12):
            self.kp[i] = 200.0
            self.kd[i] = 10.0
        # 腰部
        for i in range(12, 15):
            self.kp[i] = 200.0
            self.kd[i] = 10.0
        # 手臂
        for i in range(15, 29):
            self.kp[i] = 40.0
            self.kd[i] = 2.0

    def set_target(self, target_dict):
        """设置目标关节角度（只更新指定的关节）"""
        for joint_idx, angle in target_dict.items():
            self.target[joint_idx] = angle

    def compute_torques(self):
        """计算 PD 力矩"""
        # 当前关节角度和角速度
        q = self.data.qpos[7:]  # 跳过浮动基座的7个自由度
        dq = self.data.qvel[6:]  # 跳过浮动基座的6个自由度

        torques = self.kp * (self.target - q[:NUM_ACTUATORS]) - self.kd * dq[:NUM_ACTUATORS]

        # 限幅（不超过驱动器力矩范围）
        for i in range(NUM_ACTUATORS):
            limit = self.model.actuator(i).ctrlrange[1]
            torques[i] = np.clip(torques[i], -limit, limit)

        return torques


# ====================================================================
# 动作定义
# ====================================================================

def standing_pose():
    """站立姿态"""
    pose = {}
    # 腿部微弯（更稳定）
    for side in [(L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH),
                 (R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH)]:
        pose[side[0]] = -0.1   # 髋关节微前倾
        pose[side[1]] = 0.2    # 膝盖微弯
        pose[side[2]] = -0.1   # 踝关节补偿
    # 手臂自然下垂
    pose[L_SHOULDER_PITCH] = 0.3
    pose[R_SHOULDER_PITCH] = 0.3
    return pose


def wave_hand():
    """挥手打招呼"""
    poses = []
    for angle in [0.5, -0.3, 0.5, -0.3, 0.0]:
        pose = standing_pose()
        pose[R_SHOULDER_PITCH] = -1.5    # 右臂抬起
        pose[R_SHOULDER_ROLL] = -0.8     # 向外展开
        pose[R_ELBOW] = 1.2              # 弯曲肘部
        pose[R_WRIST_ROLL] = angle       # 手腕左右摆动
        poses.append(pose)
    return poses


def bow():
    """鞠躬"""
    poses = []
    # 下弯
    for angle in np.linspace(0, 0.4, 5):
        pose = standing_pose()
        pose[WAIST_PITCH] = angle
        pose[L_HIP_PITCH] = -0.1 - angle * 0.3
        pose[R_HIP_PITCH] = -0.1 - angle * 0.3
        poses.append(pose)
    # 停顿
    poses.append(poses[-1])
    # 回正
    for angle in np.linspace(0.4, 0, 5):
        pose = standing_pose()
        pose[WAIST_PITCH] = angle
        pose[L_HIP_PITCH] = -0.1 - angle * 0.3
        pose[R_HIP_PITCH] = -0.1 - angle * 0.3
        poses.append(pose)
    return poses


def punch():
    """出拳"""
    poses = []
    base = standing_pose()

    # 蓄力：双拳收腰
    pose1 = dict(base)
    pose1[L_SHOULDER_PITCH] = 0.5
    pose1[L_ELBOW] = 1.5
    pose1[R_SHOULDER_PITCH] = 0.5
    pose1[R_ELBOW] = 1.5
    pose1[WAIST_YAW] = 0.2
    poses.append(pose1)

    # 左拳出击
    pose2 = dict(base)
    pose2[L_SHOULDER_PITCH] = -1.2
    pose2[L_ELBOW] = 0.1
    pose2[R_SHOULDER_PITCH] = 0.5
    pose2[R_ELBOW] = 1.5
    pose2[WAIST_YAW] = -0.3
    poses.append(pose2)

    # 收回
    poses.append(dict(pose1))

    # 右拳出击
    pose3 = dict(base)
    pose3[R_SHOULDER_PITCH] = -1.2
    pose3[R_ELBOW] = 0.1
    pose3[L_SHOULDER_PITCH] = 0.5
    pose3[L_ELBOW] = 1.5
    pose3[WAIST_YAW] = 0.3
    poses.append(pose3)

    # 收回
    poses.append(dict(pose1))
    return poses


def raise_leg():
    """抬腿（金鸡独立）"""
    poses = []
    base = standing_pose()

    # 重心右移
    pose1 = dict(base)
    pose1[WAIST_ROLL] = -0.15
    poses.append(pose1)

    # 抬左腿
    for h in np.linspace(0, 0.8, 5):
        pose = dict(base)
        pose[WAIST_ROLL] = -0.15
        pose[L_HIP_PITCH] = -0.1 - h
        pose[L_KNEE] = 0.2 + h * 1.2
        # 双臂展开保持平衡
        pose[L_SHOULDER_PITCH] = -0.5
        pose[R_SHOULDER_PITCH] = -0.5
        pose[L_SHOULDER_ROLL] = 0.8
        pose[R_SHOULDER_ROLL] = -0.8
        poses.append(pose)

    # 停顿
    poses.append(poses[-1])
    poses.append(poses[-1])

    # 放下
    for h in np.linspace(0.8, 0, 5):
        pose = dict(base)
        pose[WAIST_ROLL] = -0.15 * (h / 0.8) if h > 0 else 0
        pose[L_HIP_PITCH] = -0.1 - h
        pose[L_KNEE] = 0.2 + h * 1.2
        poses.append(pose)

    return poses


def taichi():
    """太极起势"""
    poses = []
    base = standing_pose()

    # 双臂缓缓前举
    for t in np.linspace(0, 1, 8):
        pose = dict(base)
        # 下蹲
        pose[L_HIP_PITCH] = -0.1 - t * 0.3
        pose[R_HIP_PITCH] = -0.1 - t * 0.3
        pose[L_KNEE] = 0.2 + t * 0.6
        pose[R_KNEE] = 0.2 + t * 0.6
        pose[L_ANKLE_PITCH] = -0.1 - t * 0.2
        pose[R_ANKLE_PITCH] = -0.1 - t * 0.2
        # 双臂前举
        pose[L_SHOULDER_PITCH] = 0.3 - t * 1.8
        pose[R_SHOULDER_PITCH] = 0.3 - t * 1.8
        pose[L_SHOULDER_ROLL] = t * 0.3
        pose[R_SHOULDER_ROLL] = -t * 0.3
        poses.append(pose)

    # 停顿
    poses.append(poses[-1])
    poses.append(poses[-1])

    # 双臂缓缓下按
    for t in np.linspace(1, 0, 8):
        pose = dict(base)
        pose[L_HIP_PITCH] = -0.1 - t * 0.3
        pose[R_HIP_PITCH] = -0.1 - t * 0.3
        pose[L_KNEE] = 0.2 + t * 0.6
        pose[R_KNEE] = 0.2 + t * 0.6
        pose[L_ANKLE_PITCH] = -0.1 - t * 0.2
        pose[R_ANKLE_PITCH] = -0.1 - t * 0.2
        pose[L_SHOULDER_PITCH] = 0.3 - t * 1.8
        pose[R_SHOULDER_PITCH] = 0.3 - t * 1.8
        pose[L_SHOULDER_ROLL] = t * 0.3
        pose[R_SHOULDER_ROLL] = -t * 0.3
        poses.append(pose)

    return poses


# ====================================================================
# 主程序
# ====================================================================
def main():
    scene = os.path.join(os.path.dirname(__file__), "unitree_robots/g1/scene.xml")
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)

    # 初始化站立高度
    data.qpos[2] = 0.75
    mujoco.mj_forward(model, data)

    controller = PDController(model, data)

    # 动作序列
    actions = [
        ("站立稳定", [standing_pose()] * 5),
        ("挥手打招呼", wave_hand()),
        ("回到站立", [standing_pose()] * 3),
        ("鞠躬", bow()),
        ("左右出拳", punch()),
        ("回到站立", [standing_pose()] * 3),
        ("金鸡独立", raise_leg()),
        ("回到站立", [standing_pose()] * 3),
        ("太极起势", taichi()),
        ("回到站立", [standing_pose()] * 5),
    ]

    print("=" * 50)
    print("  宇树 G1 人形机器人动作演示")
    print("=" * 50)
    print(f"  DOF: {model.nq} | 驱动器: {model.nu}")
    print()
    print("  动作序列:")
    for i, (name, _) in enumerate(actions):
        print(f"    {i + 1}. {name}")
    print()
    print("  操作: 左键旋转 | 右键平移 | 滚轮缩放")
    print("=" * 50)

    # 启动被动 viewer（允许我们控制仿真循环）
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置初始视角
        viewer.cam.azimuth = 150
        viewer.cam.elevation = -15
        viewer.cam.distance = 3.0
        viewer.cam.lookat[:] = [0, 0, 0.5]

        step_dt = model.opt.timestep
        steps_per_keyframe = 300  # 每个关键帧持续的仿真步数

        for action_name, keyframes in actions:
            print(f"\n  >> {action_name}")

            for kf_idx, keyframe in enumerate(keyframes):
                controller.set_target(keyframe)

                for step in range(steps_per_keyframe):
                    if not viewer.is_running():
                        print("\n窗口已关闭")
                        return

                    # PD 控制计算力矩
                    torques = controller.compute_torques()
                    data.ctrl[:] = torques

                    # 物理仿真前进一步
                    mujoco.mj_step(model, data)

                    # 按实际时间同步显示
                    viewer.sync()
                    time.sleep(step_dt)

        # 演示结束，保持窗口打开
        print("\n" + "=" * 50)
        print("  所有动作演示完成！")
        print("  窗口保持打开，你可以自由旋转查看")
        print("=" * 50)

        while viewer.is_running():
            torques = controller.compute_torques()
            data.ctrl[:] = torques
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(step_dt)


if __name__ == "__main__":
    main()
