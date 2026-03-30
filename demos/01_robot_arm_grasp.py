"""
示例1: 用 PyBullet 仿真一个机械臂抓取物体
========================================

这个程序做的事情：
1. 创建一个虚拟的物理世界（有重力、地面、光照）
2. 在世界中放入一个机械臂和一个方块
3. 用简单的控制策略让机械臂去抓取方块
4. 所有物理效果（碰撞、重力、摩擦）都是真实模拟的

这和真实机器人上跑的程序逻辑完全一样——
唯一的区别是数据来源从"真实传感器"变成了"仿真器API"。
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


def create_world():
    """创建仿真世界——相当于搭建一个虚拟的实验室"""

    # 连接物理引擎（GUI模式会弹出3D窗口，你可以拖拽视角）
    physics_client = p.connect(p.GUI)

    # 设置重力（和地球一样：9.81 m/s²）
    p.setGravity(0, 0, -9.81)

    # 设置仿真步长（240Hz，和真实机器人控制频率类似）
    p.setTimeStep(1.0 / 240.0)

    # 加载地面
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")

    # 设置相机视角
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0, 0.3]
    )

    return physics_client


def load_robot():
    """加载一个机械臂（Kuka iiwa）到仿真世界中

    URDF文件描述了机器人的：
    - 每个关节的位置、类型（旋转/平移）、运动范围
    - 每个连杆的质量、惯性矩、形状
    - 碰撞体和视觉模型
    """
    robot_id = p.loadURDF(
        "kuka_iiwa/model.urdf",
        basePosition=[0, 0, 0],
        useFixedBase=True  # 底座固定在地面上
    )

    num_joints = p.getNumJoints(robot_id)
    print(f"机械臂已加载！共有 {num_joints} 个关节")

    # 打印每个关节信息
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = ["旋转", "平移", "球形", "固定"][joint_info[2]]
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        print(f"  关节{i}: {joint_name} ({joint_type}), "
              f"范围: [{lower_limit:.2f}, {upper_limit:.2f}] rad")

    return robot_id


def create_target_object():
    """在桌面上创建一个要抓取的红色方块"""

    # 创建一个红色方块（0.05m边长）
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.025, 0.025, 0.025],
        rgbaColor=[1, 0, 0, 1]  # 红色
    )
    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.025, 0.025, 0.025]
    )

    # 把方块放在机械臂前方的位置
    target_pos = [0.5, 0.1, 0.025]
    target_id = p.createMultiBody(
        baseMass=0.1,  # 100克
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=target_pos
    )

    print(f"目标物体已放置在: {target_pos}")
    return target_id, target_pos


def get_end_effector_pos(robot_id):
    """获取机械臂末端执行器（手）的当前位置

    这在真实机器人上是通过正运动学计算或传感器测量得到的。
    在仿真中，我们直接从物理引擎查询。
    """
    end_effector_index = 6  # Kuka的末端关节
    state = p.getLinkState(robot_id, end_effector_index)
    return np.array(state[0])  # 世界坐标系下的位置 [x, y, z]


def simple_reach_policy(robot_id, target_pos):
    """一个简单的到达策略——让机械臂末端移动到目标位置

    这里用的是 PyBullet 的逆运动学求解器：
    给定目标位置 → 计算每个关节应该转到什么角度

    在真实的具身智能系统中，这个函数会被一个
    神经网络（AI模型）替代，直接从图像预测动作。
    """
    end_effector_index = 6

    # 逆运动学：目标位置 → 关节角度
    # 这是机器人学的核心算法之一
    joint_angles = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        target_pos
    )

    return joint_angles


def move_robot(robot_id, target_angles, speed=0.01):
    """控制机械臂移动到目标关节角度

    这相当于真实机器人上的 ros2_control 做的事：
    设置每个电机的目标位置，电机会自动转过去。
    """
    num_joints = p.getNumJoints(robot_id)
    for i in range(min(len(target_angles), num_joints)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,  # 位置控制模式
            targetPosition=target_angles[i],
            force=200,        # 最大力矩（N·m）
            maxVelocity=1.0   # 最大角速度（rad/s）
        )


def main():
    print("=" * 60)
    print("  具身智能仿真示例：机械臂抓取")
    print("  (没有真实机器人，一切都在仿真器中运行)")
    print("=" * 60)
    print()

    # 1. 创建仿真世界
    print("[1/4] 创建仿真世界（虚拟实验室）...")
    create_world()

    # 2. 加载机械臂
    print("\n[2/4] 加载机械臂 (Kuka iiwa)...")
    robot_id = load_robot()

    # 3. 放置目标物体
    print("\n[3/4] 放置目标物体（红色方块）...")
    target_id, target_pos = create_target_object()

    # 4. 开始控制循环
    print("\n[4/4] 开始控制循环——机械臂将移动到目标位置")
    print("       (这就是具身智能的核心：感知 → 决策 → 执行)")
    print()
    print("💡 你可以用鼠标拖拽旋转视角，滚轮缩放")
    print("   按 Ctrl+C 退出")
    print()

    # 分阶段运动：先到目标上方，再下降
    waypoints = [
        np.array([target_pos[0], target_pos[1], target_pos[2] + 0.3]),  # 上方
        np.array([target_pos[0], target_pos[1], target_pos[2] + 0.1]),  # 接近
        np.array([target_pos[0], target_pos[1], target_pos[2] + 0.05]),  # 抓取位置
    ]

    waypoint_names = ["移动到目标上方", "下降接近目标", "到达抓取位置"]

    try:
        for wp_idx, (waypoint, name) in enumerate(zip(waypoints, waypoint_names)):
            print(f"  阶段 {wp_idx + 1}: {name} → {waypoint}")

            # 计算到达目标需要的关节角度（逆运动学）
            target_angles = simple_reach_policy(robot_id, waypoint)

            # 移动并等待到达（加速：每步多仿真几步，减少sleep）
            for step in range(300):
                if not p.isConnected():
                    print("    [仿真器窗口已关闭]")
                    return

                move_robot(robot_id, target_angles)

                # 每次前进多步物理仿真（加速运行）
                for _ in range(4):
                    p.stepSimulation()

                # 控制仿真速度
                time.sleep(1.0 / 120.0)

                # 检查是否到达
                current_pos = get_end_effector_pos(robot_id)
                distance = np.linalg.norm(current_pos - waypoint)

                if step % 50 == 0:
                    print(f"    步骤 {step}: 末端位置={current_pos.round(3)}, "
                          f"距目标={distance:.4f}m")

                if distance < 0.03:
                    print(f"    -> 已到达！距离={distance:.4f}m")
                    break

            time.sleep(0.3)

        print()
        print("=" * 60)
        print("  抓取任务完成！")
        print()
        print("  你刚才看到的就是具身智能的基本流程：")
        print("    1. 感知：获取末端位置和目标位置")
        print("    2. 决策：逆运动学计算关节角度")
        print("    3. 执行：发送指令给（虚拟的）电机")
        print()
        print("  在真实机器人上：")
        print("    - '感知'来自真实的相机和传感器")
        print("    - '决策'由神经网络（AI模型）完成")
        print("    - '执行'通过CAN总线发给真实的电机")
        print("    但代码结构和逻辑是一样的！")
        print("=" * 60)

        # 保持窗口打开一会儿让用户观看
        print("\n保持窗口10秒后自动退出（或按 Ctrl+C 退出）...")
        for _ in range(2400):  # 约10秒
            if not p.isConnected():
                return
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print("\n仿真结束。")
    except Exception as e:
        print(f"\n仿真窗口已关闭: {e}")
    finally:
        try:
            p.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
