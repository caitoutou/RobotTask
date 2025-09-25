import os
import sys
workspace_folder_path = os.getcwd()
print(f"当前工作目录 (workspaceFolder): {workspace_folder_path}")
python_path = os.environ.get('PYTHONPATH')
if python_path:
    print(f"PYTHONPATH 环境变量的值是: {python_path}")
    # 为了看得更清楚，可以将路径拆分成列表
    print("PYTHONPATH 包含的路径有:")
    for path in python_path.split(os.pathsep): # os.pathsep 会自动处理不同系统下的分隔符 (Windows是';', Linux是':')
        print(f"  - {path}")
else:
    print("PYTHONPATH 环境变量当前未设置或为空。")
print("\nPython 解释器当前的实际搜索路径 (sys.path):")
for path in sys.path:
    print(f"  - {path}")

print("--- 调试结束 ---\n")

import math
import time
import numpy as np
import logging


from XRobot.envs import RobotEnv
from XRobot.commons.cv_utils import get_cam_intrinsic
import XRobot.commons.transform as T

logging.basicConfig(level=logging.INFO)
import cv2


def pickup_cube_demo(env):
    q = T.euler_2_quat(np.array([0, 0, 45 * np.pi / 180]))
    action = np.array([0.4, 0.3, 0.1, q[3], q[0], q[1], q[2]])

    # 运动到物块位置
    for t in range(int(200)):
        env.robot.end['arm0'].open()
        env.step(action)

    # 夹爪向下运动
    action[2] = 0.02
    for t in range(int(100)):
        env.step(action)

    # 夹爪闭合
    for t in range(int(20)):
        env.robot.end['arm0'].close()
        env.step(action)

    # 夹爪向上运动
    action[2] = 0.1
    for t in range(int(100)):
        env.step(action)

    # 运动到对应的圆盘位置
    action = np.array([0.4, -0.4, 0.1, q[3], q[0], q[1], q[2]])
    for t in range(int(200)):
        env.step(action)

    # 末端旋转
    action[3:] = np.array([0.707, 0, 0, 0.707])
    for t in range(int(100)):
        env.step(action)

    # 打开夹爪
    for t in range(int(20)):
        env.robot.end['arm0'].open()
        env.step(action)

    # 回到中间位置
    action[1] = 0
    for t in range(int(100)):
        env.step(action)


def pickup_square_nut_demo(env):
    action_init = np.array([0.3, 0.4, 0.4, 1, 0, 0, 0])
    for t in range(int(20)):
        env.step(action_init)

    cv_image = env.render("rgb_array")
    cv2.imwrite(f"image.png", cv_image)

    ## TODO: 使用基于视觉的方法获取square nuts的抓取点
    q = T.euler_2_quat(np.array([0, 0, -45 * np.pi / 180]))
    action = np.array([0.56, 0.06, 0.1, q[3], q[0], q[1], q[2]])

    # 运动到物块位置
    for t in range(int(200)):
        env.robot.end['arm0'].open()
        env.step(action)

    # 夹爪向下运动
    action[2] = 0.02
    for t in range(int(150)):
        env.step(action)

    # 夹爪闭合
    for t in range(int(20)):
        env.robot.end['arm0'].close()
        env.step(action)

    # 夹爪向上运动
    action[2] = 0.3
    for t in range(int(100)):
        env.step(action)

    # 运动到对应的圆盘位置
    action = np.array([0.65, -0.2, 0.3, q[3], q[0], q[1], q[2]])
    for t in range(int(200)):
        env.step(action)

    # 末端旋转
    action[3:] = np.array([0.707, 0, 0, -0.707])
    for t in range(int(200)):
        env.step(action)

    # 打开夹爪
    for t in range(int(20)):
        env.robot.end['arm0'].open()
        env.step(action)

    # 回到中间位置
    action[1] = 0
    for t in range(int(100)):
        env.step(action)


def main():

    robot = 'UR5e'
    # robot = 'Panda'

    env = RobotEnv(
        robot=robot+'Grasp',
        render_mode='human',
        control_freq=200,
        controller='CARTIK',
        is_show_camera_in_cv=True,
        is_render_camera_offscreen=True,
        camera_in_render="0_cam",
        # camera_in_window="frontview",     # 固定的渲染视角
    )

    ## TODO: 示例代码，可以删除，并在此编写你的代码

    start_time = time.time()
    env.reset()

    ## TODO: 抓取物块演示
    pickup_square_nut_demo(env)

    ## TODO: 抓取物块演示
    pickup_cube_demo(env)

    task_time = time.time() - start_time
    print("time: %.2f" % task_time)

    env.close()


if __name__ == "__main__":
    main()



