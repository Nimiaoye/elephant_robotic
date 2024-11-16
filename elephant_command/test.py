# -*- coding: utf-8 -*-

from elephant import elephant_command
import time
import random

erobot = elephant_command()


print(erobot.get_angles())

# erobot.set_angle('6', -180, 100)
# 0 - > -147.6

erobot.task_stop()

# X0 = -173.0
# Y0 = 298.0
# Z0 = 320.0 # mm
# # RX0 = 180.0
# RY0 = 0.0
# RZ0 = -22.0
# ZG = 240.0 # mm

# cur_poses_str = erobot.get_coords()
# print(cur_poses_str)
# cur_poses = [float(p) for p in cur_poses_str[12:-1].split(',')]
# print(cur_poses)
# cur_poses[5] = -10

# erobot.set_coords(cur_poses, 500)
# time.sleep(2)
# cur_poses_str = erobot.get_coords()
# print(cur_poses_str)
# # erobot.set_coord('rz', -90, 800)
# # erobot.set_coord('z', 360, 800)
# time.sleep(2)

# 9.14测试
# 笛卡尔空间
# cur_pose = erobot.get_coords()
# print(cur_pose)
# cur_pose[2] = 200
# erobot.set_coords(cur_pose, 800)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running', time.time())
#     time.sleep(0.1)

# erobot.set_coord('z', 250, 800)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running')
#     time.sleep(0.1)

# 关节空间
# cur_angles = erobot.get_angles()
# print(cur_angles)
# cur_angles[0] = -80.0
# erobot.set_angles(cur_angles, 500)
# time.sleep(0.1)
# run_flag = erobot.check_running()
# while run_flag:
#     print('running')
#     time.sleep(0.1)
#     run_flag = erobot.check_running()

# erobot.set_angle(1, -90, 500)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running')
#     time.sleep(0.1)

# for i in range(3):
#     print(i)
#     # 读取位姿
#     cur_poses = erobot.get_coords()
#     print(cur_poses)
#     time.sleep(0.1)
#     # 设定位姿
#     erobot.set_coords([100.0, 320.0, 365.0, cur_poses[3], 0.0, -22.0], 1200)
#     time.sleep(0.1)
#     while erobot.check_running():
#         print('running')
#         time.sleep(0.1)
#     erobot.set_digital_out(0, 0)
#     time.sleep(0.5)


#     for _ in range(2):
#         for _ in range(100):
#             erobot.jog_coord('x',1,50)
#             erobot.jog_coord('y',1,50)
#             erobot.jog_coord('rz',1,50)
#             time.sleep(0.02)
#         for _ in range(100):
#             erobot.jog_coord('x',-1,50)
#             erobot.jog_coord('y',-1,50)
#             erobot.jog_coord('rz',-1,50)
#             time.sleep(0.02)
 
#     print("close the speed mode")
#     erobot.jog_stop('x')
#     erobot.jog_stop('y')
#     erobot.jog_stop('rz')
#     time.sleep(0.1)
#     # erobot.task_stop()
#     # time.sleep(0.5)


# # 速度控制
# erobot.jog_coord('x',1,1000)
# erobot.jog_coord('y',1,1000)
# erobot.jog_coord('rz',1,300)
# # erobot.jog_angle(6,1,500)
# time.sleep(2)
# erobot.jog_coord('x',-1,1000)
# erobot.jog_coord('y',-1,1000)
# erobot.jog_coord('rz',-1,300)
# # erobot.jog_angle(6,-1,500)
# time.sleep(2)
# erobot.jog_coord('x',0,200)
# erobot.jog_coord('y',0,200)
# erobot.jog_coord('rz',0,200)
# # erobot.jog_angle(6,0,500)
# time.sleep(0.2)

# erobot.jog_stop('x')
# erobot.jog_stop('y')
# erobot.jog_stop('rz')


# erobot = elephant_command()

# X0 = -173.0
# Y0 = 298.0
# Z0 = 320.0 # mm
# # RX0 = 180.0
# RY0 = 0.0
# RZ0 = -114.0
# ZG = 240.0 # mm

# # 笛卡尔空间
# cur_pose = erobot.get_coords()
# print(cur_pose)
# erobot.set_coords([X0,Y0,Z0,cur_pose[3],RY0,RZ0], 1000)
# time.sleep(0.1)
# while erobot.check_running():
#     # print('running', time.time())
#     time.sleep(0.1)

# time.sleep(2)
# # 笛卡尔空间
# cur_pose = erobot.get_coords()
# print(cur_pose)
# cur_pose[0] = -140
# cur_pose[5] -= 90
# erobot.set_coords(cur_pose, 1000)
# time.sleep(0.1)
# while erobot.check_running():
#     # print('running', time.time())
#     time.sleep(0.1)

# time.sleep(2)
# cur_pose = erobot.get_coords()
# print(cur_pose)
# cur_pose[2] = ZG
# erobot.set_coords(cur_pose, 1000)
# time.sleep(0.1)
# while erobot.check_running():
#     # print('running', time.time())
#     time.sleep(0.1)


# I/O
# time.sleep(0.5)
# for _ in range(5):
#     erobot.set_digital_out(0,1)
#     time.sleep(1)
#     erobot.set_digital_out(0,0)
#     time.sleep(1)
# erobot.set_digital_out(0, 0)
# time.sleep(0.5)

# print(erobot.check_running())
# time.sleep(0.1)
