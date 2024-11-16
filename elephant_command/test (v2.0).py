# -*- coding: utf-8 -*-

from elephant import elephant_command
import time
import pandas as pd

# erobot = elephant_command()

# 9.14测试
# 笛卡尔空间
# cur_pose = erobot.get_coords()
# print(cur_pose)
# cur_pose[2] = 300
# erobot.set_coords(cur_pose, 800)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running')
#     time.sleep(0.1)
# erobot.set_coord('z', 250, 800)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running')
#     time.sleep(0.1)

# 读ecxel
data_frame=pd.read_excel('E:\学习资料\毕设\MATLAB\KineCalibration\数据记录 - 3MakerRigBody.xlsx')
for iterm in data_frame.itertuples():
    theta=[iterm.q1,iterm.q2,iterm.q3,iterm.q4,iterm.q5,iterm.q6]
    erobot.set_angles(theta, 600)
    time.sleep(0.1)
    run_flag = erobot.check_running()
    while run_flag:
        print('running')
        time.sleep(0.1)
        run_flag = erobot.check_running()
    print('recording')
    time.sleep(1)


# 关节空间
# cur_angles = erobot.get_angles()
# print(cur_angles)
# cur_angles = [-73.324073,-45.450406,20.831436,-76.830769,-74.122641,-32.910895]
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

# 速度控制
# # erobot.jog_coord('x',1,1000)
# # erobot.jog_coord('y',1,1000)
# # erobot.jog_coord('rz',1,300)
# erobot.jog_angle(6,1,500)
# time.sleep(2)
# # erobot.jog_coord('x',-1,1000)
# # erobot.jog_coord('y',-1,1000)
# # erobot.jog_coord('rz',-1,300)
# erobot.jog_angle(6,-1,500)
# time.sleep(2)
# # erobot.jog_coord('x',0,200)
# # erobot.jog_coord('y',0,200)
# # erobot.jog_coord('rz',0,200)
# erobot.jog_angle(6,0,500)
# time.sleep(2)


# I/O
# time.sleep(0.5)
# for _ in range(5):
#     erobot.set_digital_out(0,1)
#     time.sleep(1)
#     erobot.set_digital_out(0,0)
#     time.sleep(1)

