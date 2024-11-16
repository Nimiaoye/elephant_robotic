# -*- coding: utf-8 -*-
from elephant import elephant_command
import time
import random                 
import sys


erobot = elephant_command()



X0 = 114.0
Y0 = 298.0
Z0 = 320.0 # mm
# RX0 = 180.0
RY0 = 0.0
RZ0 = -120.0
ZG = 240.0 # mm

# 笛卡尔空间
cur_pose = erobot.get_coords()
print(cur_pose)
erobot.set_coords([X0,Y0,Z0,cur_pose[3],RY0,RZ0], 1000)
time.sleep(0.1)
while erobot.check_running():
    # print('running', time.time())
    time.sleep(0.1)

time.sleep(2)
# 笛卡尔空间
cur_pose = erobot.get_coords()
print(cur_pose)
cur_pose[5] = 0
erobot.set_coord('rz', cur_pose[5], 1000)
time.sleep(0.1)
while erobot.check_running():
    # print('running', time.time())
    time.sleep(0.1)


time.sleep(2)
# 笛卡尔空间
cur_pose = erobot.get_coords()
print(cur_pose)
cur_pose[5] = 150
erobot.set_coord('rz', cur_pose[5], 1000)
time.sleep(0.1)
while erobot.check_running():
    # print('running', time.time())
    time.sleep(0.1)

time.sleep(2)
cur_pose = erobot.get_coords()
print(cur_pose)
cur_pose[2] = 220.0
erobot.set_coord('z', cur_pose[2], 1000)
time.sleep(0.1)
while erobot.check_running():
    # print('running', time.time())
    time.sleep(0.1)


