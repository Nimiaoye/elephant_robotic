#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    第四届中国高校智能机器人创意大赛
    主题二————俄罗斯方块
    机器人控制

    最后修改：2021.8.26
    注意：长边尽量避免位于第一象限，T方块尽量避免位于第四象限，布局时放置区域的右侧的蓝绿块要水平放置
"""
# import sys
# sys.path.append('/home/zyh/ros_workspace/src/elephant_command')
import math
import numpy as np
import rospy as ros
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from elephant_cmd import elephant_command
from robot_tetris.msg import ProcessResult, PlaceInfo
from robot_tetris.srv import AI_Tetris, AI_TetrisRequest, ImgProcess, ImgProcessRequest, Solution, SolutionRequest


CUBE_LENTH = 20 # 每个方块的边长
#末端初始状态  角度逆时针旋转为正方向  joint6初始角度最好为-100左右
INIT_STATE = {'X0':-325.0, 'Y0':113.0, 'Z0':300.0, 'RX0':179.15, 'RY0':-0.41, 'RZ0':-25, 'vel':15000}

MID_POINT = {'X':-150, 'Y':260, 'Z':310} # 中间过渡点

PLACE_ORIGIN = {'X':-213, 'Y':270, 'Z':270} # 放置原点  放置平面的左上角网格的中心
# REFERENCE_POINT_1 = {'X':-199, 'Y':315, 'Z':270} # 参考定位点  拼接盘左上角圆孔
# REFERENCE_POINT_2 = {'X':-486, 'Y':4, 'Z':270} # 参考定位点  拼接盘右下角圆孔

HEIGHT = {'before':270, 'pick':156, 'place':166} # 抓取或放置操作前的末端高度 / 操作时的末端高度

# 初赛 初始盘面为空时的拼接方案
# solution = [['I', 0, [19, 2]], ['S', 1, [18, 3]], ['Z', 1, [18, 5]], ['L', 2, [19, 8]], ['J', 0, [18, 0]], 
#             ['L', 3, [17, 9]], ['J', 3, [16, 7]], ['Z', 0, [16, 1]], ['O', 0, [16, 4]], ['O', 0, [15, 2]], 
#             ['T', 1, [15, 9]], ['T', 2, [15, 6]], ['I', 0, [14, 7]], ['O', 0, [14, 0]], ['J', 2, [14, 4]], 
#             ['S', 0, [12, 5]], ['Z', 1, [12, 3]], ['I', 0, [13, 8]], ['J', 0, [13, 0]], ['O', 0, [11, 1]], 
#             ['L', 2, [12, 9]], ['S', 1, [10, 2]], ['T', 3, [10, 0]], ['Z', 0, [10, 5]], ['S', 0, [10, 8]], 
#             ['J', 1, [10, 7]], ['T', 2, [8, 1]], ['T', 0, [9, 5]], ['Z', 1, [8, 3]], ['I', 0, [7, 2]], 
#             ['I', 0, [6, 2]], ['S', 1, [7, 5]], ['O', 0, [8, 8]], ['L', 2, [7, 9]], ['L', 2, [6, 8]]]

# 决赛 初始方块 T 0 13 4
# solution = [['Z', 0, [18, 6]], ['S', 0, [18, 2]], ['Z', 1, [18, 0]], ['T', 3, [16, 0]], ['L', 2, [17, 4]], ['L', 1, [17, 5]], 
#             ['J', 1, [19, 9]], ['Z', 0, [15, 2]], ['L', 1, [18, 7]], ['O', 0, [14, 3]], ['O', 0, [13, 0]], ['O', 0, [11, 0]], 
#             ['I', 1, [12, 2]], ['Z', 1, [16, 8]], ['O', 0, [12, 3]], ['S', 0, [14, 8]], ['L', 3, [14, 6]], ['L', 0, [13, 7]], 
#             ['Z', 0, [10, 3]], ['S', 0, [12, 6]], ['I', 1, [10, 5]], ['J', 1, [12, 9]], ['S', 1, [9, 3]], ['J', 0, [11, 6]], 
#             ['I', 1, [8, 0]], ['J', 2, [8, 6]], ['O', 0, [9, 7]], ['T', 3, [9, 1]], ['I', 1, [7, 9]], ['T', 2, [7, 2]], 
#             ['I', 0, [6, 2]], ['T', 0, [7, 5]], ['J', 1, [8, 8]], ['S', 1, [6, 6]]]

position = [[[-213.6,271.8],[-213.6,250.8],[-213.4,230.8],[-213.4,211.1],[-212.1,190.9],[-212.1,169.7],[-212.1,149.9],[-212.1,129.8],[-212.1,110.6],[-212.1,90.6]],
            [[-232.0,270.4],[-232.2,251.2],[-232.2,230.3],[-232.2,211.2],[-232.2,190.8],[-232.8,169.8],[-232.8,150.2],[-231.3,129.7],[-231.3,110.2],[-231.3,90.6]],
            [[-252.8,270.4],[-252.8,250.5],[-252.8,230.8],[-251.9,211.5],[-251.9,191.7],[-251.9,170.4],[-251.9,150.9],[-251.4,130.8],[-251.4,110.6],[-250.6,90.8]],
            [[-272.2,270.8],[-271.1,250.9],[-271.1,230.6],[-271.1,210.9],[-271.1,191.2],[-271.1,170.7],[-271.1,151.2],[-271.1,130.3],[-269.9,111.4],[-269.9,90.8]],
            [[-291.7,270.8],[-291.7,251.2],[-291.7,232.0],[-291.7,211.0],[-291.0,192.0],[-291.0,171.2],[-291.0,151.4],[-291.0,131.6],[-291.0,112.1],[-291.0,91.3]],
            [[-310.6,272.7],[-310.6,250.3],[-310.6,231.9],[-310.6,211.0],[-310.6,191.5],[-310.6,171.7],[-310.6,151.0],[-309.6,131.3],[-309.6,111.5],[-309.6,91.3]],
            [[-331.4,272.7],[-331.4,251.7],[-331.4,231.7],[-330.2,211.8],[-330.2,193.1],[-330.2,172.0],[-330.2,151.5],[-330.2,131.5],[-330.2,110.9],[-329.2,92.3]],
            [[-352.1,271.3],[-352.1,252.1],[-350.1,232.2],[-350.1,211.6],[-350.1,191.7],[-350.1,171.7],[-350.1,150.6],[-350.1,129.0],[-350.2,111.9],[-350.1,92.3]],
            [[-371.6,272.1],[-371.6,252.9],[-371.6,232.4],[-371.6,212.3],[-371.6,192.4],[-371.6,170.9],[-371.6,151.5],[-371.6,131.0],[-369.9,111.4],[-369.9,91.5]],
            [[-391.2,272.9],[-391.2,252.7],[-391.2,232.4],[-391.2,211.6],[-391.2,191.8],[-391.2,171.9],[-389.1,151.8],[-389.1,131.4],[-389.1,112.0],[-389.1,92.3]],
            [[-411.6,272.9],[-411.6,252.5],[-411.6,232.3],[-411.6,212.2],[-411.6,193.2],[-411.6,172.6],[-411.6,150.9],[-411.6,132.5],[-410.0,112.2],[-410.0,92.7]],
            [[-431.2,272.6],[-431.2,252.9],[-430.1,232.0],[-430.1,213.0],[-430.1,192.9],[-430.1,171.6],[-429.6,151.3],[-429.6,131.1],[-429.6,111.4],[-429.6,92.7]],
            [[-451.2,272.6],[-451.2,252.7],[-451.2,233.7],[-451.2,212.5],[-451.2,192.7],[-451.2,172.9],[-451.2,151.7],[-451.2,132.5],[-451.2,111.9],[-451.2,92.2]],
            [[-472.7,272.3],[-470.8,252.1],[-470.8,232.5],[-470.8,212.7],[-470.8,192.6],[-470.8,171.5],[-470.8,151.8],[-470.8,131.5],[-470.8,111.8],[-470.8,93.2]]]

# HSV threshold
hsv_th = {'blue'   : [[97,95,35], [125,255,255]], 
          'green'  : [[60,68,41], [86,255,255]],  
          'red'    : [[[0,120,100], [10,255,255]], [[160,120,100], [180,255,255]]],
          'yellow' : [[21,130,113], [34,255,255]], # RGB(255,255,0)  保留
          'purple' : [[123,42,35], [158,255,255]],
          'brown'  : [[3,74,48], [18,195,100]]} # RGB(128,0,128)

TRACk_BAR_FLAG = False

def hsv_process(image_src, DISPLAY=False, TEST=False):
    global TRACk_BAR_FLAG
    image = image_src.copy()
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    # hsv_image = cv.GaussianBlur(hsv_image, (21,21), 0)

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))

    th_blue = np.array(hsv_th['blue'])
    th_green = np.array(hsv_th['green'])
    th_red1 = np.array(hsv_th['red'][0])
    th_red2 = np.array(hsv_th['red'][1])
    th_yellow = np.array(hsv_th['yellow'])
    th_purple = np.array(hsv_th['purple'])
    th_brown = np.array(hsv_th['brown'])

    hsv_mask_blue = cv.morphologyEx(cv.inRange(hsv_image, th_blue[0], th_blue[1]), cv.MORPH_OPEN, kernel)
    hsv_mask_green = cv.morphologyEx(cv.inRange(hsv_image, th_green[0], th_green[1]), cv.MORPH_OPEN, kernel)
    # hsv_mask_red1 = cv.morphologyEx(cv.inRange(hsv_image, th_red1[0], th_red1[1]), cv.MORPH_OPEN, kernel)
    # hsv_mask_red2 = cv.morphologyEx(cv.inRange(hsv_image, th_red2[0], th_red2[1]), cv.MORPH_OPEN, kernel)
    # hsv_mask_red = hsv_mask_red1 | hsv_mask_red2
    hsv_mask_yellow = cv.morphologyEx(cv.inRange(hsv_image, th_yellow[0], th_yellow[1]), cv.MORPH_OPEN, kernel)
    hsv_mask_purple = cv.morphologyEx(cv.inRange(hsv_image, th_purple[0], th_purple[1]), cv.MORPH_OPEN, kernel)

    brown_kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
    hsv_mask_brown = cv.morphologyEx(cv.inRange(hsv_image, th_brown[0], th_brown[1]), cv.MORPH_OPEN, brown_kernel)
    # kernel = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    # hsv_mask_brown = cv.morphologyEx(hsv_mask_brown, cv.MORPH_OPEN, kernel)

    hsv_mask_red1 = cv.inRange(hsv_image, th_red1[0], th_red1[1])
    hsv_mask_red2 = cv.inRange(hsv_image, th_red2[0], th_red2[1])
    # hsv_mask_red = hsv_mask_red1 | hsv_mask_red2
    # erode_kernel = np.ones((9,9),np.uint8)
    # kernel = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    hsv_mask_red = cv.morphologyEx((hsv_mask_red1 | hsv_mask_red2), cv.MORPH_OPEN, kernel)
    # hsv_mask_red = cv.erode((hsv_mask_red1 | hsv_mask_red2), kernel, iterations = 1)
    # hsv_mask_red = hsv_mask_red1 | hsv_mask_red2

    masks = {'O': hsv_mask_red, 'I': hsv_mask_red, 'J': hsv_mask_purple, 'L': hsv_mask_yellow,
            'S': hsv_mask_green, 'Z': hsv_mask_blue, 'T': hsv_mask_brown}

    segmentation_mask = hsv_mask_blue | hsv_mask_green | hsv_mask_yellow | hsv_mask_red | hsv_mask_purple | hsv_mask_brown

    segmentation_result = cv.bitwise_and(image, image, mask=segmentation_mask)

    # cv.imshow('mask', hsv_mask_brown)
    cv.waitKey(5)

    if DISPLAY:
        cv.imshow("segmentation_result", segmentation_result)
        cv.waitKey(3)

    if TEST:
        if TRACk_BAR_FLAG is False:
            TRACk_BAR_FLAG = True
            cv.namedWindow("hsv_test_window")
            cv.createTrackbar("H_min", "hsv_test_window", th_blue[0][0], 180, onTrackbar)
            cv.createTrackbar("H_max", "hsv_test_window", th_blue[1][0], 180, onTrackbar)
            cv.createTrackbar("S_min", "hsv_test_window", th_blue[0][1], 255, onTrackbar)
            cv.createTrackbar("S_max", "hsv_test_window", th_blue[1][1], 255, onTrackbar)
            cv.createTrackbar("V_min", "hsv_test_window", th_blue[0][2], 255, onTrackbar)
            cv.createTrackbar("V_max", "hsv_test_window", th_blue[1][2], 255, onTrackbar)

        Hmin = cv.getTrackbarPos("H_min", "hsv_test_window")
        Hmax = cv.getTrackbarPos("H_max", "hsv_test_window")
        Smin = cv.getTrackbarPos("S_min", "hsv_test_window")
        Smax = cv.getTrackbarPos("S_max", "hsv_test_window")
        Vmin = cv.getTrackbarPos("V_min", "hsv_test_window")
        Vmax = cv.getTrackbarPos("V_max", "hsv_test_window")

        lower = np.array([Hmin, Smin, Vmin]) #测试调整阈值用
        upper = np.array([Hmax, Smax, Vmax])
        hsv_mask_test = cv.inRange(hsv_image, lower, upper) 
        image_sprtn_test = cv.bitwise_and(image, image, mask=hsv_mask_test) #合并mask与原图像，实现分割
        cv.imshow("hsv_test_window", image_sprtn_test)
        cv.waitKey(3)
    
    return segmentation_mask, masks

def onTrackbar(x):
    pass

class MyRobot:
    def __init__(self):
        ros.init_node('Robot_node')
        ros.loginfo('Start Robot_node...')
        
        self.cv_bridge = CvBridge()
        self.robot_cmd = elephant_command()
        self.init_move()
        ros.sleep(0.5)

        ros.wait_for_service("/object_detection") # 获取待抓取骨牌的信息
        self.img_client = ros.ServiceProxy("/object_detection", ImgProcess)
        # ros.wait_for_service('/ai_tetris') # 针对某一特定形状的骨牌，调用AI算法获得最佳摆放位置
        # self.tetris_client = ros.ServiceProxy('/ai_tetris', AI_Tetris)
        ros.wait_for_service('/solution') # 获取全局搜索拼接方案，初始方块信息通过命令行参数给定
        self.solution_client = ros.ServiceProxy('/solution', Solution)
        ros.sleep(0.5)

        self.objects = [] # 待抓取的骨牌信息，每次抓取前进行更新

    def init_move(self):
        """
        将机械臂末端设置为初始状态
        """
        # 读取位姿
        pose_cur = self.robot_cmd.get_coords()
        print("current pose: ", pose_cur)
        ros.sleep(0.1)
        # 设定初始位姿
        self.robot_cmd.set_coords([INIT_STATE['X0'], INIT_STATE['Y0'], INIT_STATE['Z0'], 
                                    INIT_STATE['RX0'], INIT_STATE['RY0'], INIT_STATE['RZ0']], INIT_STATE['vel'])
        ros.sleep(0.1)
        while self.robot_cmd.check_running():
            ros.sleep(0.1)
        self.robot_cmd.set_digital_out(0, 0)
        ros.sleep(0.1)

    def set_pose_move(self, pstn_3d, delta_z_angle):
        """
        pstn_3d: 设定空间点坐标
        delta_z_angle: 设定末端绕Z轴旋转角度增量
        """
        pose_cur = self.robot_cmd.get_coords()
        print("current pose: ", pose_cur)
        pose_cur[0] = pstn_3d[0]
        pose_cur[1] = pstn_3d[1]
        pose_cur[2] = pstn_3d[2]
        pose_cur[5] += delta_z_angle # 考虑角度插补
        # 设定位姿
        self.robot_cmd.set_coords(pose_cur, INIT_STATE['vel'])
        ros.sleep(0.1)
        while self.robot_cmd.check_running():
            ros.sleep(0.1)
            # print('running')
        ros.sleep(0.1)

    def z_move(self, delta_z):
        """
        沿着z轴移动  delta_z  mm
        """
        pose_cur = self.robot_cmd.get_coords()
        pose_cur[2] += delta_z

        self.robot_cmd.set_coords(pose_cur, INIT_STATE['vel'])
        ros.sleep(0.1)
        while self.robot_cmd.check_running():
            ros.sleep(0.1)
        ros.sleep(0.1)

    def position_trans(self, pixel_pstn, hand_camera):
        """
        得到物体在基坐标系中的位置  eye-in-hand
        hand_camera: 手眼关系  0 -- eye-in-hand  |  1 -- eye-to-hand
        pixel_pstn[0] -- u
        pixel_pstn[1] -- v
        """
        #相机到机械臂末端的偏移
        L1 = 55.0 #基坐标系x方向 mm 
        L2 = 32.0 #基坐标系y方向

        intrinsic = np.array([[618.674, 0, 329.289],
                              [0, 615.679, 236.361],
                              [0,       0,       1]], dtype=np.float)
        R_camera_2_robot = np.array([[-0.05462911, -0.99847776, -0.00760379],
                                     [-0.98745868,  0.05289342,  0.14875364],
                                     [-0.14812501,  0.01563471, -0.98884505]], dtype=np.float)
        t_camera_2_robot = np.array([[0.48342977],
                                     [-0.07321325],
                                     [0.63483207]], dtype=np.float)

        pose_cur = self.robot_cmd.get_coords()

        if hand_camera == 0:
             #像素坐标系 -> 图像坐标系
            x = pixel_pstn[0] - 320
            y = pixel_pstn[1] - 240
            # 图像坐标系 -> 相机坐标系
            # z高度下的视野范围 (z * 0.9781, z * 0.7192)
            X = x * (pose_cur[2] * 0.9781) / 640 # 单位 mm
            Y = y * (pose_cur[2] * 0.7192) / 480

            X1 = pose_cur[0] - Y - L1 #物体在基坐标系中的位置
            Y1 = pose_cur[1] - X + L2
        elif hand_camera == 1:
            # X1 = -249 + 1.0417 * (480 - pixel_pstn[1]) + 9
            # Y1 = 70 + 0.9844 * (640 - pixel_pstn[0]) + 3

            pos_pixel = np.array([[pixel_pstn[0]], [pixel_pstn[1]], [1]], dtype=np.float)
            pos_camera = np.dot(np.linalg.inv(intrinsic), pos_pixel) * 0.63483207
            pos_robot = np.dot(np.linalg.inv(R_camera_2_robot.T), (pos_camera - t_camera_2_robot))
            X1, Y1 = pos_robot[0][0] * 1000 + 4, pos_robot[1][0] * 1000 - 4 # 降唯
        
        return [X1, Y1, HEIGHT['before']]
        
    def gripper_close(self):
        """ 夹爪关闭 """
        # ros.sleep(0.1)
        self.robot_cmd.set_digital_out(1, 1)
        self.robot_cmd.set_digital_out(1, 1)
        self.robot_cmd.set_digital_out(1, 1)
        ros.sleep(0.1)

    def gripper_open(self):
        """ 
            夹爪打开
            0 端口对应夹爪   1 端口对应吸盘
        """
        ros.sleep(0.1)
        self.robot_cmd.set_digital_out(1, 0)
        self.robot_cmd.set_digital_out(1, 0)
        self.robot_cmd.set_digital_out(1, 0)
        ros.sleep(0.1)

    def pick(self, delta_z_abs):
        print("picking...")
        # self.gripper_open()
        self.z_move(-delta_z_abs)
        self.gripper_close()
        self.z_move(delta_z_abs)

    def place(self, delta_z_abs):
        print("placing...")
        self.z_move(-delta_z_abs)
        self.gripper_open()
        self.z_move(delta_z_abs)

    def detect_color(self, image):
        """
            根据颜色识别不同的俄罗斯方块
        """
        image_bgr = image.copy()
        SHAPE = ['L', 'S', 'Z', 'I', 'J', 'T'] # O形状包含在I情况下
        segmentation_mask, masks = hsv_process(image_bgr, DISPLAY=False, TEST=False)

        objects = []
        # print('*'*10)

        for shape in SHAPE:
            mask = masks[shape]
            # cv.imshow('mask_{}'.format(shape), mask)
            # cv.waitKey(0)
            contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]

            # if False:
            if len(contours) > 0:
                for contour in contours:
                    object_info = {'shape':shape, 'center':None, 'angle':None}
                    # print(cv.contourArea(contour))
                    # print(cv.arcLength(contour, True))
                    if cv.contourArea(contour) < 6000 or cv.arcLength(contour, True) > 8000:
                        continue
                    epsilon = 0.036 * cv.arcLength(contour, True)
                    vertex = np.squeeze(cv.approxPolyDP(contour, epsilon, True)) # shape:(n,2) n个顶点

                    min_rect = cv.minAreaRect(contour)
                    min_rect_points = np.array(cv.boxPoints(min_rect), dtype=np.float) # 顺时针排序

                    x_list = [vertex[i][0] for i in range(len(vertex))]
                    x_min_pos, x_max_pos = np.argmin(x_list), np.argmax(x_list)
                    y_list = [vertex[i][1] for i in range(len(vertex))]
                    y_min_pos, y_max_pos = np.argmin(y_list), np.argmax(y_list)

                    side = [] # 轮廓的各边长
                    for i in range(len(vertex)):
                        side.append(np.sqrt((vertex[i][0]-vertex[(i+1) % len(vertex)][0]) ** 2 + 
                                            (vertex[i][1]-vertex[(i+1) % len(vertex)][1]) ** 2))
                    side_max_pos, side_min_pos = np.argmax(side), np.argmin(side)

                    if shape == 'I':
                        # print(cv.contourArea(contour))
                        if cv.contourArea(contour) > 20000:
                            # 粘连情况
                            pass

                        else:
                            if np.var(side) < 50:
                                object_info['shape'] = 'O'
                                object_info['angle'] = -min_rect[2]
                                center_x = int(min_rect_points[1][0] + (min_rect_points[3][0] - min_rect_points[1][0]) / 4)
                                center_y = int(min_rect_points[1][1] + (min_rect_points[3][1] - min_rect_points[1][1]) / 4)
                                object_info['center'] = [center_x, center_y]
                            else:
                                object_info['shape'] = 'I'
                                if min_rect[1][0] > min_rect[1][1]:
                                    # angle < 90
                                    object_info['angle'] = -min_rect[2]
                                    # 找到上下两条短边的中点
                                    point1 = [(min_rect_points[0][0] + min_rect_points[1][0]) / 2, (min_rect_points[0][1] + min_rect_points[1][1]) / 2]
                                    point2 = [(min_rect_points[2][0] + min_rect_points[3][0]) / 2, (min_rect_points[2][1] + min_rect_points[3][1]) / 2]
                                    center_x = int(point2[0] + (point1[0] - point2[0]) * 3 / 8)
                                    center_y = int(point2[1] + (point1[1] - point2[1]) * 3 / 8)
                                    object_info['center'] = [center_x, center_y]
                                else:
                                    # angle > 90
                                    object_info['angle'] = 90 - min_rect[2]
                                    # 找到上下两条短边的中点
                                    point1 = [(min_rect_points[0][0] + min_rect_points[3][0]) / 2, (min_rect_points[0][1] + min_rect_points[3][1]) / 2]
                                    point2 = [(min_rect_points[1][0] + min_rect_points[2][0]) / 2, (min_rect_points[1][1] + min_rect_points[2][1]) / 2]
                                    center_x = int(point2[0] + (point1[0] - point2[0]) * 3 / 8)
                                    center_y = int(point2[1] + (point1[1] - point2[1]) * 3 / 8)
                                    object_info['center'] = [center_x, center_y]
                                # print('red_obj_info: ',object_info)

                    elif shape == 'J':
                        # print(len(vertex))
                        if len(vertex) == 6:
                            point1 = [vertex[side_max_pos][0], vertex[side_max_pos][1]] # 外拐顶点
                            point2 = [vertex[(side_max_pos + 3) % 6][0], vertex[(side_max_pos + 3) % 6][1]] # 内拐顶点
                            object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                            # 以外拐顶点为原点  按照长边所在象限进行分类  角度范围[-180,180)
                            # if side_max_pos == (y_max_pos + 1) % 6 and side_max_pos == (x_min_pos + 2) % 6:
                            #     # 90°特殊情况
                            #     object_info['angle'] = 90.0
                            if side_max_pos == y_max_pos:
                                # 长边在第一象限  外拐顶点的y最大  角度范围[0,90)
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(side_max_pos+1) % 6][1] - vertex[side_max_pos][1]), 
                                                                                abs(vertex[(side_max_pos+1) % 6][0] - vertex[side_max_pos][0])))
                            elif side_max_pos == x_max_pos:
                                # 长边在第二象限  外拐顶点x最大  角度范围[90,180)
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(side_max_pos+1) % 6][0] - vertex[side_max_pos][0]), 
                                                                                abs(vertex[(side_max_pos+1) % 6][1] - vertex[side_max_pos][1]))) + 90
                            elif side_max_pos == y_min_pos or (side_max_pos+1) % 6 == y_min_pos:
                                # 长边在第三象限  外拐顶点y最小  角度范围[-180,-90) ****
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(side_max_pos+1) % 6][1] - vertex[side_max_pos][1]), 
                                                                                abs(vertex[(side_max_pos+1) % 6][0] - vertex[side_max_pos][0]))) - 180
                            elif side_max_pos == x_min_pos:
                                # 长边在第四象限  外拐顶点x最小  角度范围[-90,0)
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(side_max_pos+1) % 6][0] - vertex[side_max_pos][0]), 
                                                                                abs(vertex[(side_max_pos+1) % 6][1] - vertex[side_max_pos][1]))) - 90

                    elif shape == 'L':
                        if len(vertex) == 6:
                            point1 = [vertex[(side_max_pos+1) % 6][0], vertex[(side_max_pos+1) % 6][1]] # 外拐顶点
                            point2 = [vertex[(side_max_pos-2) % 6][0], vertex[(side_max_pos-2) % 6][1]] # 内拐顶点
                            object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                            # 以外拐顶点为原点  按照长边所在象限进行分类  角度范围[-180,180)
                            if side_max_pos == x_min_pos +1 and side_max_pos == y_max_pos and x_max_pos == 0:
                                # -180特殊情况
                                object_info['angle'] = -180.0
                            elif side_max_pos == x_max_pos + 1 and side_max_pos == y_min_pos and y_max_pos == 0:
                                # 0°特殊情况
                                object_info['angle'] = 0.0
                            elif (side_max_pos + 1) % 6 == x_min_pos:
                                # 长边在第一象限  外拐顶点的x最小  角度范围[0,90)
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(x_min_pos-1) % 6][1] - vertex[x_min_pos][1]), 
                                                                            abs(vertex[(x_min_pos-1) % 6][0] - vertex[x_min_pos][0])))
                            elif (side_max_pos + 1) % 6 == y_max_pos:
                                # 长边在第二象限  外拐顶点y最大  角度范围[90,180)
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(y_max_pos-1) % 6][0] - vertex[y_max_pos][0]), 
                                                                            abs(vertex[(y_max_pos-1) % 6][1] - vertex[y_max_pos][1]))) + 90
                            elif (side_max_pos + 1) % 6 == x_max_pos:
                                # 长边在第三象限  外拐顶点x最大  角度范围[-180,-90)
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(x_max_pos-1) % 6][1] - vertex[x_max_pos][1]), 
                                                                            abs(vertex[(x_max_pos-1) % 6][0] - vertex[x_max_pos][0]))) - 180
                            elif (side_max_pos + 1) % 6 == y_min_pos or (side_max_pos + 1 + 1) % 6 == y_min_pos:
                                # 长边在第四象限  外拐顶点y最小  角度范围[-90,0)
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[((side_max_pos + 1) % 6 - 1) % 6][0] - vertex[(side_max_pos + 1) % 6][0]), 
                                                                            abs(vertex[((side_max_pos + 1) % 6 - 1) % 6][1] - vertex[(side_max_pos + 1) % 6][1]))) - 90

                    elif shape == 'S' or shape == 'Z':
                        # print(len(vertex))
                        # print(vertex[x_min_pos], vertex[x_max_pos])

                        if abs(vertex[x_min_pos][1] - vertex[x_max_pos][1]) < 20 and \
                            abs(x_list[x_min_pos] - x_list[x_max_pos] > abs(y_list[y_min_pos] - y_list[y_max_pos])):
                            # 最小矩形识别异常
                            if shape == 'S':
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(x_min_pos-1) % 8][1] - vertex[x_min_pos][1]), 
                                                                                abs(vertex[(x_min_pos-1) % 8][0] - vertex[x_min_pos][0]))) + 95
                                center_x = int((vertex[(x_min_pos+1)%8][0] + vertex[(x_min_pos-2)%8][0]) / 2)
                                center_y = int((vertex[(x_min_pos+1)%8][1] + vertex[(x_min_pos-2)%8][1]) / 2)
                                object_info['center'] = [center_x, center_y]
                            elif shape == 'Z':
                                object_info['angle'] = math.degrees(math.atan2(abs(vertex[(x_min_pos-1) % 8][1] - vertex[x_min_pos][1]), 
                                                                                abs(vertex[(x_min_pos-1) % 8][0] - vertex[x_min_pos][0])))
                                center_x = int((vertex[(x_min_pos-1)%8][0] + vertex[(x_min_pos+2)%8][0]) / 2)
                                center_y = int((vertex[(x_min_pos-1)%8][1] + vertex[(x_min_pos+2)%8][1]) / 2)
                                object_info['center'] = [center_x, center_y]

                            # print(object_info['angle'])

                        # if min_rect[1][0] - side[side_max_pos] > 20:
                        elif min_rect[1][0] > min_rect[1][1]:
                            # 最小矩形的w大于长边  第一象限
                            object_info['angle'] = -min_rect[2]
                            # 找到矩形两条长边的中点
                            point1 = [(min_rect_points[1][0] + min_rect_points[2][0]) / 2, (min_rect_points[1][1] + min_rect_points[2][1]) / 2]
                            point2 = [(min_rect_points[0][0] + min_rect_points[3][0]) / 2, (min_rect_points[0][1] + min_rect_points[3][1]) / 2]
                            center_x = int(point1[0] + (point2[0] - point1[0]) / 4)
                            center_y = int(point1[1] + (point2[1] - point1[1]) / 4)
                            object_info['center'] = [center_x, center_y]
                        else:
                            # 最小矩形的w等于长边  第二象限
                            object_info['angle'] = -min_rect[2] + 90
                            # 找到矩形两条长边的中点
                            point1 = [(min_rect_points[0][0] + min_rect_points[1][0]) / 2, (min_rect_points[0][1] + min_rect_points[1][1]) / 2]
                            point2 = [(min_rect_points[2][0] + min_rect_points[3][0]) / 2, (min_rect_points[2][1] + min_rect_points[3][1]) / 2]
                            center_x = int(point1[0] + (point2[0] - point1[0]) / 4)
                            center_y = int(point1[1] + (point2[1] - point1[1]) / 4)
                            object_info['center'] = [center_x, center_y]

                    elif shape == 'T':
                        object_info['shape'] = 'T'

                        # 根据最小矩形顶点附近的是否存在掩码判断方块位于哪一象限
                        min_rect_point_0_sum = np.sum(mask[min_rect_points[0][1]-10:min_rect_points[0][1]+10, 
                                                            min_rect_points[0][0]-10:min_rect_points[0][0]+10])
                        min_rect_point_1_sum = np.sum(mask[min_rect_points[1][1]-10:min_rect_points[1][1]+10, 
                                                            min_rect_points[1][0]-10:min_rect_points[1][0]+10])
                        # print('0: ',min_rect_point_0_sum)
                        # print('1: ',min_rect_point_1_sum)

                        if min_rect_point_0_sum > 10000 and min_rect_point_1_sum < 100: # 第一象限
                            object_info['angle'] = -min_rect[2]
                            point1 = [min_rect_points[0][0] + (min_rect_points[1][0] - min_rect_points[0][0]) / 4,
                                        min_rect_points[0][1] + (min_rect_points[1][1] - min_rect_points[0][1]) / 4]
                            point2 = [min_rect_points[3][0] + (min_rect_points[2][0] - min_rect_points[3][0]) / 4,
                                        min_rect_points[3][1] + (min_rect_points[2][1] - min_rect_points[3][1]) / 4]
                            object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]
                        elif min_rect_point_0_sum < 100 and min_rect_point_1_sum < 100: # 第二象限
                            object_info['angle'] = -min_rect[2] + 90
                            if object_info['angle'] == 180:
                                object_info['angle'] = -180
                            point1 = [min_rect_points[2][0] + (min_rect_points[1][0] - min_rect_points[2][0]) / 4,
                                        min_rect_points[2][1] + (min_rect_points[1][1] - min_rect_points[2][1]) / 4]
                            point2 = [min_rect_points[3][0] + (min_rect_points[0][0] - min_rect_points[3][0]) / 4,
                                        min_rect_points[3][1] + (min_rect_points[0][1] - min_rect_points[3][1]) / 4]
                            object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]
                        elif min_rect_point_0_sum < 100 and min_rect_point_1_sum > 10000: # 第三象限
                            object_info['angle'] = -min_rect[2] - 180
                            point1 = [min_rect_points[1][0] + (min_rect_points[0][0] - min_rect_points[1][0]) / 4,
                                        min_rect_points[1][1] + (min_rect_points[0][1] - min_rect_points[1][1]) / 4]
                            point2 = [min_rect_points[2][0] + (min_rect_points[3][0] - min_rect_points[2][0]) / 4,
                                        min_rect_points[2][1] + (min_rect_points[3][1] - min_rect_points[2][1]) / 4]
                            object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]
                        elif min_rect_point_0_sum > 10000 and min_rect_point_1_sum > 10000: # 第四象限
                            object_info['angle'] = (-min_rect[2]) - 90
                            point1 = [min_rect_points[0][0] + (min_rect_points[3][0] - min_rect_points[0][0]) / 4,
                                        min_rect_points[0][1] + (min_rect_points[3][1] - min_rect_points[0][1]) / 4]
                            point2 = [min_rect_points[1][0] + (min_rect_points[2][0] - min_rect_points[1][0]) / 4,
                                        min_rect_points[1][1] + (min_rect_points[2][1] - min_rect_points[1][1]) / 4]
                            object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                    # print(object_info)

                    objects.append(object_info)
                    x,y,w,h = cv.boundingRect(contour)
                    cv.rectangle(image_bgr, (x,y), (x + w,y + h), (100,100,150), 2)
                    y = y + h + 30 if y - 30 < 0 else y
                    try:
                        cv.circle(image_bgr, (object_info['center'][0], object_info['center'][1]), 5, (0,0,0), -1)
                        cv.putText(image_bgr, '{}'.format(object_info['shape']), (x+10, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                        cv.putText(image_bgr, '{}'.format(int(object_info['angle'])), (x+30, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                    except:
                        pass

        # plt.figure(figsize=(10,8))
        # plt.imshow(cv.cvtColor(image_bgr, cv.COLOR_BGR2RGB))
        # plt.show()

        return image_bgr, objects

    def servo(self, domino_info):
        current_pstn = np.array([0,0], dtype=np.int32)
        target_pstn = np.array([408, 110], dtype=np.int32)
        while True:
            rgb_image = ros.wait_for_message("/camera2/color/image_raw", Image, timeout=None)
            rgb_image = np.ndarray(shape=(rgb_image.height, rgb_image.width, 3), 
                                    dtype=np.uint8, buffer=rgb_image.data)
            image_bgr = cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR)

            detect_image, objects = self.detect_color(image_bgr)
            cv.imshow('eye-in-hand', detect_image)

            min_distance = 99999
            current_obj = None

            for obj in objects:
                if obj['shape'] == domino_info['shape']:
                    # print('obj: ', obj)
                    if obj['center'] is not None:
                        if sum(abs(target_pstn - obj['center'])) < min_distance:
                            min_distance = sum(abs(target_pstn - obj['center']))
                            current_obj = obj
            # print('current_obj: ', current_obj)

            if current_obj is not None:
                error = current_obj['center'] - target_pstn
                if sum(abs(error)) < 6:
                    self.robot_cmd.jog_coord('x', 0, 0)
                    self.robot_cmd.jog_coord('y', 0, 0)
                    domino_info['angle'] = current_obj['angle']
                    cv.destroyWindow('eye-in-hand')
                    break
                else:
                    self.robot_cmd.jog_coord('x', -1 if error[1] > 0 else 1, min(abs(20 * error[1])+50, 3000))
                    self.robot_cmd.jog_coord('y', -1 if error[0] > 0 else 1, min(abs(20 * error[0])+50, 3000))
            else:
                print('No target shape found...')
            
            # cv.waitKey(3)

        # 在手上的相机进行目标检测，获取抓取信息  调用服务，获取相应的放置信息
        # try:
        #     request = AI_TetrisRequest()
        #     request.type = current_obj['shape']
        #     response = self.tetris_client(request)

            # domino_info = {'shape':current_obj['shape'], 'pixel_pstn':current_obj['center'], 'angle':current_obj['angle'],
            #                 'place_center':response.center, 'place_state':response.state}
        #     print('place_info: ', domino_info)
        # except ros.ServiceException as e:
        #     print("ai_tetris service called failed: %s" %e)

        return domino_info

    def get_next_domino(self):
        """ 调用图像处理服务，获取所有目标信息
            选择第一块骨牌, 获取抓取信息  中心点和角度 
        """
        while True:
            try:
                request = ImgProcessRequest()
                request.signal = ['call object_detection service...']
                response = self.img_client(request)
                if response.result == "successful":
                    self.objects = response.objects
                    break
            except ros.ServiceException as e:
                print("object detection service called failed: %s" %e)

        if len(self.objects) > 0:
            print('There are {} dominos in total.'.format(len(self.objects)))
            # obj_info = {'shape':self.objects[0].type, 'center':self.objects[0].center, 
            #             'angle':self.objects[0].theta}
            # print('object_info: ', obj_info)

            # domino_info = {'shape':obj_info['shape'], 'pixel_pstn':obj_info['center'], 'angle':obj_info['angle'],
            #                 'place_center':None, 'place_state':None}
            # print('place_info: ', domino_info)

            # return domino_info
            return self.objects
        else:
            # 无剩余骨牌
            print('No domino left.')
            return None

    def set_pick_position(self, pstn_3d):
        """
        pstn_3d: 设定空间点坐标
        delta_z_angle: 设定末端绕Z轴旋转角度增量
        """
        pose_cur = self.robot_cmd.get_coords()
        print("current pose: ", pose_cur)
        pose_cur[0] = pstn_3d[0]
        pose_cur[1] = pstn_3d[1]
        pose_cur[2] = pstn_3d[2]
        pose_cur[5] = INIT_STATE['RZ0'] # 考虑角度插补
        # 设定位姿
        self.robot_cmd.set_coords(pose_cur, INIT_STATE['vel'])
        ros.sleep(0.1)
        while self.robot_cmd.check_running():
            ros.sleep(0.1)

    def set_place_position(self, domino_info):
        """
            针对不同的骨牌形状，控制机械臂运动到待放置位置
        """
        place_x = position[domino_info['place_center'][0]-6][domino_info['place_center'][1]][0]
        place_y = position[domino_info['place_center'][0]-6][domino_info['place_center'][1]][1]
        place_z = HEIGHT['place'] # INIT_STATE['Z0']
        delta_angle = 3 # 拼接盘初始角度

        if domino_info['shape'] == 'O':
            delta_angle += -domino_info['angle'] # 顺时针旋转到状态0
            self.set_pose_move([place_x, place_y, place_z], delta_angle) # 移动到待放置位置

        elif domino_info['shape'] == 'I' or domino_info['shape'] == 'S' or domino_info['shape'] == 'Z':
            if domino_info['place_state'] == 0: # 横摆放
                if domino_info['angle'] <= 90:
                    delta_angle += -domino_info['angle'] # 顺时针旋转到状态0
                    self.set_pose_move([place_x, place_y, place_z], delta_angle) # 移动到待放置位置
                else: # 角度超过90,考虑插补 ******************************************************
                    place_y -= 2 # 角度变化为钝角时放置位置存在误差，将y适当减小
                    pose_cur = self.robot_cmd.get_coords()
                    if pose_cur[5] - domino_info['angle'] < -180:
                        # 若顺时针旋转，虽然转角小于180,但rz超出-180限制
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        delta_angle += 180 - domino_info['angle'] # 逆时针旋转超过180度，到相同位置
                        # print('delta-angle: ', delta_angle)
                    else:
                        delta_angle += -domino_info['angle']
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
            elif domino_info['place_state'] == 1: # 竖放置
                if domino_info['angle'] <= 90:
                    delta_angle += 90 - domino_info['angle'] # 逆时针旋转到状态1
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                else:
                    delta_angle += -(domino_info['angle'] - 90) # 顺时针旋转到状态1
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)

        elif domino_info['shape'] == 'J' or domino_info['shape'] == 'L' or domino_info['shape'] == 'T':
            if domino_info['place_state'] == 0: # 0度摆放
                if domino_info['angle'] <= 90 and domino_info['angle'] > 0: # 位于第一象限
                    delta_angle += -domino_info['angle'] # 顺时针旋转到0度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] <= 0 and domino_info['angle'] >= -90: # 位于第四象限
                    delta_angle += -domino_info['angle'] # 逆时针旋转到0度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] > 90: # 位于第二象限  角度超过90,考虑插补 ***************
                    pose_cur = self.robot_cmd.get_coords()
                    if pose_cur[5] - domino_info['angle'] < -180:
                        # 若顺时针旋转，虽然转角小于180,但rz可能超出-180限制
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        delta_angle += 180 - domino_info['angle'] # 逆时针旋转超过180度，到相同位置
                        # print('delta-angle: ', delta_angle)
                    else:
                        delta_angle += -domino_info['angle']
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                else: # 位于第三象限  角度超过90,考虑插补 **********************************
                    # 初始rz = -25, 逆时针旋转，rz增加，不会超出180上限
                    delta_angle += -domino_info['angle'] # 逆时针旋转到状态0
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)

            elif domino_info['place_state'] == 1: # 90度摆放
                if domino_info['angle'] <= 90 and domino_info['angle'] >= 0: # 位于第一象限
                    delta_angle += 90 - domino_info['angle'] # 逆时针旋转到90度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] > 90: # 位于第二象限
                    delta_angle += -(domino_info['angle'] - 90) # 顺时针旋转到90度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] >= -90 and domino_info['angle'] < 0: # 位于第四象限
                    # 角度超过90,考虑插补 **********************************
                    # 初始rz = -25, 逆时针旋转，rz增加，不会超出180上限
                    delta_angle += -domino_info['angle'] + 90 # 逆时针旋转到状态1
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                else: # 位于第三象限  角度范围[-180,-90)
                    # 角度超过90,考虑插补 **********************************
                    # 若顺时针旋转，虽然转角小于180,但rz可能超出-180限制
                    pose_cur = self.robot_cmd.get_coords()
                    if pose_cur[5] - (90 + 180 + domino_info['angle']) < -180:
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        delta_angle += -domino_info['angle'] - 90 # 逆时针旋转超过180度，到相同位置
                        # print('delta-angle: ', delta_angle)
                    else:
                        delta_angle += -(90 + 180 + domino_info['angle']) # 顺时针旋转到状态1
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)

            elif domino_info['place_state'] == 2: # -180度放置
                if domino_info['angle'] >= 90: # 位于第二象限
                    delta_angle += 180 - domino_info['angle'] # 逆时针旋转到-180度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] <= -90: #位于第三象限
                    delta_angle += -(180 + domino_info['angle']) # 顺时针旋转到-180度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] >= 0 and domino_info['angle'] < 90: # 位于第一象限
                    # 角度超过90,考虑插补 **********************************
                    # 初始rz = -25, 逆时针旋转，rz增加，不会超出180上限
                    delta_angle += 180 - domino_info['angle'] # 逆时针旋转到状态2
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                else: # 位于第四象限  角度范围[-90,0]
                    # 角度超过90,考虑插补 **********************************
                    # 若顺时针旋转，虽然转角小于180,但rz可能超出-180限制
                    pose_cur = self.robot_cmd.get_coords()
                    if pose_cur[5] - (180 + domino_info['angle']) < -180:
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        delta_angle += -domino_info['angle'] # 逆时针旋转超过180度，到相同位置
                        # print('delta-angle: ', delta_angle)
                    else:
                        delta_angle += -(180 + domino_info['angle']) # 顺时针旋转到状态2
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)

            elif domino_info['place_state'] == 3: # -90度放置
                if domino_info['angle'] >= -90 and domino_info['angle'] <= 0: # 位于第四象限
                    delta_angle += -(90 + domino_info['angle']) # 顺时针旋转到-90度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] < -90: # 位于第三象限
                    delta_angle += -domino_info['angle'] - 90 # 逆时针旋转到-90度状态
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                elif domino_info['angle'] > 0 and domino_info['angle'] <= 90: # 位于第一象限
                    # 角度超过90,考虑插补 **********************************
                    # 若顺时针旋转，虽然转角小于180,但rz可能超出-180限制
                    pose_cur = self.robot_cmd.get_coords()
                    if pose_cur[5] - (90 + domino_info['angle']) < -180:
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 90)
                        delta_angle += 90 - domino_info['angle'] # 逆时针旋转超过180度，到相同位置
                        # print('delta-angle: ', delta_angle)
                    else:
                        delta_angle += -(90 + domino_info['angle']) # 顺时针旋转到状态3
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)
                else: # 位于第二象限
                    # 角度超过90,考虑插补 **********************************
                    # 初始rz = -25, 逆时针旋转，rz增加，不会超出180上限
                    delta_angle += 180 - domino_info['angle'] + 90 # 逆时针旋转到状态3
                    self.set_pose_move([place_x, place_y, place_z], delta_angle)

        print('place_info: ', place_x, place_y, place_z, delta_angle)

    def pick_place_once(self, domino_info):
        """
            抓取并放置一张骨牌
        """
         # 由骨牌像素坐标得到空间坐标  坐标转换待更新
        position = self.position_trans(domino_info['pixel_pstn'], hand_camera=1)
        print("robot-coordinate 2D position: ", position[0], position[1])

        # self.set_pose_move(position, 0)
        self.set_pick_position(position)
        domino_info = self.servo(domino_info)
        # print('domino_info: ', domino_info)
        self.pick(HEIGHT['before'] - HEIGHT['pick'])
        # print("move to the mid_point...")# 运动到中间点
        # self.set_pose_move([MID_POINT['X'], MID_POINT['Y'], MID_POINT['Z']], 0)
        self.set_place_position(domino_info)# 根据待放置的center与state决定具体放置坐标
        # # self.place(INIT_STATE['Z0'] - HEIGHT['place'])
        self.gripper_open()
        # self.init_move()

    def pick_place_cycle(self):
        print("Start cycle...")

        solution = []
        request = SolutionRequest()
        request.signal = 'solution'
        response = self.solution_client(request)

        if response.result == 'successful':
            for place_info in response.solution:
                solution.append([place_info.shape, place_info.state, place_info.center])
        print('solution: ', solution)

        i = 0
        obj_num = None
        # 先做全局规划再依次抓取
        # for i in range(35):
        while True:
            if i == len(solution):
                break

            objects = self.get_next_domino()
            while objects is None:
                objects = self.get_next_domino()
            if len(objects) == obj_num:
                # 上次抓取失败，重新抓取
                i -= 1
            else:
                # 上次抓取成功，更新数量
                obj_num = len(objects)

            delta_angle = []
            current_domino = None
            print('target: ', solution[i])
            for item in objects:
                if item.type == solution[i][0]:
                    # print(item.type)
                    # 选择角度变化最小的方块进行抓取
                    if item.type == 'O': # 角度变化为锐角，无需进行选择
                        current_domino = item
                        break
                    elif item.type == 'I' or item.type == 'S' or item.type == 'Z':
                        if solution[i][1] == 0: # 横摆放，角度变化可能是钝角
                            delta_angle.append(item.theta)
                            # print(item.theta)
                        else: # 竖摆放，角度变化为锐角，无需进行选择
                            current_domino = item
                            break
                    elif item.type == 'J' or item.type == 'L' or item.type == 'T':
                        # print(item.theta)
                        if solution[i][1] == 0: # 0度摆放
                            delta_angle.append(abs(item.theta))
                        elif solution[i][1] == 1: # 90度摆放
                            if item.theta >= 0 and item.theta <= 180:
                                delta_angle.append(abs(item.theta - 90))
                            elif item.theta >= -180 and item.theta <= -90:
                                delta_angle.append(180 + item.theta + 90)
                            else:
                                delta_angle.append(abs(item.theta) + 90)
                        elif solution[i][1] == 2: # -180度摆放
                            delta_angle.append(180 - abs(item.theta))
                        elif solution[i][1] == 3: # -90度放置
                            if item.theta >= -180 and item.theta <= 0:
                                delta_angle.append(abs(abs(item.theta) - 90))
                            elif item.theta > 0 and item.theta <= 90:
                                delta_angle.append(90 + item.theta)
                            else:
                                delta_angle.append(180 - item.theta + 90)

            # print(delta_angle)
            if current_domino is None:
                k = 0
                for item in objects:
                    if item.type == solution[i][0]:
                        if k == np.argmin(delta_angle):
                            current_domino = item
                            break
                        k += 1
            domino_info = {'shape':current_domino.type, 'pixel_pstn':current_domino.center, 'angle':current_domino.theta,
                            'place_center':solution[i][2], 'place_state':solution[i][1]}
            print('domino_info: ',domino_info)
            self.pick_place_once(domino_info)
            i += 1

        self.init_move()
        print('cycle end...')
            
    def test(self):
        solution = []
        request = SolutionRequest()
        request.signal = 'solution'
        response = self.solution_client(request)

        if response.result == 'successful':
            for place_info in response.solution:
                solution.append([place_info.shape, place_info.state, place_info.center])
        print('solution: ', solution)
        # return
        
        i = 6
        objects = self.get_next_domino()
        while objects is None:
            objects = self.get_next_domino()

        delta_angle = []
        current_domino = None
        print('target: ', solution[i])
        for item in objects:
            if item.type == solution[i][0]:
                # print(item.type)
                # 选择角度变化最小的方块进行抓取
                if item.type == 'O': # 角度变化为锐角，无需进行选择
                    current_domino = item
                    break
                elif item.type == 'I' or item.type == 'S' or item.type == 'Z':
                    if solution[i][1] == 0: # 横摆放，角度变化可能是钝角
                        delta_angle.append(item.theta)
                        # print(item.theta)
                    else: # 竖摆放，角度变化为锐角，无需进行选择
                        current_domino = item
                        break
                elif item.type == 'J' or item.type == 'L' or item.type == 'T':
                    # print(item.theta)
                    if solution[i][1] == 0: # 0度摆放
                        delta_angle.append(abs(item.theta))
                    elif solution[i][1] == 1: # 90度摆放
                        if item.theta >= 0 and item.theta <= 180:
                            delta_angle.append(abs(item.theta - 90))
                        elif item.theta >= -180 and item.theta <= -90:
                            delta_angle.append(180 + item.theta + 90)
                        else:
                            delta_angle.append(abs(item.theta) + 90)
                    elif solution[i][1] == 2: # -180度摆放
                        delta_angle.append(180 - abs(item.theta))
                    elif solution[i][1] == 3: # -90度放置
                        if item.theta >= -180 and item.theta <= 0:
                            delta_angle.append(abs(abs(item.theta) - 90))
                        elif item.theta > 0 and item.theta <= 90:
                            delta_angle.append(90 + item.theta)
                        else:
                            delta_angle.append(180 - item.theta + 90)

        # print(delta_angle)
        if current_domino is None:
            k = 0
            for item in objects:
                if item.type == solution[i][0]:
                    if k == np.argmin(delta_angle):
                        current_domino = item
                        break
                    k += 1
        domino_info = {'shape':current_domino.type, 'pixel_pstn':current_domino.center, 'angle':current_domino.theta,
                        'place_center':solution[i][2], 'place_state':solution[i][1]}
        print('domino_info: ',domino_info)
        self.pick_place_once(domino_info)

    def test_robot(self):
        # self.init_move()
        # self.set_pose_move([PLACE_ORIGIN['X'], PLACE_ORIGIN['Y'], PLACE_ORIGIN['Z']], 0)
        # self.pick(HEIGHT['before'] - HEIGHT['pick'])
        # self.init_move()
        # self.place(30)
        # self.z_move(-50)

        # self.robot_cmd.jog_coord('x', -1, 2000)
        # ros.sleep(2)
        # self.robot_cmd.jog_coord('x', 1, 2000)
        # ros.sleep(2)
        # self.robot_cmd.jog_coord('x', 0, 0)
        # ros.sleep(1)

        pose_cur = self.robot_cmd.get_coords()
        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], -30) # 移动到待放置位置
        ros.sleep(3)
        self.set_pose_move([pose_cur[0], pose_cur[1], pose_cur[2]], 30) # 移动到待放置位置
        ros.sleep(2)
        self.init_move()

if __name__ == '__main__':

    my_robot = MyRobot()
    # my_robot.test_robot()

    # my_robot.test()
    # my_robot.pick_place_cycle()
