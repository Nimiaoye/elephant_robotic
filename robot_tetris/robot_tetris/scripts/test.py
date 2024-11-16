# -*- coding: utf-8 -*-
import os
import sys
import rospy
import numpy as np
import cv2 as cv
import math
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import calibrate_extrinsic as ce

IMAGE_SRC_DIR = '/home/zyh/ros_workspace/src/robot_tetris/data/image_src_1'
IMAGE_CREATE_DIR = '/home/zyh/ros_workspace/src/robot_tetris/data/image_create'
IMAGE_MASK_DIR = '/home/zyh/ros_workspace/src/robot_tetris/data/image_mask'
BACKGROUND = '/home/zyh/ros_workspace/src/robot_tetris/data/white.jpg'

hsv_th = {'blue'   : [[97,125,35], [125,255,255]], 
          'green'  : [[60,170,46], [86,255,255]],  
          'red'    : [[[0,160,68], [10,255,255]], [[160,160,68], [180,255,255]]],
          'yellow' : [[26,43,46], [34,255,255]], # RGB(255,255,0)
          'orange' : [[11,43,46], [25,255,255]], # RGB(255,165,0)
          'purple' : [[125,43,46], [155,255,255]], # RGB(128,0,128)
          'brown'  : [[0,43,46], [25,255,255]]} # RGB(165,42,42)  hsv值待确定

BGR = { 'red'   : [0,0,255],
        'blue'  : [255,0,0],
        'green' : [0,255,0],
        'yellow': [0,255,255],
        'orange': [0,165,255],
        'purple': [128,0,128],
        'brown' : [42,42,165]}

rgb_th = {'brown' : [[30,30,150], [50,50,170]]}


def generate_image():
    IMAGE_SRC_DIR = r'D:\Code_exercise\Robot_Contest\data\image_src_1'
    GRID_WIDTH = 40

    image_src = np.zeros((480,640,3),dtype=np.uint8) + 255
    w, h = image_src.shape[1], image_src.shape[0]

    image_I = image_src.copy()
    image_I[h//2 - GRID_WIDTH//2:h//2 + GRID_WIDTH//2, w//2 - GRID_WIDTH*2:w//2 + GRID_WIDTH*2] = BGR['red']

    image_J = image_src.copy()
    image_J[h//2 - GRID_WIDTH//2 * 3:h//2 + GRID_WIDTH//2, w//2 - GRID_WIDTH//2 * 3:w//2 - GRID_WIDTH//2] = BGR['purple']
    image_J[h//2 - GRID_WIDTH//2:h//2 + GRID_WIDTH//2, w//2 - GRID_WIDTH//2:w//2 + GRID_WIDTH//2 * 3] = BGR['purple']

    image_L = image_src.copy()
    image_L[h//2 - GRID_WIDTH//2 * 3:h//2 + GRID_WIDTH//2, w//2 + GRID_WIDTH//2:w//2 + GRID_WIDTH//2 * 3] = BGR['yellow']
    image_L[h//2 - GRID_WIDTH//2:h//2 + GRID_WIDTH//2, w//2 - GRID_WIDTH//2 * 3:w//2 + GRID_WIDTH//2] = BGR['yellow']

    image_O = image_src.copy()
    image_O[h//2 - GRID_WIDTH:h//2 + GRID_WIDTH, w//2 - GRID_WIDTH:w//2 + GRID_WIDTH] = BGR['orange']

    image_S = image_src.copy()
    image_S[h//2:h//2 + GRID_WIDTH, w//2 - GRID_WIDTH//2 * 3:w//2 + GRID_WIDTH//2] = BGR['green']
    image_S[h//2 - GRID_WIDTH:h//2, w//2 - GRID_WIDTH//2:w//2 + GRID_WIDTH//2 * 3] = BGR['green']

    image_T = image_src.copy()
    image_T[h//2 - GRID_WIDTH//2:h//2 + GRID_WIDTH//2, w//2 - GRID_WIDTH//2*3:w//2 + GRID_WIDTH//2*3] = BGR['brown']
    image_T[h//2 - GRID_WIDTH//2*3:h//2-GRID_WIDTH//2, w//2 - GRID_WIDTH//2:w//2 + GRID_WIDTH//2] = BGR['brown']

    image_Z = image_src.copy()
    image_Z[h//2 - GRID_WIDTH:h//2, w//2 - GRID_WIDTH//2*3:w//2 + GRID_WIDTH//2] = BGR['blue']
    image_Z[h//2:h//2 + GRID_WIDTH, w//2 - GRID_WIDTH//2:w//2 + GRID_WIDTH//2*3] = BGR['blue']
    
    cv.imwrite(IMAGE_SRC_DIR + r'\I.jpg', image_I)
    cv.imwrite(IMAGE_SRC_DIR + r'\J.jpg', image_J)
    cv.imwrite(IMAGE_SRC_DIR + r'\L.jpg', image_L)
    cv.imwrite(IMAGE_SRC_DIR + r'\O.jpg', image_O)
    cv.imwrite(IMAGE_SRC_DIR + r'\S.jpg', image_S)
    cv.imwrite(IMAGE_SRC_DIR + r'\T.jpg', image_T)
    cv.imwrite(IMAGE_SRC_DIR + r'\Z.jpg', image_Z)

TRACk_BAR_FLAG = False

def hsv_process(image_src, DISPLAY=True, TEST=False):
    global TRACk_BAR_FLAG
    image = image_src.copy()
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (7,7))

    th_blue = np.array(hsv_th['blue'])
    th_green = np.array(hsv_th['green'])
    th_red1 = np.array(hsv_th['red'][0])
    th_red2 = np.array(hsv_th['red'][1])
    th_yellow = np.array(hsv_th['yellow'])
    th_orange = np.array(hsv_th['orange'])
    th_purple = np.array(hsv_th['purple'])
    th_brown = np.array(hsv_th['brown'])

    # rgb_th_brown = np.array(rgb_th['brown'])

    hsv_mask_blue = cv.inRange(hsv_image, th_blue[0], th_blue[1])
    hsv_mask_green = cv.inRange(hsv_image, th_green[0], th_green[1])
    hsv_mask_red1 = cv.inRange(hsv_image, th_red1[0], th_red1[1])
    hsv_mask_red2 = cv.inRange(hsv_image, th_red2[0], th_red2[1])
    hsv_mask_red = hsv_mask_red1 | hsv_mask_red2
    hsv_mask_yellow = cv.inRange(hsv_image, th_yellow[0], th_yellow[1])
    hsv_mask_orange = cv.inRange(hsv_image, th_orange[0], th_orange[1])
    hsv_mask_purple = cv.inRange(hsv_image, th_purple[0], th_purple[1])
    hsv_mask_brown = cv.inRange(hsv_image, th_brown[0], th_brown[1])

    # rgb_mask_brown = cv.inRange(image, rgb_th_brown[0], rgb_th_brown[1])

    # cv.imshow('rgb_mask_brown', rgb_mask_brown)
    # cv.waitKey(0)

    masks = {'O': hsv_mask_orange, 'I': hsv_mask_red, 'J': hsv_mask_purple, 'L': hsv_mask_yellow,
            'S': hsv_mask_green, 'Z': hsv_mask_blue, 'T': hsv_mask_brown}

    segmentation_mask = hsv_mask_blue | hsv_mask_green | hsv_mask_red | hsv_mask_orange | \
                        hsv_mask_yellow | hsv_mask_purple | hsv_mask_brown 
    segmentation_mask = cv.morphologyEx(segmentation_mask, cv.MORPH_OPEN, kernel) # 开运算

    segmentation_result = cv.bitwise_and(image, image, mask=segmentation_mask)

    if DISPLAY:
        cv.imshow("segmentation_result", segmentation_result)

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

def detect_hsv(image):
    # IMAGE_SRC_NAMES = os.listdir(IMAGE_SRC_DIR)
    # IMAGE_SRC_PATH = os.path.join(IMAGE_SRC_DIR, IMAGE_SRC_NAMES[IMAGE_SRC_NAMES.index('L.jpg')])
    IMAGE_CREATE_NAMES = os.listdir(IMAGE_CREATE_DIR)
    IMAGE_CREATE_PATH = os.path.join(IMAGE_CREATE_DIR, IMAGE_CREATE_NAMES[IMAGE_CREATE_NAMES.index('image_T_11.jpg')])

    # image_src = cv.imread(IMAGE_SRC_PATH)
    image_create = cv.imread(IMAGE_CREATE_PATH)
    # image_bgr = image_src.copy()
    image_bgr = image_create.copy()
    # image_bgr = image.copy()

    # cv.imshow('image_src', image_bgr)
    # cv.waitKey(0)

    SHAPE = ['O', 'I', 'J', 'L', 'S', 'Z'] # T形状包含在I情况下
    segmentation_mask, masks = hsv_process(image_bgr, DISPLAY=False, TEST=False)

    objects = []

    for shape in SHAPE:
        mask = masks[shape]
        # cv.imshow('mask_{}'.format(shape), mask)
        # cv.waitKey(0)

        contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]

        if len(contours) > 0:
            for contour in contours:
                object_info = {'shape':shape, 'center':None, 'angle':None}
                if cv.arcLength(contour, True) < 20 or cv.arcLength(contour, True) > 2000:
                    continue
                epsilon = 0.03 * cv.arcLength(contour, True)
                vertex = np.squeeze(cv.approxPolyDP(contour, epsilon, True)) # shape:(n,2) n个顶点

                min_rect = cv.minAreaRect(contour)
                min_rect_points = np.array(cv.boxPoints(min_rect), dtype=np.float)

                x_list = [vertex[i][0] for i in range(len(vertex))]
                x_min_pos, x_max_pos = np.argmin(x_list), np.argmax(x_list)
                y_list = [vertex[i][1] for i in range(len(vertex))]
                y_min_pos, y_max_pos = np.argmin(y_list), np.argmax(y_list)

                side = [] # 轮廓的各边长
                for i in range(len(vertex)):
                    side.append(np.sqrt((vertex[i][0]-vertex[(i+1) % len(vertex)][0]) ** 2 + 
                                        (vertex[i][1]-vertex[(i+1) % len(vertex)][1]) ** 2))
                side_max_pos, side_min_pos = np.argmax(side), np.argmin(side)

                # print('vertex: ', vertex)
                # print('min_rect: ', min_rect_points)
                # print('side', side)

                if shape == 'O':
                    object_info['angle'] = -min_rect[2]
                    center_x = int(vertex[x_min_pos][0] + (vertex[(x_min_pos+2) % 4][0] - vertex[x_min_pos][0]) / 4)
                    center_y = int(vertex[x_min_pos][1] + (vertex[(x_min_pos+2) %4 ][1] - vertex[x_min_pos][1]) / 4)
                    object_info['center'] = [center_x, center_y]

                elif shape == 'I':
                    if len(vertex) == 4:
                        object_info['shape'] = 'I'
                        if (abs(y_list[0]-y_list[1]) < 3 and (side[0] - side[1]) > 3) or (abs(y_list[0]-y_list[3]) < 3 and (side[0] - side[1]) < -3):
                            # angle = 0  中心点位于右二格
                            object_info['angle'] = 0.0
                            center_y = int(np.mean(y_list))
                            center_x = int(vertex[1][0] + (vertex[0][0]-vertex[1][0]) * 5 / 8) if abs(y_list[0]-y_list[1]) < 3 else \
                                    int(vertex[0][0] + (vertex[3][0]-vertex[0][0]) * 5 / 8)
                            object_info['center'] = [center_x, center_y]
                        elif (abs(y_list[0]-y_list[1]) < 3 and (side[0] - side[1]) < -3) or (abs(y_list[0]-y_list[3]) < 3 and (side[0] - side[1]) > 3):
                            # angle = 90  中心点位于上二格
                            object_info['angle'] = 90.0
                            center_x = int(np.mean(x_list))
                            center_y = int(vertex[1][1] + (vertex[2][1]-vertex[1][1]) * 3 / 8) if abs(y_list[0]-y_list[1]) < 3 else \
                                    int(vertex[0][1] + (vertex[1][1]-vertex[0][1]) * 3 / 8)
                            object_info['center'] = [center_x, center_y]
                        else:
                            # 中心点位于偏上二格
                            if abs(min_rect[1][0] - side[side_max_pos]) < 5:
                                # angle < 90
                                object_info['angle'] = -min_rect[2]
                                # 找到上下两条短边的中点
                                point1 = [(vertex[0][0] + vertex[3][0]) / 2, (vertex[0][1] + vertex[3][1]) / 2]
                                point2 = [(vertex[1][0] + vertex[2][0]) / 2, (vertex[1][1] + vertex[2][1]) / 2]
                                center_x = int(point1[0] + (point2[0] - point1[0]) * 3 / 8)
                                center_y = int(point1[1] + (point2[1] - point1[1]) * 3 / 8)
                                object_info['center'] = [center_x, center_y]
                            else:
                                # angle > 90
                                object_info['angle'] = 90 - min_rect[2]
                                # 找到上下两条短边的中点
                                point1 = [(vertex[0][0] + vertex[1][0]) / 2, (vertex[0][1] + vertex[1][1]) / 2]
                                point2 = [(vertex[2][0] + vertex[3][0]) / 2, (vertex[2][1] + vertex[3][1]) / 2]
                                center_x = int(point1[0] + (point2[0] - point1[0]) * 3 / 8)
                                center_y = int(point1[1] + (point2[1] - point1[1]) * 3 / 8)
                                object_info['center'] = [center_x, center_y]

                    else:
                        object_info['shape'] = 'T'
                        # 找到与最长边相邻的两条短边的中点
                        point1 = [(vertex[side_max_pos][0] + vertex[(side_max_pos - 1) % 8][0]) / 2,
                                (vertex[side_max_pos][1] + vertex[(side_max_pos - 1) % 8][1]) / 2]
                        point2 = [(vertex[(side_max_pos + 1) % 8][0] + vertex[(side_max_pos + 2) % 8][0]) / 2,
                                (vertex[(side_max_pos + 1) % 8][1] + vertex[(side_max_pos + 2) % 8][1]) / 2]
                        object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                        # 根据最小包围矩形的顶点与物体顶点的对应情况  判断位于哪一象限
                        if sum(abs(min_rect_points[0] - vertex[side_max_pos])) < 10 and \
                            sum(abs(min_rect_points[3] - vertex[(side_max_pos + 1) % 8])) < 10:
                            # 长边位于第一象限
                            object_info['angle'] = -min_rect[2]
                        elif sum(abs(min_rect_points[3] - vertex[side_max_pos])) <10 and \
                            sum(abs(min_rect_points[2] - vertex[(side_max_pos + 1) % 8])) < 10:
                            # 长边位于第二象限
                            object_info['angle'] = -min_rect[2] + 90
                            if object_info['angle'] == 180:
                                object_info['angle'] = -180
                        elif sum(abs(min_rect_points[2] - vertex[side_max_pos])) < 10 and \
                            sum(abs(min_rect_points[1] - vertex[(side_max_pos + 1) % 8])) < 10:
                            # 长边位于第三象限
                            object_info['angle'] = -min_rect[2] - 180
                        elif sum(abs(min_rect_points[1] - vertex[side_max_pos])) < 10 and \
                            sum(abs(min_rect_points[0] - vertex[(side_max_pos + 1) % 8])) < 10:
                            object_info['angle'] = (-min_rect[2]) - 90

                elif shape == 'J':
                    point1 = [vertex[side_max_pos][0], vertex[side_max_pos][1]] # 外拐顶点
                    point2 = [vertex[(side_max_pos + 3) % 6][0], vertex[(side_max_pos + 3) % 6][1]] # 内拐顶点
                    object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                    # 以外拐顶点为原点  按照长边所在象限进行分类  角度范围[-180,180)
                    if side_max_pos == (y_max_pos + 1) % 6 and side_max_pos == (x_min_pos + 2) % 6:
                        # 90°特殊情况
                        object_info['angle'] = 90.0
                    elif side_max_pos == y_max_pos:
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
                    if min_rect[1][0] - side[side_max_pos] > 20:
                        # 最小矩形的w大于长边  第一象限
                        object_info['angle'] = -min_rect[2]
                        # 找到矩形两条短边的偏上四等分点
                        point1 = [min_rect_points[1][0] + (min_rect_points[0][0] - min_rect_points[1][0]) / 4,
                                    min_rect_points[1][1] + (min_rect_points[0][1] - min_rect_points[1][1]) / 4]
                        point2 = [min_rect_points[2][0] + (min_rect_points[3][0] - min_rect_points[2][0]) / 4,
                                    min_rect_points[2][1] + (min_rect_points[3][1] - min_rect_points[2][1]) / 4]
                        object_info['center'] = [int((point1[0] + point2[0]) / 2),
                                                    int((point1[1] + point2[1]) / 2)]
                    else:
                        # 最小矩形的w等于长边  第二象限
                        object_info['angle'] = -min_rect[2] + 90
                        # 找到矩形两条短边的偏下四等分点
                        point1 = [min_rect_points[0][0] + (min_rect_points[3][0] - min_rect_points[0][0]) / 4,
                                    min_rect_points[0][1] + (min_rect_points[3][1] - min_rect_points[3][1]) / 4]
                        point2 = [min_rect_points[1][0] + (min_rect_points[2][0] - min_rect_points[1][0]) / 4,
                                    min_rect_points[1][1] + (min_rect_points[2][1] - min_rect_points[1][1]) / 4]
                        object_info['center'] = [int((point1[0] + point2[0]) / 2),
                                                    int((point1[1] + point2[1]) / 2)]

                        # 0°情况特殊
                        if object_info['angle'] == 180:
                            object_info['angle'] = 0
                            point1 = [min_rect_points[2][0] + (min_rect_points[1][0] - min_rect_points[2][0]) / 4,
                                        min_rect_points[2][1] + (min_rect_points[1][1] - min_rect_points[2][1]) / 4]
                            point2 = [min_rect_points[3][0] + (min_rect_points[0][0] - min_rect_points[3][0]) / 4,
                                        min_rect_points[3][1] + (min_rect_points[0][1] - min_rect_points[3][1]) / 4]
                            object_info['center'] = [int((point1[0] + point2[0]) / 2),
                                                        int((point1[1] + point2[1]) / 2)]

                print(object_info)

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

    plt.figure(figsize=(10,8))
    plt.imshow(cv.cvtColor(image_bgr, cv.COLOR_BGR2RGB))
    plt.show()

    return image_bgr, objects

def calibrate_test():
    intrinsic_path = '/home/zyh/ros_workspace/src/robot_tetris/data/json/realsense.json'
    extrinsic_path = '/home/zyh/ros_workspace/src/robot_tetris/data/json/extrinsic_realsense_1.json'
    # ce.calibrate_extrinsic(1, intrinsic_path, extrinsic_path)

    intrinsic = np.array([[618.674, 0, 329.289],
                          [0, 615.679, 236.361],
                          [0,       0,       1]], dtype=np.float)
    intrinsic_1 = np.array([[618.674, 0, 329.289, 0],
                            [0, 615.679, 236.361, 0],
                            [0, 0, 1, 0]], dtype=np.float)
    T_camera_2_robot = np.array([[ 0.02947733, -0.99694323,  0.07235522, 0.36761649],
                                 [-0.98623415, -0.01722501,  0.16445518, -0.03809442],
                                 [-0.16270616, -0.07620689, -0.98372721, 0.67137597],
                                 [          0,           0,           0,          1]], dtype=np.float)
    R_camera_2_robot = np.array([[-0.05462911, -0.99847776, -0.00760379],
                                 [-0.98745868,  0.05289342,  0.14875364],
                                 [-0.14812501,  0.01563471, -0.98884505]], dtype=np.float)
    t_camera_2_robot = np.array([[0.48342977],
                                 [-0.07321325],
                                 [0.63483207]], dtype=np.float)

    
    
    pos_pixel = np.array([[314], [242], [1]], dtype=np.float)
    pos_camera = np.dot(np.linalg.inv(intrinsic), pos_pixel) * 0.63483207
    pos_robot = np.dot(np.linalg.inv(R_camera_2_robot.T), (pos_camera - t_camera_2_robot))
    print(pos_camera)
    X1, Y1 = pos_robot[0][0] * 1000 + 4, pos_robot[1][0] * 1000 - 4
    print('X1: ', X1)
    print('Y1: ', Y1)

    # rospy.init_node("test")
    # rospy.loginfo("Starting test")
    # while True:
    #     rgb = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=None)
    #     rgb = np.ndarray(shape=(rgb.height, rgb.width, 3), dtype=np.uint8, buffer=rgb.data)
    #     cv.imshow('image', rgb)
    #     cv.waitKey(5)

    # pos_camera = np.array([pos_camera[0], pos_camera[1], pos_camera[2], [1]], dtype=np.float)
    # print(pos_camera.shape)
    # pos_robot = np.dot(T_camera_2_robot, pos_camera)
    # pos_robot = np.dot(np.linalg.inv(T_camera_2_robot), pos_camera)
    # print(pos_robot)

    # pos_robot = np.array([[0.36], [0], [0], [1]], dtype=np.float)
    # pos_pixel = np.dot(np.dot(intrinsic, T_camera_2_robot), pos_robot)
    # print(np.dot(intrinsic, np.dot(np.linalg.inv(T_camera_2_robot), pos_robot)))
    


if __name__ == '__main__':
    # generate_image()
    # detect_hsv(None)
    calibrate_test()
