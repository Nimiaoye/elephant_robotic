#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    第四届中国高校智能机器人创意大赛
    主题二————俄罗斯方块
    图像处理——目标检测

    最后修改：2021.7.7
"""
import os
import math
import rospy as ros
import numpy as np
import cv2 as cv
from PIL import Image
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_tetris.msg import ProcessResult
from robot_tetris.srv import ImgProcess, ImgProcessResponse

"""
    启动 realsense:
        cd ~/elephant_ws
        source devel/setup.bash
        roslaunch realsense2_camera rs_camera.launch  align_depth:=true

    启动电脑自带相机或usb相机：
        roslaunch pick_place usb_cam.launch
        realsense /dev/video6

    启动多个realsense：
        rs-enumerate-devices | grep Serial  查看相机端口编号
        roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=912112070121 serial_no_camera2:=011422072050
"""

IMAGE_SRC_DIR = '/home/zyh/ros_workspace/src/robot_tetris/data/image_src_1'
IMAGE_CREATE_DIR = '/home/zyh/ros_workspace/src/robot_tetris/data/image_create'
IMAGE_MASK_DIR = '/home/zyh/ros_workspace/src/robot_tetris/data/image_mask'
BACKGROUND = '/home/zyh/ros_workspace/src/robot_tetris/data/white.jpg'

# HSV threshold
hsv_th = {'blue'   : [[97,95,35], [125,255,255]], 
          'green'  : [[60,68,41], [86,255,255]],  
          'red'    : [[[0,120,100], [10,255,255]], [[160,120,100], [180,255,255]]],
          'yellow' : [[21,130,113], [34,255,255]], # RGB(255,255,0)  保留
          'purple' : [[123,42,35], [158,255,255]],
          'brown'  : [[3,74,48], [18,195,100]]} # RGB(128,0,128)

TRACk_BAR_FLAG = False

def hsv_process(image_src, DISPLAY=True, TEST=False):
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
    # cv.waitKey(5)

    if DISPLAY:
        cv.imshow("segmentation_result", segmentation_result)
        # cv.waitKey(3)

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
        # cv.waitKey(3)
    
    return segmentation_mask, masks

def onTrackbar(x):
    pass

TRACk_BAR_FLAG_1 = False
def canny_th(image):
    global TRACk_BAR_FLAG_1
    if TRACk_BAR_FLAG_1 is False:
        TRACk_BAR_FLAG_1 = True
        cv.namedWindow("canny_test")
        cv.createTrackbar("th_min", "canny_test", 30, 300, onTrackbar)
        cv.createTrackbar("th_max", "canny_test", 150, 300, onTrackbar)

    th_min = cv.getTrackbarPos("th_min", "canny_test")
    th_max = cv.getTrackbarPos("th_max", "canny_test")

    edge = cv.Canny(image, th_min, th_max)
    kernel = np.ones((5,5), dtype=np.uint8)
    edge = cv.morphologyEx(edge, cv.MORPH_CLOSE, kernel)

    cv.imshow("canny_test", edge)
    cv.waitKey(3)

def get_bound(mask):  # 获取掩码的boundingbox
    contours = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[1]
    # print('num of cnts', len(contours))
    contour = contours[0]
    if len(contours) > 1:  # 如果轮廓数目大于1， 计算最大轮廓
        max_area = 0
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > max_area:
                max_area = area
                contour = cnt

    x, y, w, h = cv.boundingRect(contour)
    return x, y, w, h

def mask_create():
    src_imgs = [os.path.join(IMAGE_SRC_DIR, item) for item in os.listdir(IMAGE_SRC_DIR)]

    for img_path in src_imgs:
        src_img = cv.imread(img_path)
        mask = hsv_process(src_img)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv.erode(mask, kernel, iterations=1)
        cv.imwrite(IMAGE_MASK_DIR + r'\mask_' + os.path.split(img_path)[1], mask)
        # print(os.path.split(img_path))

        # cv.imshow('mask', mask)
        # cv.waitKey(0)

def data_create():
    """生成单一形状的数据"""
    SHAPES = ['I','J','L','O','S','T','Z']
    IMAGE_SRC_PATHS = [os.path.join(IMAGE_SRC_DIR, item) for item in os.listdir(IMAGE_SRC_DIR)]
    IMAGE_MASK_PATHS = [os.path.join(IMAGE_MASK_DIR, item) for item in os.listdir(IMAGE_MASK_DIR)]

    NUM = 12 # 每种类型物体生成 NUM 张图片

    Image_Size = [480, 640]
    center = [x // 2 for x in Image_Size]
    DELTA_Y = 1 # 数据生成时目标位置的变化范围
    DELTA_X = 1
    
    for i in range(len(IMAGE_SRC_PATHS)):
        src_image = Image.open(IMAGE_SRC_PATHS[i])
        mask_image = Image.open(IMAGE_MASK_PATHS[i])

        for j in range(NUM):
            background = Image.open(BACKGROUND)
            pos_y = np.random.randint(center[0] - DELTA_Y, center[0] + DELTA_Y)
            pos_x = np.random.randint(center[1] - DELTA_X, center[1] + DELTA_X)
            # angle = np.random.randint(-90, 90)
            angle = 30 * j

            src_image_rotate = src_image.rotate(angle)
            mask_image_rotate = mask_image.rotate(angle)
            mask_image_np = np.array(mask_image_rotate, dtype=np.uint8)

            x, y, w, h = get_bound(mask_image_np)
            src_image_roi = src_image_rotate.crop((x, y, x+w, y+h))
            mask_image_roi = mask_image_rotate.crop((x, y, x+w, y+h))

            mask_generate = Image.new('L', mask_image.size)
            mask_generate.paste(mask_image_roi, (pos_x - w//2, pos_y - h//2), mask=mask_image_roi)
            mask_generate = np.array(mask_generate, dtype=np.uint8)

            background.paste(src_image_roi, (pos_x - w//2, pos_y - h//2), mask=mask_image_roi)
            img_generate = cv.cvtColor(np.array(background, dtype=np.uint8), cv.COLOR_BGR2RGB)

            cv.imwrite((IMAGE_CREATE_DIR + '\\' + 'image_{}_{}.jpg'.format(SHAPES[i], j)), img_generate)
            # cv.imwrite((Label_Create_Path + '\\' + 'label_{}_{}_{}.jpg'.format(i, j, idx)), mask_generate)

            # cv.imshow('image', img_generate)
            # cv.imshow('mask', mask_generate)
            # cv.waitKey(0)

def data_create_1():
    """生成多种形状的数据"""
    SHAPES = ['I','J','L','O','S','T','Z']
    IMAGE_SRC_PATHS = [os.path.join(IMAGE_SRC_DIR, item) for item in os.listdir(IMAGE_SRC_DIR)]
    IMAGE_MASK_PATHS = [os.path.join(IMAGE_MASK_DIR, item) for item in os.listdir(IMAGE_MASK_DIR)]

    NUM = 10 # 总共生成的图片数量
    SHAPE_MAX_NUM = 8 # 生成的图像中最多有几种形状

    Image_Size = [480, 640]
    center = [x // 2 for x in Image_Size]
    DELTA_Y = 190 # 数据生成时目标位置的变化范围
    DELTA_X = 270
    
    for i in range(NUM):
        shape_num = np.random.randint(1, SHAPE_MAX_NUM + 1)
        # print(shape_num)
        background = Image.open(BACKGROUND)
        rand_xy = []

        for j in range(shape_num):
            shape_index = np.random.randint(0, len(IMAGE_SRC_PATHS))
            src_image = Image.open(IMAGE_SRC_PATHS[shape_index])
            mask_image = Image.open(IMAGE_MASK_PATHS[shape_index % 7])

            while True:
                rand_x = np.random.randint(0,5)
                rand_y = np.random.randint(0,4)
                if [rand_x, rand_y] not in rand_xy:
                    rand_xy.append([rand_x, rand_y])
                    break
                else:
                    continue
            pos_x = 70 + 120 * rand_x
            pos_y = 80 + 105 * rand_y
            # pos_y = np.random.randint(center[0] - DELTA_Y, center[0] + DELTA_Y)
            # pos_x = np.random.randint(center[1] - DELTA_X, center[1] + DELTA_X)
            rand_angle = np.random.randint(0, 18)
            angle = 20 * rand_angle

            src_image_rotate = src_image.rotate(angle)
            mask_image_rotate = mask_image.rotate(angle)
            mask_image_np = np.array(mask_image_rotate, dtype=np.uint8)

            x, y, w, h = get_bound(mask_image_np)
            src_image_roi = src_image_rotate.crop((x, y, x+w, y+h))
            mask_image_roi = mask_image_rotate.crop((x, y, x+w, y+h))

            mask_generate = Image.new('L', mask_image.size)
            mask_generate.paste(mask_image_roi, (pos_x - w//2, pos_y - h//2), mask=mask_image_roi)
            mask_generate = np.array(mask_generate, dtype=np.uint8)

            background.paste(src_image_roi, (pos_x - w//2, pos_y - h//2), mask=mask_image_roi)
            img_generate = cv.cvtColor(np.array(background, dtype=np.uint8), cv.COLOR_BGR2RGB)

            # cv.imwrite((IMAGE_CREATE_DIR + '\\' + 'image_{}.jpg'.format(i)), img_generate)
            # cv.imwrite((Label_Create_Path + '\\' + 'label_{}_{}_{}.jpg'.format(i, j, idx)), mask_generate)

        cv.imwrite((IMAGE_CREATE_DIR + '\\' + 'image_{}.jpg'.format(i)), img_generate)
        # cv.imshow('image', img_generate)
        # cv.imshow('mask', mask_generate)
        # cv.waitKey(0)

class ImageProcess():
    
    # IMAGE_SRC_NAMES = os.listdir(IMAGE_SRC_DIR)
    # IMAGE_SRC_PATH = os.path.join(IMAGE_SRC_DIR, IMAGE_SRC_NAMES[IMAGE_SRC_NAMES.index('I.jpg')])
    IMAGE_CREATE_NAMES = os.listdir(IMAGE_CREATE_DIR)
    IMAGE_CREATE_PATH = os.path.join(IMAGE_CREATE_DIR, IMAGE_CREATE_NAMES[IMAGE_CREATE_NAMES.index('image_Z_2.jpg')])

    def __init__(self):
        ros.init_node('ImageProcess_node')
        ros.loginfo('Start ImageProcess node...')

        self.cv_bridge = CvBridge()
        self.detect_result = ImgProcessResponse()
        # self.image_sub = ros.Subscriber('/camera/color/image_raw', Image, self.image_sub_callback)
        # self.image_sub = ros.Subscriber('/usb_cam/image_raw', Image, self.image_sub_callback)
        self.image_sub = ros.Subscriber('/camera1/color/image_raw', Image, self.image_sub_callback)
        # self.image_sub = ros.Subscriber('/camera2/color/image_raw', Image, self.image_sub_callback)
        self.image_srv = ros.Service('/object_detection', ImgProcess, self.image_srv_callback)

    def image_sub_callback(self, data):
        try:
            self.image_src = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        image_src = self.image_src.copy()
        mask_roi = np.zeros((image_src.shape[0], image_src.shape[1]), dtype=np.uint8)
        mask_roi[10:340, 250:630] = 255
        mask_bg = cv.bitwise_not(mask_roi)
        image_roi = cv.bitwise_and(image_src, image_src, mask=mask_roi)

        # 每次进入回调函数  初始化检测结果
        self.detect_result.result = 'failed'
        self.detect_result.objects = []

        # image_create = cv.imread(self.IMAGE_CREATE_PATH)
        # self.image_src = image_create.copy()

        hsv_process(image_roi, True, True)

        # image, objects = self.detect(self.image_src)
        image_detect, objects = self.detect_color(image_roi)

        for obj in objects:
            self.detect_result.objects.append(ProcessResult(obj['shape'], obj['center'], obj['angle']))
        self.detect_result.result = 'successful'

        detection = image_detect + cv.bitwise_and(image_src, image_src, mask=mask_bg)

        cv.imshow('detection', detection)
        cv.waitKey(5)

    def image_sub_callback_2(self, data):
        try:
            self.image_src_2 = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        cv.imshow('eye-in-hand', self.image_src_2)
        cv.waitKey(5)

    def detect(self, image):
        """
            根据角点、边长、最小外接矩形等几何关系识别不同的俄罗斯方块
        """
        image_bgr = image.copy()
        
        objects = []
        
        segmentation_mask, masks = hsv_process(image_bgr, DISPLAY=True, TEST=False)
        contours = cv.findContours(segmentation_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        # print('---------------------')
        # print('len(contours): ', len(contours))

        # ############### test ########################
        # print(type(contours))
        # for contour in contours:
        #     # print(cv.arcLength(contour, True))
        #     epsilon = 0.03 * cv.arcLength(contour, True)
        #     vertex = np.squeeze(cv.approxPolyDP(contour, epsilon, True))
        #     cv.drawContours(image_bgr, [vertex.reshape(-1,1,2)], -1, (100,100,100), 2)

        # cv.imshow('mask', mask)
        # cv.imshow('image', image_bgr)
        # cv.waitKey(0)

        # return
        # ##############################################
        
        for contour in contours:
            # print('arcLenth: ', cv.arcLength(contour, True))
            if cv.arcLength(contour, True) < 20 or cv.arcLength(contour, True) > 2000:
                continue
            epsilon = 0.03 * cv.arcLength(contour, True)
            vertex = np.squeeze(cv.approxPolyDP(contour, epsilon, True)) # shape:(n,2) n个顶点

            min_rect = cv.minAreaRect(contour)
            min_rect_points = np.array(cv.boxPoints(min_rect), dtype=np.int32)
            # print('min_rect_points: ', min_rect_points)
            # print('len(vertex): ', len(vertex))
            # print('vertex: ', vertex) # 顶点从最上方开始逆时针排序
            # cv.drawContours(image_bgr, [vertex], -1, (100,100,100), 2)
            x_list = [vertex[i][0] for i in range(len(vertex))]
            y_list = [vertex[i][1] for i in range(len(vertex))]
            x_min_pos, x_max_pos = np.argmin(x_list), np.argmax(x_list)
            y_min_pos, y_max_pos = np.argmin(y_list), np.argmax(y_list)
            side = [] # 轮廓的各边长
            for i in range(len(vertex)):
                side.append(np.sqrt((vertex[i][0]-vertex[(i+1) % len(vertex)][0]) ** 2 + 
                                    (vertex[i][1]-vertex[(i+1) % len(vertex)][1]) ** 2))
            # print('side: ', np.array(side, dtype=np.int32))
            side_max_pos = np.argmax(side)
            side_min_pos = np.argmin(side)
            
            object_info = {'shape':None, 'center':None, 'angle':None}

            if len(vertex) == 4:
                # 4个顶点，形状I / O
                if abs(side[0]-side[1]) < 5 and abs(side[1]-side[2]) < 5:
                    # 邻边等长  形状O  角度范围[0,90)
                    object_info['shape'] = 'O'
                    # angle：水平轴逆时针旋转直到与第一条边接触，经过的角度
                    object_info['angle'] = math.degrees(math.atan2(abs(vertex[y_max_pos][1] - vertex[(y_max_pos+1) % 4][1]), 
                                                                abs(vertex[y_max_pos][0] - vertex[(y_max_pos+1) % 4][0])))
                    center_x = int(vertex[x_min_pos][0] + (vertex[(x_min_pos+2) % 4][0] - vertex[x_min_pos][0]) / 4)
                    center_y = int(vertex[x_min_pos][1] + (vertex[(x_min_pos+2) %4 ][1] - vertex[x_min_pos][1]) / 4)
                    object_info['center'] = [center_x, center_y]
                else:
                    # 邻边不等长  形状I  角度范围[0,180)
                    object_info['shape'] = 'I'
                    # angle: 水平轴沿逆时针方向与长边所成的夹角
                    if (abs(y_list[0]-y_list[1]) < 2 and (side[0] - side[1]) > 5) or (abs(y_list[0]-y_list[3]) < 2 and (side[0] - side[1]) < -5):
                        # angle = 0  中心点位于右二格
                        object_info['angle'] = 0.0
                        center_y = int(np.mean(y_list))
                        center_x = int(vertex[1][0] + (vertex[0][0]-vertex[1][0]) * 5 / 8) if abs(y_list[0]-y_list[1]) < 3 else \
                                    int(vertex[0][0] + (vertex[3][0]-vertex[0][0]) * 5 / 8)
                        object_info['center'] = [center_x, center_y]
                    elif (abs(y_list[0]-y_list[1]) < 2 and (side[0] - side[1]) < -5) or (abs(y_list[0]-y_list[3]) < 2 and (side[0] - side[1]) > 5):
                        # angle = 90  中心点位于上二格
                        # print(90)
                        object_info['angle'] = 90.0
                        center_x = int(np.mean(x_list))
                        center_y = int(vertex[1][1] + (vertex[2][1]-vertex[1][1]) * 3 / 8) if abs(y_list[0]-y_list[1]) < 3 else \
                                    int(vertex[0][1] + (vertex[1][1]-vertex[0][1]) * 3 / 8)
                        object_info['center'] = [center_x, center_y]

                        # print('angle=90')
                    else:
                        # 中心点位于偏上二格
                        if side[y_max_pos] - side[(y_max_pos-1) % 4] > 5:
                            # angle < 90
                            object_info['angle'] = math.degrees(math.atan2(abs(vertex[y_max_pos][1]-vertex[(y_max_pos+1)%4][1]), 
                                                                            abs(vertex[y_max_pos][0]-vertex[(y_max_pos+1)%4][0])))
                            # 找到上下两条短边的中点
                            point1 = [(vertex[0][0] + vertex[3][0]) / 2, (vertex[0][1] + vertex[3][1]) / 2]
                            point2 = [(vertex[1][0] + vertex[2][0]) / 2, (vertex[1][1] + vertex[2][1]) / 2]
                            center_x = int(point1[0] + (point2[0] - point1[0]) * 3 / 8)
                            center_y = int(point1[1] + (point2[1] - point1[1]) * 3 / 8)
                            object_info['center'] = [center_x, center_y]
                        else:
                            # angle > 90
                            object_info['angle'] = math.degrees(math.atan2(abs(vertex[y_max_pos][1]-vertex[(y_max_pos+1)%4][1]), 
                                                                            abs(vertex[y_max_pos][0]-vertex[(y_max_pos+1)%4][0]))) + 90
                            # 找到上下两条短边的中点
                            point1 = [(vertex[0][0] + vertex[1][0]) / 2, (vertex[0][1] + vertex[1][1]) / 2]
                            point2 = [(vertex[2][0] + vertex[3][0]) / 2, (vertex[2][1] + vertex[3][1]) / 2]
                            center_x = int(point1[0] + (point2[0] - point1[0]) * 3 / 8)
                            center_y = int(point1[1] + (point2[1] - point1[1]) * 3 / 8)
                            object_info['center'] = [center_x, center_y]
                            
                            # print(y_max_pos, object_info['angle'])
                            # if object_info['angle'] < 100:
                            #     print('------------------')
                            #     print('angle < 100')
                            #     print(vertex)
                            #     print(side)
                            #     print('------------------')
                            
                            object_info['angle'] = -min_rect[2] + 90

            elif len(vertex) == 6:
                # 6个顶点，形状J / L
                if abs(side[(side_max_pos-1) % 6] - side[side_min_pos]) < 5:
                    # 最长边的前一条边是最短边  形状L
                    object_info['shape'] = 'L'
                    point1 = [vertex[(side_max_pos+1) % 6][0], vertex[(side_max_pos+1) % 6][1]] # 外拐顶点
                    point2 = [vertex[(side_max_pos-2) % 6][0], vertex[(side_max_pos-2) % 6][1]] # 内拐顶点
                    object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                    # print(side_max_pos, x_min_pos, x_max_pos, y_min_pos, y_max_pos)

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
                else:
                    object_info['shape'] = 'J'
                    point1 = [vertex[side_max_pos][0], vertex[side_max_pos][1]] # 外拐顶点
                    point2 = [vertex[(side_max_pos + 3) % 6][0], vertex[(side_max_pos + 3) % 6][1]] # 内拐顶点
                    object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                    # print(side_max_pos, x_min_pos, x_max_pos, y_min_pos, y_max_pos)

                    # 以外拐顶点为原点  按照长边所在象限进行分类  角度范围[-180,180)
                    if side_max_pos == y_max_pos + 1 and side_max_pos == x_min_pos + 2:
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

            elif len(vertex) == 8:
                # 8个顶点，形状S / T / Z
                min_area_box = cv.minAreaRect(contour)
                box_points = np.array(cv.boxPoints(min_area_box), dtype=np.int32) # 矩形顶点顺时针排序

                if np.max(side) > 2.5 * np.min(side):
                    # 最长边大于最短边的两倍  形状T  角度范围[-180, 180)
                    object_info['shape'] = 'T'
                    # 找到与最长边相邻的两条短边的中点
                    point1 = [(vertex[side_max_pos][0] + vertex[(side_max_pos - 1) % 8][0]) / 2,
                            (vertex[side_max_pos][1] + vertex[(side_max_pos - 1) % 8][1]) / 2]
                    point2 = [(vertex[(side_max_pos + 1) % 8][0] + vertex[(side_max_pos + 2) % 8][0]) / 2,
                            (vertex[(side_max_pos + 1) % 8][1] + vertex[(side_max_pos + 2) % 8][1]) / 2]
                    object_info['center'] = [int((point1[0] + point2[0]) / 2), int((point1[1] + point2[1]) / 2)]

                    # 根据最小包围矩形的顶点与物体顶点的对应情况  判断位于哪一象限
                    if sum(abs(box_points[0] - vertex[side_max_pos])) < 10 and \
                        sum(abs(box_points[3] - vertex[(side_max_pos + 1) % 8])) < 10:
                        # 长边位于第一象限
                        object_info['angle'] = -min_area_box[2]
                    elif sum(abs(box_points[3] - vertex[side_max_pos])) <10 and \
                        sum(abs(box_points[2] - vertex[(side_max_pos + 1) % 8])) < 10:
                        # 长边位于第二象限
                        object_info['angle'] = -min_area_box[2] + 90
                        if object_info['angle'] == 180:
                            object_info['angle'] = -180
                    elif sum(abs(box_points[2] - vertex[side_max_pos])) < 10 and \
                        sum(abs(box_points[1] - vertex[(side_max_pos + 1) % 8])) < 10:
                    # 长边位于第三象限
                        object_info['angle'] = -min_area_box[2] - 180
                    elif sum(abs(box_points[1] - vertex[side_max_pos])) < 10 and \
                        sum(abs(box_points[0] - vertex[(side_max_pos + 1) % 8])) < 10:
                        object_info['angle'] = (-min_area_box[2]) - 90
                    
                else:
                    # 形状S / Z  角度范围[0, 180)
                    if abs(min_area_box[1][0] - side[side_max_pos]) > 10:
                        # 最小矩形的w大于长边  第一象限
                        object_info['angle'] = -min_area_box[2]
                        # 找到矩形两条短边的偏上四等分点
                        point1 = [box_points[1][0] + (box_points[0][0] - box_points[1][0]) / 4,
                                    box_points[1][1] + (box_points[0][1] - box_points[1][1]) / 4]
                        point2 = [box_points[2][0] + (box_points[3][0] - box_points[2][0]) / 4,
                                    box_points[2][1] + (box_points[3][1] - box_points[2][1]) / 4]
                        object_info['center'] = [int((point1[0] + point2[0]) / 2),
                                                    int((point1[1] + point2[1]) / 2)]
                        if sum(abs(vertex[side_max_pos] - box_points[0])) < 10 or \
                            sum(abs(vertex[side_max_pos] - box_points[2])) < 10:
                            # 物体长边对应的顶点与最小矩形顶点重合  形状S
                            object_info['shape'] = 'S'
                        else:
                            # 物体长边对应的顶点与最小矩形顶点不重合  形状Z
                            object_info['shape'] = 'Z'
                    else:
                        # 最小矩形的w等于长边  第二象限
                        object_info['angle'] = -min_area_box[2] + 90
                        # 找到矩形两条短边的偏下四等分点
                        point1 = [box_points[0][0] + (box_points[3][0] - box_points[0][0]) / 4,
                                    box_points[0][1] + (box_points[3][1] - box_points[3][1]) / 4]
                        point2 = [box_points[1][0] + (box_points[2][0] - box_points[1][0]) / 4,
                                    box_points[1][1] + (box_points[2][1] - box_points[1][1]) / 4]
                        object_info['center'] = [int((point1[0] + point2[0]) / 2),
                                                    int((point1[1] + point2[1]) / 2)]
                        if sum(abs(vertex[(side_max_pos + 1) % 8] - box_points[0])) < 10 or \
                            sum(abs(vertex[(side_max_pos + 1) % 8] - box_points[2])) < 10:
                            # 物体长边对应顶点的下一顶点与最小矩形顶点重合  形状Z
                            object_info['shape'] = 'Z'
                            # 0°情况特殊
                            if object_info['angle'] == 180:
                                object_info['angle'] = 0
                                point1 = [box_points[2][0] + (box_points[1][0] - box_points[2][0]) / 4,
                                            box_points[2][1] + (box_points[1][1] - box_points[2][1]) / 4]
                                point2 = [box_points[3][0] + (box_points[0][0] - box_points[3][0]) / 4,
                                            box_points[3][1] + (box_points[0][1] - box_points[3][1]) / 4]
                                object_info['center'] = [int((point1[0] + point2[0]) / 2),
                                                            int((point1[1] + point2[1]) / 2)]
                        else:
                            # 物体长边对应顶点的下一顶点与最小矩形顶点不重合  形状S
                            object_info['shape'] = 'S'
            
            # print(object_info)
            objects.append(object_info)

            x,y,w,h = cv.boundingRect(contour)
            # print(x,y)
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

        ############## hsv阈值调整 ###################
        # while True:
        #     hsv_process(image_src, TEST=True)
        #############################################

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
                    if cv.contourArea(contour) < 800 or cv.contourArea(contour) > 2500:
                        continue
                    epsilon = 0.03 * cv.arcLength(contour, True) # 0.03
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
                        if cv.contourArea(contour) > 2000:
                            # 粘连情况
                            # print(len(vertex))
                            # for point in vertex:
                            #     cv.circle(image_bgr, (point[0],point[1]), 5, (0,255,0), -1)

                            # x,y,w,h = cv.boundingRect(contour)
                            # mask = np.zeros((480,640,3), dtype=np.uint8)
                            # mask[y-10:(y+h+10),x-10:(x+w+10)] = image_bgr[y-10:(y+h+10),x-10:(x+w+10)]

                            # gray = cv.cvtColor(mask, cv.COLOR_BGR2GRAY)
                            # corners = cv.goodFeaturesToTrack(gray,13,0.01,10)
                            # # 返回的结果是 [[ 311., 250.]] 两层括号的数组。
                            # corners = np.int0(corners)
                            # for i in corners:
                            #     x,y = i.ravel()
                            #     cv.circle(mask,(x,y),3,255,-1)

                            # hull = cv.convexHull(contour,returnPoints = False)
                            # defects = cv.convexityDefects(contour, hull)
                            # for i in range(defects.shape[0]):
                            #     s,e,f,d = defects[i,0]
                            #     start = tuple(contour[s][0])
                            #     end = tuple(contour[e][0])
                            #     far = tuple(contour[f][0])
                            #     cv.line(mask,start,end,[0,255,0],2)
                            #     cv.circle(mask,far,5,[0,0,255],-1)

                            # cv.imshow('edge', mask)
                            # cv.imshow('edge', edge)
                            # cv.waitKey(0)

                            # canny_th(mask)
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

    def image_srv_callback(self, req):
        print('object detection service callback...')
        # print('request signal: ', req.signal)

        return self.detect_result


if __name__ == '__main__':

    # main()
    # mask_create()
    # data_create()
    # data_create_1()

    try:
        ImageProcess()
        ros.spin()
    except KeyboardInterrupt:
        print("Shut down detect node.")
        cv.destroyAllWindows()
