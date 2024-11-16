#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
import os
import sys
# ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
# if ros_cv2_path in sys.path: sys.path.remove(ros_cv2_path)
import cv2
import cv2.aruco as aruco
# sys.path.append(ros_cv2_path)
import json
import argparse

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', '-d', type=str, default="kinect", choices=["realsense", "kinect"])
    args = parser.parse_args()
    return args

# device=0, calibrate kinect; device=1, calibrate realsense.
def calibrate_extrinsic(device, intrinsic_path, extrinsic_path):
    # Topic
    color_topic = None
    depth_topic = None
    if device == 1:
        # Realsense,640*480
        color_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
    else:
        # # Kinectv2,sd:512*424,qhd:960*540,hd:1920*1080
        color_topic = "/kinect2/hd/image_color_rect"
        depth_topic = "/kinect2/hd/image_depth_rect"

    # rosrun
    try:
        # 初始化ros节点
        rospy.init_node("test")
        rospy.loginfo("Starting test")
        rgb = rospy.wait_for_message(color_topic, Image, timeout=None)
        rgb = np.ndarray(shape=(rgb.height, rgb.width, 3), dtype=np.uint8
                           , buffer=rgb.data)
        # rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # depth = rospy.wait_for_message(depth_topic, Image, timeout=None)
        # depth = np.ndarray(shape=(depth.height, depth.width), dtype=np.uint16
        #                             , buffer=depth.data)

        # 转化为灰度图
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        # 设置预定义的字典
        aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # 使用默认值初始化检测器参数
        parameters = aruco.DetectorParameters_create()
        # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # 画出标志位置
        aruco.drawDetectedMarkers(rgb, corners, ids)
        cv2.imshow("frame", rgb)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # 获取角点作为控制点
        id0_corners = np.zeros((4, 2), dtype=np.double)
        id5_corners = np.zeros((4, 2), dtype=np.double)
        for i in range(len(ids)):
            # print(ids[i])
            if ids[i] == 0:
                id0_corners = corners[i]
            if ids[i] == 5:
                id5_corners = corners[i]

        # 8个控制点
        points2d = np.vstack((np.array(id0_corners[0]), np.array(id5_corners[0])))
        points2d = points2d.astype(np.double)
        # print("Image point:\n", points2d)
        points3d = np.zeros((8, 3), dtype=np.double)
        points3d[0] = [-0.0676, 0.3281, -0.0088]
        points3d[1] = [-0.0676, 0.3764, -0.0088]
        points3d[2] = [-0.0193, 0.3764, -0.0088]
        points3d[3] = [-0.0193, 0.3281, -0.0088]
        points3d[4] = [-0.0097, 0.3281, -0.0088]
        points3d[5] = [-0.0097, 0.3764, -0.0088]
        points3d[6] = [0.0386, 0.3764, -0.0088]
        points3d[7] = [0.0386, 0.3281, -0.0088]
        # print("World point:\n", points3d)

        f = open(intrinsic_path, 'r')
        intrinsic = json.load(f)
        camera_matrix = np.array(
            [[intrinsic["fx"], 0, intrinsic["cx"]],
             [0, intrinsic["fy"], intrinsic["cy"]],
             [0, 0, 1]], dtype=np.double)
        dist_coeffs = np.zeros((4, 1), dtype=np.double)  # Assuming no lens distortion

        # solvePnP
        (success, rotation_vector, translation_vector) = \
            cv2.solvePnP(points3d, points2d, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        rotM = cv2.Rodrigues(rotation_vector)[0]
        print(success)
        print(rotM)
        print(translation_vector)

        # Reproject error
        projectPoints, _ = cv2.projectPoints(points3d, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        projectPoints = np.squeeze(projectPoints)
        print("Point number: ", len(projectPoints))
        error = np.sum(np.linalg.norm(points2d - projectPoints, axis=1)) / 8
        print("Reproject error: ", error)

        # save json
        data = {}
        data["rotation"] = np.squeeze(rotM.reshape((9, 1))).tolist()
        data["translation"] = np.squeeze(translation_vector).tolist()
        jsondata = json.dumps(data)
        f = open(extrinsic_path, 'w')
        f.write(jsondata)
        f.close()

    except KeyboardInterrupt:
        print("Shutting down test node.")
        cv2.destroyAllWindows()
