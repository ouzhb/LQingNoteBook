---
title: RealSense摄像头
categories: "RealSense" 
date: 2019-11-21
comments: false
toc: true
tags:
    - RealSense
    - 机器视觉
---

RealSense摄像头在环境中的使用

<!--more-->

# 安装驱动


IntelRealSense官方在Github上提供驱动程序的源码([链接](https://github.com/IntelRealSense/librealsense))，以及Linux \ Windows \ Mac OS \ Android 等系统SDK安装包。

SDK中包含以下工具：

- Intel® RealSense™ Viewer：快速获取摄像头深度图片的GUI工具
- Depth Quality Tool：用来获取深度图像的一些统计信息、质量指标
- Debug Tools：包括一些其他调试工具
- Sample Code：示例程序代码，主要是C/C++（[链接](https://github.com/IntelRealSense/librealsense/tree/master/examples)）
- Wrappers：包括python、Ros、OpenCV等各种语言的API([链接](https://github.com/IntelRealSense/librealsense/tree/development/wrappers))

## Python Demo

``` shell
pip install pyrealsense2
```



基于RealSense障碍物距离检测，实现参照以下步骤：

- 从相机获取深度信息
- 对深度信息进行以下预处理，获取**二值图**
  - 将**深度信息**转换为**深度图**
  - 对深度图进行**中值滤波**
  - 转换为灰度图，并且进行二值化
  - **开操作**去除二值图噪声
- 使用findContours寻找二值图的轮廓
- 获取轮廓中心点，以此中心点的深度信息作为障碍物的距离

```python
# coding:utf-8
import math

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt


def deal(depth, color):
    """
        处理获取的每一帧图像
    :param depth:
    :param color:
    :return:
    """
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth.get_data())
    color_image = np.asanyarray(color.get_data())

    # 将深度图像转换成带色彩的深度图
    tmp = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 将16位的深度图片转换位8位图形进行显示
    depth_colormap = cv2.applyColorMap(tmp, cv2.COLORMAP_JET)  # 将深度信息转换成带颜色的深度图

    # 去除黑边,这里需要除去黑边的原因是：图像左侧有一条很长的黑色竖线
    # deepImg_cut = depth_colormap[:, 40:].copy()

    # 中值滤波
    img_median = cv2.medianBlur(depth_colormap, 7)

    # 转换为单通道图(灰度)
    gray = cv2.cvtColor(img_median, cv2.COLOR_RGB2GRAY)
    ret, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)  # 二值图
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (75, 75))  # 膨胀或者腐蚀操作使用的核函数
    mask = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)  # 开操作，去除图片中的噪声点（先膨胀，再腐蚀）

    # 寻找轮廓,contours中每一个元素是一个point序列，表示一组轮廓
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # 计算凸包,并根据凸包面积筛选
    length = len(contours)
    # 根据面积阈值过滤contours
    contours_area = []
    contours_select = []
    for i in range(length):
        area = cv2.contourArea(contours[i])  # 计算每个轮廓包围的面积，并且使用面积大小进行过滤
        if area >= 500:  # area_threshold = 500
            xycenter = CalCenterCoo(contours[i])  # 这里返回的是闭包内的若干个采样点
            # tmp = xycenter.append(area)
            contours_area.append(xycenter)
            contours_select.append(contours[i])

    cv2.drawContours(color_image, contours_select, -1, (0, 0, 255), 3)
    cv2.drawContours(mask, contours_select, -1, (0, 0, 255), 3)
    images = [color_image, mask]
    return contours_area, images


def GetDist(depth_frame, ret):
    """
        在深度图上获取指定点的深度信息，在闭包的9个采样点中取最近的点
    :return:
    """

    distArray = []
    for r in ret:
        dist = 10
        for i in r:
            x = i[0]
            y = i[1]
            dist_tmp = depth_frame.get_distance(x, y)
            if dist_tmp > 0 and dist_tmp < dist:
                dist = dist_tmp
        if dist == 10:
            dist = -1
        distArray.append(dist)

    d = ",".join(map(lambda x: str(x), distArray))
    print "障碍物: %s" % d


def CalCenterCoo(contours):
    """
        取区域闭包内的中心点，以及其周围9个采样点
    :param contours:
    :return:
    """
    x_list = []
    y_list = []
    xyre_list = []
    for cont in contours:
        x_list.append(cont[0][0])
        y_list.append(cont[0][1])

    x_center = (min(x_list) + max(x_list)) / 2
    y_center = (min(y_list) + max(y_list)) / 2
    xyre_list.append([x_center, y_center])

    x_step = math.floor((max(x_list) - min(x_list)) / 10)
    y_step = math.floor((max(y_list) - min(y_list)) / 10)

    # TODO：这里这样取可能会取到闭包之外的点
    for i in range(9):
        x_tmp = int(min(x_list) + i * x_step)
        y_tmp = int(min(y_list) + i * y_step)
        xyre_list.append([x_tmp, y_tmp])

    return xyre_list


if __name__ == "__main__":
    # 打开链接摄像头的管道
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)  # 配置深度图信息
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)  # 配置RGB图像信息
    device = pipeline.start(config)

    depth_scale = device.get_device().first_depth_sensor().get_depth_scale()
    print depth_scale  # 深度信息和meter的转换关系， depth*depth_scale > meters

    try:
        while True:
            # 获取图像信息
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # 处理获取的图像
            contours_area, images = deal(depth_frame, color_frame)

            # Get distance
            GetDist(depth_frame, contours_area)

            # Show images
            for i in xrange(images.__len__()):
                cv2.namedWindow('RealSense_{}'.format(i), cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense_{}'.format(i), images[i])

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    finally:
        pipeline.stop()

```





## ROS Wrappers

ROS API是一个独立项目（[github](https://github.com/IntelRealSense/realsense-ros)）。

安装步骤：

```shell
# 参考官方说明在ROS环境中安装好librealsense

# 安装以下依赖包
sudo apt install -y ros-kinetic-ddynamic-reconfigure
sudo apt install -y ros-kinetic-ddynamic-reconfigure-python

# 将realsense-ros仓库的密码Copy到SRC目录，执行编译安装

catkin_init_workspace # 在src目录执行
catkin_make clean     # 在工程目录执行
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

# echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
# source ~/.bashrc

# 在ROS中运行，其他topic和参数信息可以参考
roslaunch realsense2_camera rs_camera.launch
```