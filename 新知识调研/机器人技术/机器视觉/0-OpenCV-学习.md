---
title: OpenCV 学习笔记
categories: "机器学习" 
date: 2019-09-04
comments: false
toc: true
tags:
	- OpenCV
---

关于[OpenCV-Python Tutorials](https://docs.opencv.org/4.1.1/d6/d00/tutorial_py_root.html)、ROS By Example的学习笔记



<!--more-->

# ROS和OpenCV

在ROS中使用OpenCV进行图像处理是开发ROS程序的常见需要，以下是在Ros中使用OpenCV开发的一些随笔。

## 摄像头的驱动

ROS中使用摄像机包括下面的情景：

### 1.Kinect

在ROS上安装Kinect驱动有以下几种方案：

- freenect_stack：底层基于libfreenect，需要额外安装[libfreenect](https://github.com/OpenKinect/libfreenect)或者[libfreenect2](https://github.com/OpenKinect/libfreenect2)

- openni_camera：同样基于openni需要分别安装依赖OpenNI和SensorKinect，安装过程可以参考[博客](https://www.20papercups.net/programming/kinect-on-ubuntu-with-openni/)

- openni_kinect：基于openni是PrimeSense官方负责维护项目，包含：openni_camera、openni_launch、openni_tracker等多个模块。**但是PrimeSense被收购之后，该项目的源码已经无法访问。**

- kinect_aux: Kinect上accelerometer/tilt/led相关功能的独立驱动。

### 2. IntelRealSense/Xtion

IntelRealSense官方提供了ROS上的相关驱动，官方[github](https://github.com/IntelRealSense/realsense-ros)。

Xtion使用[openni2_camera](http://wiki.ros.org/openni2_camera)驱动，使用相机是可以参考[openni2_launch](http://docs.ros.org/api/openni2_launch/html/)。

### 3. webcam 

 webcam是指通过USB接入的“即插即用”相机，此类相机没有专门的驱动程序，通过系统工具工作。

- [libuvc_camera](http://wiki.ros.org/libuvc_camera)
- [usb_cam](https://wiki.ros.org/usb_cam)
- [gscam](http://wiki.ros.org/gscam)

## ROS处理图像

ROS中相机信息通过Topic进行发布，其中关键Topic包括：

- image_raw：摄像头拍摄的原始数据，格式为[**sensor_msgs/Image.msg**](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
- camera_info：摄像头元数据信息，包括内参矩阵、对准系数等等，通常这个Topic中数据需要用户进行对准之后，进行人为设定。

ROS图像处理方面涉及到下面这些工具：

### 1. **cv_bridge**

[cv_bridge](http://wiki.ros.org/cv_bridge/Tutorials)用来在sensor_msgs/Image和numpy两种格式之间进行转换，使OpenCV能够处理Topic中的图像数据。

目前cv_bridge提供C++/Python/JAVA 的接口。

### 2. **image_transport**

image_transport 可以将Image数据重新转发到新的topic中，其输入可以是Topic、图片、视频。

### 3. **image_pipeline**

[image_pipeline](http://wiki.ros.org/image_pipeline)是ROS的图像处理工具包，包括以下几个部分：

- [camera_calibration](http://wiki.ros.org/camera_calibration)：摄像头标定包
- [image_proc](http://wiki.ros.org/image_proc)：图像校正包
- [stereo_image_proc](http://wiki.ros.org/stereo_image_proc)：处理双目相机
- [depth_image_proc](http://wiki.ros.org/depth_image_proc)：处理深度相机
- image_view、stereo_view：可视化

简单介绍一下image_pipeline的各个组件：

**image_proc**

image_proc主要用来处理rgb图片，提供node、nodelet两种运行方式。

```shell
# node方式运行
ROS_NAMESPACE=my_camera rosrun image_proc image_proc

# 上述node会寻找/my_camera/image_raw和/my_camera/camera_info，并根据后者的参数去校正前者中的图像。最终输出以下（未）校正图像：灰度（去）畸变、彩色（去）畸变。

```

image_proc 还提供了四个nodelet：

- debayer：将image转换成灰度、彩色两个版本并输出
- rectify：校正图像
- crop_decimate：图像抽样，即将图像的像素减小
- resize：调整图像大小

**depth_image_proc**

depth_image_proc主要用来处理深度图像，其所有的功能通过nodelet来提供：

- convert_metric：量测值变换，将深度信息的单位从mm变为m
- disparity：将深度图重变为disparity格式（disparity是一种视差图，可以通过双目相机生成）
- point_cloud_xyz：将深度图转换成点云图像，输出格式为sensor_msgs/PointCloud2
- point_cloud_xyzrgb：将深度图和RGB合成，并转换成点云图像，输出格式为sensor_msgs/PointCloud2
- register：将深度相机的frame-id变换到另一个坐标系中。

### 4. **nodelet**

由于ROS使用Topic方式传输数据存在一定的延时和阻塞。在数据量小、频率低的情况下，传输耗费的时间可以忽略不计。但当传输图像流，点云等数据量较大的消息，或者执行有一定的实时性要求的任务时，因传输而耗费的时间就不得不考虑。nodelet的作用是让多个node在一个进程中用实现零拷贝通信。

用户需要manager节点，该节点管理多个nodelet节点，并为它们提供高性能数据通信。用户需要根据实际需要开发nodelet（nodelet基于pluginlib插件机制），并将nodelet加载到nodelet manager中（[blog关于nodelet的介绍](https://www.cnblogs.com/21207-iHome/p/8213411.html)）。

当前图像相关的包多数提供了node和nodelet两种运行模式，使用以下命令可以查看当前系统中所有可运行nodelet

```shell
rosrun nodelet declared_nodelets
```

以下是调用了image_proc和depth_image_proc中的nodelet实例：

```xml
<launch>

  <group ns="camera">

   
    <include file="$(find rgbd_launch)/launch/includes/manager.launch.xml">
      <arg name="name" value="camera_nodelet_manager" />
      <arg name="debug" value="false" /> <!-- Run manager in GDB? -->
      <arg name="num_worker_threads"  value="2" />
    </include>

    <!-- decimated to 160x120，将像素变为原来1/4 -->
    <node pkg="nodelet" type="nodelet" name="crop_decimate"
          args="load image_proc/crop_decimate /camera/camera_nodelet_manager"
          output="screen">
      <remap from="camera/image_raw" to="rgb/image_raw" />
      <remap from="camera/camera_info" to="rgb/camera_info" />
      <remap from="camera_out" to="depth_downsample" />
      <param name="decimation_x" value="4" />
      <param name="decimation_y" value="4" />
      <param name="queue_size" value="1" />
    </node>

    <!--&lt;!&ndash; downsampled XYZ point cloud  &ndash;&gt;-->
    <node pkg="nodelet" type="nodelet" name="points_downsample"
          args="load depth_image_proc/point_cloud_xyz /camera/camera_nodelet_manager"
          ns="depth_cloud"
          output="screen">
      <remap from="image_rect" to="/camera/depth/image_raw"/>
      <remap from="camera_info" to="/camera/depth/camera_info"/>
    </node>

  </group>

</launch>

```



# ROS By Example的教程

ROS By Example第十章介绍了以下内容：

1. 如何ROS中安装摄像头驱动。
2. 在ROS框架之中使用Python-OpenCV，并通过Topic实现数据交互。
3. 基于普通相机，使用OpenCV实现一个人脸追踪程序。
4. 基于OpenNI的Skeleton Tracking程序。
5. PCL库的介绍

上述内容的源码在Github的[pirobot/rbx1](https://github.com/pirobot/rbx1/tree/indigo-devel/rbx1_vision)项目中，使用官网这份代码有以下几点值得注意：

1. rbx1中不少例子基于OpenCV1编写，由于OpenCV2以上版本和OpenCV1接口并非完全兼容，因此需要修改后才能正常使用。

      - OpenCV2中完全整合numpy，如cv.CreateImage、cv.GetSize等接口已经完全取消，使用numpy的接口替代。
      - cv2中不少常量的名称也和cv中不同

2. [video2ros.py](https://github.com/pirobot/rbx1/blob/kinetic-devel-beta/rbx1_vision/nodes/video2ros.py)能够将avi格式的视频输出到ROS的指定topic中，使用这个脚本可以替代RGB摄像机，该脚本支持视频暂停、循环播放。

3. [ros2opencv2.py](https://github.com/pirobot/rbx1/blob/kinetic-devel-beta/rbx1_vision/src/rbx1_vision/ros2opencv2.py)订阅指定Topic的图像输出，并在process_image函数中进行数据处理。编写视频处理程序时，可以继承该脚本中ROS2OpenCV2类，并复写process_image实现处理逻辑。

4. 在catkin中如何编写python项目的makefile可以参考[catkin_python_setup](http://wiki.ros.org/cn/rospy_tutorials/Tutorials/Makefile)

## 人脸追踪Demo

ROS By Example 10.8 详细介绍了如何利用一个OpenCV的现成算法实现一个人脸追踪程序，该Demo**识别/最终**流程如下：

1. 通过 Haar分类器，在一帧图像中识别人脸（[face_detector.py](https://github.com/pirobot/rbx1/blob/kinetic-devel-beta/rbx1_vision/src/rbx1_vision/face_detector.py)）；
2. 通过 goodFeaturesToTrack方法在人脸区域提取KeyPoint（[good_features.py](https://github.com/pirobot/rbx1/blob/kinetic-devel-beta/rbx1_vision/src/rbx1_vision/good_features.py)）；
3. 通过 OpenCV's Lucas-Kanade optical flow 的实现跟踪上述KeyPoint；
4. 验证当前追踪角点的有效性，添加新KeyPoint或移除失效KeyPoint；

PS：上述方案基于灰度图像进行人脸追踪，教程中还提供了了一种基于图像色彩进行追踪的方案（基于CamShift算法）。

### Haar分类器

在图片中识别人脸是一个分类过程，OpenCV提供基于haar特征和lbp特征的分类器。

使用Haar分类器，需要注意下面几点：

- 需要提供训练好的xml格式的模型文件，每个模型文件可以初始化一个分类器（CascadeClassifier）
- 对分类器调用detectMultiScale可以返回frame中的匹配结果，可以将多个分类器并联（串联）提高识别准确率

OpenCV的Haar分类器实现：Haar-like特征 + AdaBoost + 积分图（用来加速计算特征）

### 角点检测

OpenCV中的角点检测：

- cornerHarris：Harris角点检测，其检测原理、以及特征可以[参考blog](https://www.cnblogs.com/ronny/p/4009425.html)

  - 参数α对角点检测的灵敏度成反比
  - Harris角点检测算子对**亮度**和**对比度**的变化不敏感
  - 旋转不变性、尺度不变性、不具备仿射不变性(但是可以通过Harris-Affine实现)

- goodFeaturesToTrack：Shi-Tomasijiao
- FAST：SUSAN算法

[参考: 图像局部特征点检测算法综述](https://www.cnblogs.com/ronny/p/4260167.html)

### 特征点追踪

ROS by Example 中使用calcOpticalFlowPyrLK追踪，人脸上的角点运动，从而实现人脸跟踪的目的。

calcOpticalFlowPyrLK 是OpenCV提供的Lucas Kanade光流算法的实现，能够计算两帧图像之间特征点的位移。

光流定义为：空间运动物体在观察成像平面上的像素运动的瞬时速度，是利用图像序列中像素在时间域上的变化以及相邻帧之间的相关性来找到上一帧跟当前帧之间存在的对应关系，从而计算出相邻帧之间物体的运动信息的一种方法。


个人理解，光流可以认为是素点的瞬时位移向量，是世界中可以感觉到的明显的视觉运动。

参考：[对OpenCV中光流函数的介绍](https://blog.csdn.net/zouxy09/article/details/8683859)

## 总结

ROS by Example 中人脸跟踪的Demo全都是基于传统CV算法的。由于我没有摄像头，输入使用的是一个清晰度很差的视频图像，最终导致识别和追踪效果全都差强任意。


# 参考

[ROS图像相关包](https://blog.csdn.net/u011722133/article/details/53337818)

[Haar分类器](https://www.cnblogs.com/ello/archive/2012/04/28/2475419.html)



