---
title: PCL 调研
categories: "PCL" 
date: 2019-09-11
comments: false
toc: true
tags:
    - PCL
    - 机器视觉
---

调研点云处理库PCL

<!--more-->

# 安装部署

```shell
# 从源码安装

cd pcl-pcl-1.9.1 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
```

# 模块介绍

PCL是用于处理3D点云的一组API接口，其内部按照功能分为以下多个模块：

- Filters：滤波接口
- Features：从点云数据中提取3D特征数据
- Keypoints：特征点检测
- Registration：用于数据配准（根据图片的特征点，将多个图幅配准到同一个中）
- Kd-tree：kdtree算法库
- Octree：？？
- Segmentation
- Sample Consensus
- Surface：通过3D扫描重建原始曲面
- Range Image：通过这个库可以实现深度图和点云数据之间的转换
- I/O：用于连接硬件设备、以及读写点云文件的接口
- Visualization：用于可视化点云图像的GUI
- Common：基础库，主要包括类型定义、以及常用变换函数
- Search：搜索库主要和Octree、Kd-tree一起使用，
- Binaries：包含PCL相关的调试工具
[pcl_ros](http://wiki.ros.org/pcl_ros)

[Tutorials](http://www.pointclouds.org/documentation/tutorials/)

[python-pcl](https://github.com/strawlab/python-pcl)

[python-pcl说明文档](chrome-extension://ikhdkkncnoglghljlkmcimlnlhkeamad/pdf-viewer/web/viewer.html?file=https%3A%2F%2Fbuildmedia.readthedocs.org%2Fmedia%2Fpdf%2Fpython-pcl-fork%2Frc_patches4%2Fpython-pcl-fork.pdf)