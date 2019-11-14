---
title: OpenCV 学习笔记
categories: "机器学习" 
date: 2019-09-11
comments: false
toc: true
tags:
    - OpenCV
    - 机器视觉
---

整理机器学习中的一些常见概念。

<!--more-->

# 直方图均衡

直方图均衡化是通过调整图像的灰阶分布，使像素点在0~255灰阶上的分布更加均衡，从而提高了图像的对比度。

简单理解，图像直方图均衡化是一种**增强图像对比度**的图像增强方法。上述教程中，处理的图像均是均衡之后的图像。

```python
    # 通过opencv将灰度图像进行均衡化
    grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    equalizeHist = cv.equalizeHist(grey)
    cv.imshow("equalizeHist", equalizeHist)
```

PS：

*对比度*可以理解为：就一幅图片而言，图片上最亮处与最黑处的比值。

数字图像处理中的对比度拉伸是当一幅图像中的亮暗差较小时（例如只有50~150），我们比较均匀的扩展它的对比度，将它的幅值拉伸到10~180或5~240。这样，暗处显得更黑，亮处显得更亮些，由于是均匀拉伸，原本不易分辨的灰度差（中间灰度）也拉开了距离，显得图片更富有层次。


# 图形金字塔

[参考](https://www.kancloud.cn/aollo/aolloopencv/272073),[参考](https://blog.csdn.net/xbcreal/article/details/52629465) 定义：

- 金字塔是一系列分辨率逐步降低，且来源于同一张原始图的图像集合。
- 金字塔的底部是待处理图像的高分辨率表示，而顶部是低分辨率的近似。
- 常见金字塔：高斯金字塔、拉普拉斯金字塔

    - 高斯金字塔: 用来向下采样（向下指分辨率降低，实际是指金字塔第i层到i+1层），主要的图像金字塔
    - 拉普拉斯金字塔：从金字塔低层图像重建上层未采样图像


# 参考

[OpenCV-Python中文教程](https://www.kancloud.cn/aollo/aolloopencv/269602)