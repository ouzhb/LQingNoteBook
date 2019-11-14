---
title: ROS环境安装部署
categories: "机器人" 
date: 2019-08-08
comments: false
toc: true
tags:
    - ROS
    - Ubuntu
---


<!--more-->

# 操作系统配置

安装Ubuntu 18.04版本操作系统，安装完成后进行以下配置：

### 1. 配置sudo权限

```shell
sudo passwd root
sudo -s
#编辑/etc/sudoers，添加 ruijie  ALL=(ALL:ALL) ALL
```
### 2. 安装SSH服务

```shell
sudo apt-get install openssh-server net-tools vim

# 修改sshd_config文件，添加PermitRootLogin yes，注释掉PermitRootLogin prohibit-password
vim /etc/ssh/sshd_config

sudo service sshd restart

```

### 3. 配置远程桌面

Ubuntu 18.04使用官方xRDP时无法正常登陆，参考Goolge上的[解决方案](http://c-nergy.be/blog/?p=13455)安装第三方版本的xRDP。

```shell
sudo add-apt-repository ppa:martinx/xrdp-hwe-18.04
sudo apt-get update
sudo apt-get install xrdp xorg
sudo adduser xrdp ssl-cert
# 修改 /usr/share/polkit-1/actions/org.freedesktop.color.policy（直接删掉）
# 参考：https://c-nergy.be/blog/?p=12073
reboot
```

# ROS安装

## 版本选择

当前ROS包含两个长期维护版本，分别支持不同版本操作系统。通过 rosdistro 工具，我们可以在系统中安装多个版本的ROS。

通常ROS发行版的维护周期是5年或者2年，当前仍然在维护周期内的发行版是以下两个，均为5年维护周期。

|版本名称|Release|EOL|操作系统|
|----|----|----|----|
|Melodic Morenia|May 23rd, 2018|May, 2023|Ubuntu 18.04、Debian、Win10|
|Kinetic Kame|May 23rd, 2016|April, 2021(Xenial EOL)|Ubuntu 15.10</br>Ubuntu 16.04</br>Debian 8|

**文本选择在Ubuntu 18.04环境中安装，ROS Melodic Morenia。**

## 安装过程

参考官方文档，通过以下步骤安装ROS Melodic：

### 1. 部署repositories

ROS除了提供官方仓库以外，在全球范围提供了[镜像仓库](http://wiki.ros.org/ROS/Installation/UbuntuMirrors)，安装过程中为了避免网络问题，同时配置两个国内的仓库地址。

```shell
# 添加官方仓库
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# 添加中科大仓库
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" >> /etc/apt/sources.list.d/ros-latest.list'
# 添加清华仓库
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" >> /etc/apt/sources.list.d/ros-latest.list'
# 添加仓库的Key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 2. 安装DEB

```shell
sudo apt update
sudo apt install ros-melodic-desktop-full
```
Ros提供下面几种安装包，本文选择ros-melodic-desktop-full，完整安装所有工具。

|DEB包|包含组件|说明|
|----|----|----|
|ros-melodic-desktop-full|ROS、rqt、rviz</br>robot常用lib</br>2D/3D仿真器传感器|完整安装|
|ros-melodic-desktop|ROS、rqt、rviz</br>robot常用lib||
|ros-melodic-ros-base|不包含GUI||
|ros-melodic-PACKAGE|单个模块|PACKAGE表示模块名，如ros-melodic-slam-gmapping|

**备注*：RViz和rqt均是ROS的可视化工具，

### 3. 初始化ROS

```shell
# 初始化rosdep仓库
sudo rosdep init
rosdep update
# 配置环境变量
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# 下载依赖
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

# 参考

[ROS安装](http://wiki.ros.org/ROS/Installation)

[ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)