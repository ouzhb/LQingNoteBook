---
title: ROS学习笔记（一）
categories: "机器人" 
date: 2019-08-09
comments: false
toc: true
tags:
    - ROS
---

了解ROS的基本框架

<!--more-->

# ROS概述

ROS是运行在Linux等操作系统上的一个次级操作系统，提供机器人从开发到运行的一系列工具。

ROS的核心模块包括：

**1. ROS通信机制**

ROS使用分布式架构，将机器人的功能和软件做成一个个node，然后每个node通过topic进行沟通，这些node可以部署在同一台机器上，也可以部署在不同机器上，还可以部署在互联网上。node之间通过**topic发布/订阅**的方式进行通信，或者基于ROS中**Service机制点对点通信**。

**2. 机器人特性功能**

机器人特性功能主要指机器人开发中常用的**数据结构**、**算法**、**规范**等内容。当基于ROS开发机器人程序时，可以直接引用相应lib。

- 标准机器人消息
- 机器人几何库
- 机器人描述语言
- 抢占式远程过程调用
- 诊断
- 位置估计
- 定位导航

**3. 工具集**

工具集只机器人开发过程使用的**辅助工具**，包括：

- 常用命令工具
- 工程管理工具catkin
- 可视化工具是rqt、rviz
- 包管理工具
- 第三方工具

# ROS系统整体架构

## 文件系统架构

文件系统架构实际上指的是ROS程序开发过程中工作空间的架构。

![](https://github.com/LinQing2017/notes/blob/master/pictures/ros%E6%96%87%E4%BB%B6%E6%9E%B6%E6%9E%84.png)

ROS应用在开发过程中使用catkin进行包管理，而catkin的底层是通过CMake工作的。

catkin目录说明：

- src：放置各个功能包和一个用于这些功能包的CMake配置文件CMakeLists.txt。
- build编译空间：放置CMake和catkin编译功能包时产生的缓存、配置、中间文件等。
- devel开发空间：放置编译好的可执行程序，这些可执行程序是不需要安装就能直接运行的。

功能包是ROS程序的最小结构，表示一个node。安装完成ROS后所有自带的功能包位于$ROS_PACKAGE_PATH，用户可以自行开发功能包。

功能包中通常包括下面几个文件：

- CMakeLists.txt：cmake配置文件。
- package.xml：功能包的配置信息，如依赖、名称等。
- include/<package_name>：*.h头文件放在这里。
- msg：非标准消息定义目录，ROS本身已经定义了许多用于node之间通信[标准消息](http://wiki.ros.org/common_msgs/)已经[标准数据类型](http://wiki.ros.org/std_msgs/)。
- srv：服务类型定义目录，ROS中node通信可以通过消息和服务两种方式，消息基于topic订阅，服务则是node之间点对点通信机制。
- scripts：bash、python或其他脚本的可执行文件。
- launch：存放*.launch文件，*.launch文件用于启动ROS功能包中的一个或多个节点（Ansible？？？）
- src：源码目录

## 计算图级架构

计算图实际上指的是多个node在ROS中构成的网络，所有计算图中的节点可以进行数据交互。



计算图中包含以下概念：

### 1. node

节点是计算执行进程，功能包中创建的每个可执行程序在被启动加载到系统进程中后，该进程就是一个ROS节点。

节点都是各自独立的可执行文件，能够通过主题（topic）、服务（server）或参数服务器（parameter server）与其他节点通信。

节点如果用c++进行编写，需要用到ROS提供的库roscpp；节点如果用python进行编写，需要用到ROS提供的库rospy。

### 2. Topic

node间沟通的方式之一是消息订阅/发布，每个消息都必须发布到相应的主题（topic），通过主题来实现在ROS计算图网络中的路由转发。

同一个主题可以有多个订阅者也可以有多个发布者。每个主题都是强类型的，不管是发布消息到主题还是从主题中订阅消息，发布者和订阅者定义的消息类型必须与主题的消息类型相匹配。

### 3. 服务

在一些特殊的场合，节点间需要点对点的高效率通信并及时获取应答，这个时候就需要用服务的方式进行交互。

服务通信过程中服务的数据类型需要用户自己定义，与消息不同，节点并不提供标准服务类型。服务类型的定义文件都是以*.srv为扩展名，并且被放在功能包的srv/文件夹下。

### 4. 配置管理器

参数服务器（parameter server）能够使数据通过关键词存储在一个系统的核心位置。通过使用参数，就能够在节点运行时动态配置节点或改变节点的工作任务。参数服务器是可通过网络访问的共享的多变量字典，节点使用此服务器来存储和检索运行时的参数。

### 5. 节点管理器

节点管理器（master）用于节点的名称注册和查找等，也负责设置节点间的通信。

由于ROS本身就是一个分布式的网络系统，所以你可以在某台计算机上运行节点管理器，在这台计算机和其他台计算机上运行节点（节点管理器需不需要高可用？？）。

ROS中提供了跟节点管理器相关的命令行工具，就是roscore，roscore命令用于启动节点管理器，这个命令会加载ROS节点管理器和其他ROS核心组件。

当ROS开始工作时，用户需要首先执行**roscore**运行一个节点管理器，以此来保证后续启动的node可以正常通信。

执行roscore的同时，系统会默认启动一个日志node（rosout），执行rosnode可以查看该node的详细信息。

```shell
# rosout启动发布了一个topic（rosout_agg），以及两个服务（get_loggers、set_logger_level）

--------------------------------------------------------------------------------
Node [/rosout]
Publications: 
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions: 
 * /rosout [unknown type]

Services: 
 * /rosout/get_loggers
 * /rosout/set_logger_level


contacting node http://ruijie-virtual-machine:39349/ ...
Pid: 31314
```

## 消息文件

ROS使用消息类型描述语言，描述node之间传递的消息，并可以在不同的编程语言（如c++、python等）书写的程序中使用此消息。

不管是ROS系统提供的标准类型消息，还是用户自定义的非标准类型消息，定义文件都是以*.msg作为扩展名。

消息类型的定义分为两个主要部分：字段的数据类型和字段的名称，简单点说就是结构体中的变量类型和变量名称。

经常用到的类型包括：
- [基本类型](http://wiki.ros.org/std_msgs/)：包括int、boolean等基础数据类型。
- [通用类型](http://wiki.ros.org/common_msgs)：高级数据类型，如四元数、传感器数据等。


# 参考

[SLAM+语音机器人DIY系列](https://www.cnblogs.com/hiram-zhang/tag/ROS%E5%85%A5%E9%97%A8/)