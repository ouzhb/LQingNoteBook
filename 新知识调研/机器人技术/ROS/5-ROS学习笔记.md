---
title: ROS学习笔记（五）
categories: "机器人" 
date: 2019-08-13
comments: false
toc: true
tags:
    - ROS
    - Moveit
    - ROS Control
---


<!--more-->

# ROS Control

ros_control是通用Robot控制框架，用来连接ROS中应用和实际（仿真）机器人，它包含一系列控制器接口、传动装置接口、硬件接口、控制器工具箱等等，可以帮助机器人应用快速落地，提高开发效率。

ros_control运行时，包括以下几个部分：

- Controller Manager：Manger管理了多个Controller，每个Controller代表一个上层ROS应用。
- Controller：Controller可以完成每个joint的控制，请求下层的硬件资源，并且提供了PID控制器，读取硬件资源接口中的状态，并发布控制命令。
- Hardware Rescource：抽象接口层
- RobotHW：硬件抽象层和硬件直接打交道，通过write和read方法来完成硬件的操作，这一层也包含关节限位、力矩转换、状态转换等功能。

下图是ros_control的数据流图：

![](https://wiki.ros.org/ros_control?action=AttachFile&do=get&target=gazebo_ros_control.png)


目前，ros_control中提供了一些现成Controllers和Hardware Interface，用户可以根据需要自己定制这些插件。

# 其他概念

## Transmissions

Transmissions就是机器人的传动系统，机器人每个需要运动的关节都需要配置相应的Transmission。

通常情况下Transmission的内容会在[URDF文件](http://wiki.ros.org/urdf/XML/Transmission)中直接定义。用户定义Transmission内容时可以定义在单独的文件中，之后include到需要的URDF文件中。

目前，ROS提供的[transmission类型](http://docs.ros.org/jade/api/transmission_interface/html/c++/group__transmission__types.html)包括：

- DifferentialTransmission
- FourBarLinkageTransmission
- SimpleTransmission

机械臂中一般只会用到SimpleTransmission。

以下是一个Joint的传动定义
```xml
<transmission name="trans_name">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_name">
        <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="joint_motor">
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```
上述定义中，注意以下两点：
- hardwareInterface：Controller和RobotHw沟通的接口，基本上和controllers的种类是对应的（参考[HardwareInterface类型](http://docs.ros.org/api/hardware_interface/html/c++/namespacehardware__interface.html)）。
- actuator：??? 不知道用来干啥的？？？

## Joint Limits

Joint Limits是RobotHW中的一块，维护一个关节限位的数据结构，包含关节速度、位置、加速度、加加速度、力矩等方面的限位，还包含安全作用的位置软限位、速度边界（k_v）和位置边界（k_p）等等。

Joint Limits可以直接定义在URDF文件中，可以通过YAML文件加载到ROS parameter server中，还可以使用joint_limits_interface在代码中定义。


## Controller manager

Controller Manager可以加载、开始运行、停止运行、卸载不同的controller，并且提供了多种工具来完成这些操作。

命令行工具：

```shell
rosrun controller_manager controller_manager <command> <controller_name>
```



# 参考

[ROS Control介绍](https://www.ncnynl.com/archives/201708/1932.html)

[Gazebo Ros Control](http://gazebosim.org/tutorials?tut=ros_control)