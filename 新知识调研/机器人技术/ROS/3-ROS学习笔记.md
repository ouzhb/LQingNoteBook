---
title: ROS学习笔记（三）
categories: "机器人" 
date: 2019-08-13
comments: false
toc: true
tags:
    - ROS
    - Moveit！
---

<!--more-->

# Moveit

## 系统结构



![](https://moveit.ros.org/assets/images/diagrams/move_group.png)

Moveit！中最重要的模块是move_group节点，该节点起到接口整合，以及提供Action、Service的作用。

关于move_group有以下几点：

1. 整合C++（move_group_interface）、Python（moveit_commander）、GUI接口（Rviz），提供Action、Service功能

2. 从ROS Param Server加载URDF、SRDF、Moveit！以及其他配置

3. move_group节点通过ROS的topic和actions控制robot，告知其位置信息、点云信息、以及其他传感器数据

## 相关问题

下面的问题是阅读[Concepts](https://moveit.ros.org/documentation/concepts/)后提炼出来的：


1. URDF文件如何定义？如何展示(gazebo、Rviz)？
2. SRDF文件包含哪些信息？详细了解SRDF文件文件内容？movit-setup-assistant 生成了哪些文件？这些文件有啥用途？

~~3. /joint_states 中的数据含义是啥？如何通过他控制机械臂动作？~~

~~4. robot_state_publisher 如何从/joint_states以及robot_description的URDF中计算TF？TF详细场景如何？~~

3. ros_control 如何工作？对hardwareInterface、actuator如何理解？
4. 如何使用control？如何定义control？control如何工作？control中涉及的数据有哪些含义？怎么生成的？

5. move_group 和 robot_state_publisher、joint_state_publisher如何交互？
6. FollowJointTrajectoryAction 是啥？如何通过他控制机器人？
7. PlanRequestAdapters和montion_planner如何工作？如何发送Request？Response是什么样的？规划路径时如何添加限制条件？
8. PlanningScene中包含哪些东西？都有什么作用？
9. 了解正/逆运动学原理？
10. 了解如何进行碰撞检测？如何应用ACM矩阵？


# 运行UR5的movit! Demo

## 开源项目

    - jontromanab/ur5_barrett_description
    - jontromanab/ur5_barrett_bringup
    - jontromanab/ur5_barrett_moveit

## 仿真过程

Step 1： roslaunch ur5_barrett_bringup ur5_barrett_table_world.launch limited:=true

Step 2： roslaunch ur5_barrett_moveit ur5_barrett_moveit_planning_execution.launch limited:=true

Step 3： 运行下面的代码控制末端执行器

```python
import sys
import PyKDL
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('control_ur5', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"  # endeffector #manipulator
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory, queue_size=30)

    current_pose = move_group.get_current_pose()
    print("================= Before Pos ================= ")
    print(current_pose)

    pose_goal = geometry_msgs.msg.Pose()  # Goal position
    rot = PyKDL.Rotation.Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                              current_pose.pose.orientation.z, current_pose.pose.orientation.w)

    # set goal rot
    print(rot.GetQuaternion())
    rot.DoRotY(-pi/2)
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*list(rot.GetQuaternion()))
    print(rot.GetQuaternion())
    # set goal position
    pose_goal.position.x = current_pose.pose.position.x
    pose_goal.position.y = current_pose.pose.position.y
    pose_goal.position.z = current_pose.pose.position.z
    # pose_goal.position.x = 0.7
    # pose_goal.position.y = 0
    # pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    move_group.stop()

    print("================= Current Pos ================= ")
    print(current_pose)
```

需要注意：

手臂长度有限，臂长大概是0.8m，离地高度大概是0.36m。如果输入的目标位置，不在这个范围内将会规划失败。指定目标点时，使用的是geometry_msgs/Pose参数，包含：
		
- geometry_msgs/Point position：世界坐标系下(x,y,z)
- geometry_msgs/Quaternion orientation：执行器转过的角度(绕x轴转动为手掌转动朝向不变，绕y，x轴转动为手掌不动，朝向变动)。

关于Geometry_Msgs.msg.Pose参考这个[帖子](https://answers.ros.org/question/265988/can-someone-explain-geometry_msgs-as-used-for-robot-arm-poses-eg/)


# 参考

[moveit！中文参考资料](https://www.ncnynl.com/archives/201610/947.html)

