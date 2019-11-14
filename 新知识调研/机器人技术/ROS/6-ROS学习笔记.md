---
title: ROS学习笔记（六）
categories: "机器人" 
date: 2019-08-21
comments: false
toc: true
tags:
    - ROS
    - tf
---

tf包的介绍，以及基本使用。

<!--more-->

# 介绍

# 示例程序

示例程序展示了二维平面内：通过TF实现的坐标跟随（turtle仿真中两只小乌龟相互跟随，[参考](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf)）。

整个示例程序包括以下node：

- turtle1_tf_broadcaster：广播turtle1的绝对坐标
- turtle2_tf_broadcaster: 广播turtle2的绝对坐标
- turtle_tf_listener: 从/tf获取turtle2到turtle1的相对变换，并控制turtle2向turtle1移动
- turtlesim_node、turtle_teleop_key：turtle套件



## 广播Node

广播node监听/turtle/pose中的位置数据，并计算其绝对坐标系。

针对其接收到的每一条消息，调用回调函数将其广播到/tf中。

```python

def handle_turtle_pose(msg, turtlename):
    """
        回调函数，负责广播turtle的相对坐标系
    :param msg:
    :param turtlename:
    :return:
    """
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta), # 将欧拉角，转换为四元数
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    """
        turtle_tf_broadcaster节点负责发布某个turtle的绝对坐标系（即相对于World坐标系的变换）。
        
        进程启动后完成以下工作：
            1. 获取传入的参数turtle；
            2. 监听/~turtle/pose的消息，并对每个消息调用回调函数handle_turtle_pose
    """
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,        # 回调函数
                     turtlename)                # 回调函数的参数
    rospy.spin()
```
[完整代码](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29)

## 监听变换

接收并缓冲系统中广播的所有坐标系，查询特定坐标系之间的变换关系。



``` python

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
    listener = tf.TransformListener()

    """
        在turtlesim_node中在模拟出第二个turtle，并指定其初始位置和位置控制topiccmd_vel
    """
        .....

    rate = rospy.Rate(10.0) # 指定以下循环的频率
    while not rospy.is_shutdown():
        try:
            # 获取/turtle2到/turtle1的坐标系变换，rospy.Time(0)表示最新的变换
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # 以下内容控制turtle2向turtle1移动，即将turtle2坐标系向turtle1移动

        ......

        rate.sleep()
```
[完整代码](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29)

## TF和时间的关系

计算frame-A在Ta相对于frame-B在Tb的时间坐标系：

```python
try:
    now = rospy.Time.now()
    past = now - rospy.Duration(5.0)
    listener.waitForTransformFull("/turtle2", now,"/turtle1", past,"/world", rospy.Duration(1.0))
    (trans, rot) = listener.lookupTransformFull("/turtle2", now,"/turtle1", past,"/world")
```


# 命令行工具

```
# 打印当前tf的拓扑到pdf文件中
rosrun tf view_frames

# 打印tf拓扑
rosrun rqt_tf_tree rqt_tf_tree

# 打印坐标系的变换关系，输出为 Translation（原点平移关系）、Rotation（转动角度，包括：四元数和欧拉角）
rosrun tf tf_echo [frame_id_1] [frame_id_2]

```


# 参考

[tf](http://wiki.ros.org/tf)

[Debugging tf problems](http://wiki.ros.org/tf/Tutorials/Debugging%20tf%20problems)

[tf2](http://wiki.ros.org/tf2)