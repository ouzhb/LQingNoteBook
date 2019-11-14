---
title: ROS学习笔记（二）
categories: "机器人" 
date: 2019-08-09
comments: false
toc: true
tags:
    - ROS
---

编写一个ROS上运行的Hello World程序。

<!--more-->

# Turtlesim

Turtlesim是ROS官方提供的一个示例，用来帮助用户理解ROS中基本概念。

## 观察node之间如何通信

参考官方[Topic的概念](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)。

示例提供了一个简易的海龟绘图功能，涉及到下面几个Package：

|Package|Publications|Pub|Subscriptions|备注|
|-----|-----|-----|-----|-----|
|turtlesim |turtlesim_node|/turtle1/color_sensor</br>/turtle1/pose</br>|/turtle1/cmd_vel|-----|
|turtlesim|turtle_teleop_key|/turtle1/cmd_vel|-----|-----|

turtlesim_node是海龟的绘图界面，通过监听cmd_vel的位置信息来进行绘图。

turtle_teleop_key是基于键盘方向键的发送程序，不断的向cmd_vel中发送geometry_msgs/Twist格式的位置数据。

用户可以通过rostopic直接向cmd_vel发送数据控制海龟运动轨迹，或查看cmd_vel的数据。

使用rqt_graph可以绘制**topic连接的具体拓扑信息**，命令为**rosrun rqt_graph rqt_graph**。

使用rqt_plot可以**将topic的数值信息绘制为连续曲线**，命令为**rosrun rqt_plot rqt_plot**。

## 调用Node服务以及修改参数
turtlesim_node 提供了一些Services用来清空画板、复制画笔等功能，这些功能可以使用rosservice来调用（[参考](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)）。

## 查看node日志使用roslaunch

[参考](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)

[参考2](https://www.cnblogs.com/hiram-zhang/p/10393328.html)


# 定义msg和srv

## 自定义msg

假设在beginner_tutorials包中定义一个新msg包含以下内容：

Step 1：在beginner_tutorials中创建msg目录，并在该目录下创建Student.msg文件，内容如下：

```
string first_name
string last_name
uint8 age
uint32 score
```
Step 2：修改package.xml文件添加message_generation依赖和message_runtime，其中编译时只用依赖message_generation，而运行时需要依赖message_runtime。

```xml
  <!--编译时依赖-->
  <build_depend>message_generation</build_depend>
  <!--运行时依赖-->
  <exec_depend>message_runtime</exec_depend>
```
Step 3： 修改CMakeLists.txt

```c
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation	# 添加编译时需要的message_generation组件
)
add_message_files(
   FILES
   Num.msg				# 指明msg定义文件的地址
 )
generate_messages(		# message_generation组件主键的依赖，这里只依赖std_msgs
   DEPENDENCIES
   std_msgs
 )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime		#CATKIN指明依赖
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
```
Step 3：编译beginner_tutorials，使用rosmsg show beginner_tutorials/Student查看生成的msg

## srv接口定义

[参考](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)

# Python实现Publisher/Subscriber 

python脚本放在Package的scripts目录下，可以无需编译直接运行：

talker.py
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    # 创建一个Topic，名称为chatter，类型为String，队列深度为10
    pub = rospy.Publisher('chatter', String, queue_size=10) 
    # 启动一个和roscore通信的node，名称为talker+xxxx
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz的数量发送数据
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # loginfo：将信息打印到屏幕，并写入到rosout和日志文件
        rospy.loginfo(hello_str)
        pub.publish(hello_str)  #发送数据
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
listener.py
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

# Python实现Service和Client

[参考](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)


# 使用rosbag记录和回放Topic

[参考](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)