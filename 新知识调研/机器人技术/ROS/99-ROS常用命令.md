---
title: ROS常用命令
categories: "机器人" 
date: 2019-08-09
comments: false
toc: true
tags:
    - ROS

---

ROS中使用的一些常用命令

<!--more-->

# ROS Command

## rospack

Package管理工具
```shell
# 返回package的安装目录
rospack find [package_name]
rospack depends1 [package_name] # 打印直接依赖
rospack depends [package_name]  # 打印直接和间接依赖
```

## roscd/rosls

类似于cd/ls命令，可以快速定位包路径

```shell
# 进入日志目录
roscd log
# cd/ls指定package的安装目录
roscd/rosls <package_name>
```
## rosnode

node管理工具

```shell
rosnode info <node_name>：用于输出当前节点信息。
rosnode kill <node_name>：用于杀死正在运行节点进程来结束节点的运行。
rosnode list：用于列出当前活动的节点。
rosnode machine <hostname>：用于列出指定计算机上运行的节点。
rosnode ping <node_name>：用于测试节点间的网络连通性。
rosnode cleanup：用于将无法访问节点的注册信息清除。
```
## rosmsg

msg管理工具

```shell
rosmsg show <message_type>：用于显示一条消息的字段。
rosmsg list：用于列出所有消息。
rosmsg package <package _name>：用于列出功能包的所有消息。
rosmsg packages：用于列出所有具有该消息的功能包。
rosmsg users <message_type>：用于搜索使用该消息类型的代码文件。
rosmsg md5 <message_type>：用于显示一条消息的MD5求和结果。
```
## rostopic 

topic管理工具

```shell
rostopic bw </topic_name>：用于显示主题所使用的带宽。
rostopic echo </topic_name>：用于将主题中的消息数据输出到屏幕。
rostopic find <message_type>：用于按照消息类型查找主题。
rostopic hz </topic_name>：用于显示主题的发布频率。
rostopic info </topic_name>：用于输出活动主题、发布的主题、主题订阅者和服务的信息。
rostopic list：用于列出当前活动主题的列表。
rostopic pub </topic_name> <message_type> <args>：用于通过命令行将数据发布到主题。
rostopic type </topic_name>：用于输出主题中发布的消息类型。
```

使用**rqt_graph**可以绘制topic连接的具体拓扑信息，命令为**rosrun rqt_graph rqt_graph**。

使用**rqt_plot**可以将topic信息中的值绘制为曲线，命令为**rosrun rqt_plot rqt_plot**。

## rosservice 

rosservice 管理工具

```shell
rosservice call </service_name> <args>：用于通过命令行参数调用服务。
rosservice find <service_type>：用于根据服务类型查询服务。
rosservice info </service_name>：用于输出服务信息。
rosservice list：用于列出活动服务清单。
rosservice type </service_name>：用于输出服务类型。
rosservice uri </service_name>：用于输出服务的ROSRPC URI。
rosservice type /spawn | rossrv show : 输出参数的详细信息。

```

## rosparam 

```shell
rosparam list：用于列出参数服务器中的所有参数。
rosparam get <parameter_name>：用于获取参数服务器中的参数值。
rosparam set <parameter_name> <value>：用于设置参数服务器中参数的值。
rosparam delete <parameter_name>：用于将参数从参数服务器中删除。
rosparam dump <file>：用于将参数服务器的参数保存到一个文件。
rosparam load <file>：用于从文件将参数加载到参数服务器。
```
## rosrun 

运行一个node
```shell
rosrun [package_name] [node_name]
```
# catkin

## 创建工作空间/功能包
```shell

cd ~/ && mkdir catkin_ws

#在catkin_ws目录下新建src文件夹
cd catkin_ws && mkdir src

#初始化src目录，生成的CMakeLists.txt为功能包编译配置
cd src
catkin_init_workspace

#切回catkin_ws目录，对该工作空间执行一次编译
cd ~/catkin_ws
catkin_make

#环境变量配置，使新建的catkin_ws工作空间可用
source devel/setup.bash

#在catkin_ws/src/下创建取名为hello_world的功能包，
#ROS功能包命名规范：只允许使用小写字母、数字和下划线，且首字符必须为一个小写字母。
cd ~/catkin_ws/src/
catkin_create_pkg hello_world

```

## 创建packeage

```shell
# 创建一个package名称为beginner_tutorials，依赖std_msgs、rospy、roscpp
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
catkin_create_pkg会自动在 package.xml 和  CMakeLists.txt 生成需要的依赖配置。

### 手工添加依赖
如果用户生成package后需要添加依赖，需要修改package.xml和CMakeLists.txt文件。

```shell
# 修改package.xml
#
# 在<buildtool_depend>catkin</buildtool_depend>之后添加：
#
#       <build_depend>{package_name}</build_depend>
#       <build_export_depend>{package_name}</build_export_depend>
#       <exec_depend>{package_name}</exec_depend>

# 修改CMakeLists.txt
#
# 在find_package函数中添加要依赖的package，同时找到include_directories(...)字段添加catkin_INCLUDE_DIRS环境变量
#
#       find_package(catkin REQUIRED COMPONENTS roscpp)
# 
#       include_directories(
#           #include
#           ${catkin_INCLUDE_DIRS}
)
```

### package.xml文件

package.xml文件描述了package的基本信息，并且决定在本工程依赖的其他package。一个简单的package.xml文件示例，参考下面的文档。

```xml
<?xml version="1.0"?>
<package format="2">
    <!-- 配置Package的名称、版本号、描述信息-->
    <name>beginner_tutorials</name> 
    <version>0.1.0</version>
    <description>The beginner_tutorials package</description>
    <!--Package的开源信息-->
    <maintainer email="you@yourdomain.tld">Your Name</maintainer>
    <license>BSD</license>
    <url type="website">http://wiki.ros.org/beginner_tutorials</url>
    <author email="you@yourdomain.tld">Jane Doe</author>
  
    <!--依赖配置， roscpp、rospy、std_msgs是三个最常用的依赖包-->
    <buildtool_depend>catkin</buildtool_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>rospy</build_depend>
    <build_depend>std_msgs</build_depend>
    <exec_depend>roscpp</exec_depend>
    <exec_depend>rospy</exec_depend>
    <exec_depend>std_msgs</exec_depend>
  
</package>
```

## 声明可执行文件

声明可执行文件可以在CMakeLists.txt中添加：

```shell

add_executable(file_name src/{file_name}.cpp)
target_link_libraries(file_name ${catkin_LIBRARIES})

```

## 编译

```shell
# 编译整个ws
cd ~/catkin_ws/
catkin_make 
# 编译单个package
cd ~/catkin_ws/
catkin_make -DCATKIN_WHITELIST_PACKAGES="hello_world"
```
## 激活
注册工作空间，以及空间中的所有package
```shell
cd ~/catkin_ws/
source devel/setup.bash
```
# 源码编译rospackage

```shell
cd ~/{ws}
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```
