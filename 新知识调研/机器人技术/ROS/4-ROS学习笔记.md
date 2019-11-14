---
title: ROS学习笔记（四）
categories: "机器人" 
date: 2019-08-13
comments: false
toc: true
tags:
    - ROS
    - URDF
---
理解URDF、XACRO文件的内容

<!--more-->


# URDF

URDF(Unified Robot Description Format), 是一种特殊的xml文件格式，绝大多数机器人应当提供xxx_description安装包，可以通过apt-get直接安装。

xacro文件是一种提供了一些更为高级编辑方式的宏文件，通过下面的命令可以将xacro解析出urdf。

```shell
# 生成urdf，一个xacro中可能include了多个子文件，但是最终只会生成一个urdf文件
rosrun xacro xacro.py /opt/ros/melodic/share/franka_description/robots/panda_arm_hand.urdf.xacro > panda_arm_hand.urdf
```
## URDF文件格式

URDF文件主要包括：

- link：表示Robot的一个关节
    - visual：描述形状，可以是简单形状，或者一个stl格式的文件
    - collision：描述link的碰撞体积
- joint：表示link之间的相对位置关系，包括：
    - parent
    - child
    - xyz：原点的平移向量(父链 ——> 子链)，单位为米
    - rpy: child和parent坐标系的旋转矢量（转动顺序为：x -> y -> z）


## Joint 类型

参考URDF中[joint标签](http://wiki.ros.org/urdf/XML/joint)的定义，Joint有六种类型：

- revolute ：绕固定轴旋转有角度上下限
- continuous ：绕固定轴旋转没有角度限制
- prismatic ：滑动接头，沿轴线滑动，并具有由上限和下限指定的有限范围
- fixed ：固定连接
- floating ：该关节允许所有6个自由度的运动。
- planar ：该关节允许在垂直于轴的平面中运动。

上述六种类型中，除fixed和floating以外，其余四种类型的运动轴均通过axis标签（默认情况下是(1,0,0)）定义。

**[sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html)**描述revolute和prismatic描述机器人的姿态，格式为：

```
Header header

string[] name           # 所有joint名称序列
float64[] position      # joint的运动幅度，是角度或者位移
float64[] velocity      # 当前的速度
float64[] effort        # 关节上施加的力
```

## XACRO

XACRO文件和URDF实质上是等价的， 但是提供了一些更高级的方式来组织编辑机器人描述，包括：

- 复用URDF段落
- 嵌入简单的计算
- include多个xacro文件



## 可视化化URDF

查看urdf文件, 可以使用urdf_tutorial包:

```shell
roslaunch urdf_tutorial display.launch model:=/home/ruijie/Desktop/panda_arm_hand.urdf

roslaunch urdf_tutorial display.launch model:=/home/ruijie/Desktop/panda_arm_hand.urdf gui:=true

roslaunch urdf_tutorial xacrodisplay.launch model:=/opt/ros/melodic/share/franka_description/robots/panda_arm_hand.urdf.xacro

```

上述命令中display.launch包含以下内容：

```xml
<launch>

  <!-- 输入参数包括模型文件、是否使用GUI、rvizconfig配置-->
  <arg name="model" default="$(find urdf_tutorial)/urdf/01-myfirst.urdf"/>
  <arg name="gui" default="true" />
  <arg name="rvizconfig" default="$(find urdf_tutorial)/rviz/urdf.rviz" />

  <!-- 将urdf上传到robot_description中 -->
  <param name="robot_description" command="$(find xacro)/xacro.py $(arg model)" />
  <param name="use_gui" value="$(arg gui)"/>

  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />

</launch>
```

上述launch启动三个node：

- joint_state_publisher： 一个发布JointState数据的工具。joint_state_publisher读取robot_description参数，查找所有非固定关节并发布一个JointState消息，其中定义了所有这些关节。joint_state_publisher的输出是/`joint_states` ,输入可以是GUI、其他Topic等。
- robot_state_publisher：订阅/joint_states，根据其中的JointState数据以及Robot的关节参数，更新/tf和/tf_static树中的坐标变换关系。
- rviz：可视化显示

## 组合机器人

UR + Barrett：[参考文章](https://www.2cto.com/kf/201608/541696.html)中的xacro，没有成功。

Git上有不少UR + Barrett的例子，如[ur5_barrett_moveit](https://github.com/jontromanab/ur5_barrett_moveit)、[ur5_barrett_description](https://github.com/jontromanab/ur5_barrett_description)

# SDF

SDF格式是一种从世界级到机器人级的所有内容的完整描述，能够描述Gazebo环境中的Robot以及其他内容。

SDF可以认为是URDF格式的扩展，提供了以下能力：

- 在URDF基础上描述了物体的质量、惯性等更丰富的力学性质；
- 通过插件的方式可以描述摄像机、IMU等Gazebo支持的传感器；

Gazebo提供了一些开源SDF格式的模型，用户可以在线下载或者[离线下载](https://bitbucket.org/osrf/gazebo_models/downloads/)后解压到.gazebo/models目录下。

Gazebo同时还支持一种.world格式的模型文件，这个文件同样支持SDF格式的语法，同SDF文件本质上没有差异。

一个空白的world文件，只定义了灯光、背景、视角等信息等信息
```xml

<?xml version="1.0" ?>
<sdf version="1.4">
  <!-- We use a custom world for the rrbot so that the camera angle is launched correctly -->

  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Focus camera on tall pendulum -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>

```



## SDF格式和URDF格式的差异

SDF比URDF多出了inertial标签和gazebo标签：

- inertial标签：定义在link标签中，用来描述物体物理属性（如，质心位置、质量、惯性矩阵）
- gazebo标签：gazebo标签可能定义在robot、link、joint等各种地方，其内容一般是Gazebo的扩展属性，通常一般使用.gazebo文件额外定义gazebo标签中的内容。

通过下面的launch文件，可以将World和URDF文件加载到Gazebo中：

```xml
<launch>

  <!-- Gazebo的启动参数，通常是paused、use_sim_time、gui、headless、debug -->

  <!-- 运行empty_world.launch可以加载，world_name指定的world文件 -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find rrbot_gazebo)/worlds/rrbot.world"/>
    <!--
        Gazebo的启动参数
    -->
  </include>

  <!-- 将URDF文件加载到robot_description -->
  <param name="robot_description"
    command="$(find xacro)/xacro --inorder '$(find rrbot_description)/urdf/rrbot.xacro'" />

  <!-- 
    spawn_model是gazebo_ros中提供的Python脚本可以解析URDF格式的文件，并在软件中展示。
    
    参考：rosrun gazebo_ros spawn_model -h 会打印该命令的详细用法。

  -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-urdf -model rrbot -param robot_description"/>

  <!-- ros_control rrbot launch file -->
  <!--include file="$(find rrbot_control)/launch/rrbot_control.launch" /-->

</launch>
```
## Gazebo插件

Gazebo包括三种类型的插件：

- ModelPlugins
- SensorPlugins
- VisualPlugins

需要注意SensorPlugins需要附加到link标签中进行定义，而不能单独定义。

```xml
<robot>
  <!-- ... robot description ... -->
  <link name="sensor_link">
  <!-- ... link description ... -->
  </link>
  <gazebo reference="sensor_link">
    <sensor type="camera" name="camera1">
      <!-- 传感器参数 -->
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <!-- 插件参数 -->
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

# 参考

[moveit！中文参考资料](https://www.ncnynl.com/archives/201610/947.html)

[Tutorial: Using roslaunch to start Gazebo, world files and URDF models](http://gazebosim.org/tutorials?tut=ros_roslaunch)

