UR机械臂：

- 坐标系：
	- 基座坐标系：右手坐标系，中心为底座中心，z轴正方向为机座关节朝上的方向。
	- 工具坐标系：右手坐标系，中心为工具圆盘中心，z轴正方向为工具圆盘朝外的方向。
	- TCP：Tool Central Point，通常指夹具的中心点。
	- CSYS：工作基准坐标系。
	
默认情况下，UR5的基座坐标系为CSYS，工具坐标系中心作为TCP（可以通过软件修改）。

UR5可以通过以下三种命令使TCP移动到指定位姿态：

- 移动指令
	- movej：参数是相应关节终止位置的弧度。该指令能够使TCP快速移动
	- movel：参数是（x，y，z，rx，ry，rz），表示TCP的终止位姿。该指令能够使TCP在空间内线性移动。
	- movep：参数同movel。该指令能够使TCP在曲线上匀速运动。
	 

- 软件
	
	- Polyscope：图形API，具有示教编程功能。
	- URScript：脚本程序运行在独立PC机上，通过TCP和URControl通信。
	- TCP/IP：可以获取机器人实时位姿，发送控制指令。上述三种方式，实际上就是基于TCP/IP通信接口。
	
- 其他资源
	
	- [UR机器人ROS工具包](https://github.com/ros-industrial/universal_robot)

