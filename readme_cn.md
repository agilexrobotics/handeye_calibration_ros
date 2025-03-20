# 基于ROS2的手眼标定程序包

|UBUNTU|ROS|PYTHON|OPENCV|STATE|
|---|---|---|---|---|
|![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![python](https://img.shields.io/badge/python-3.10-blue.svg)|![opencv](https://img.shields.io/badge/opencv-4.5.0-blue.svg)|![Pass](https://img.shields.io/badge/Pass-green.svg)|

## 手眼标定
> 通过采集多组机械臂末端位姿和相机识别的标定板位姿作为输入，获取两种标定结果：
- 眼在手上：机械臂末端和相机之间的坐标变换矩阵。
- 眼在手外：机械臂底部和相机之间的坐标变换矩阵。

## 1、 安装方法
### 1. 相关依赖
```
$ sudo apt-get install ros-$ROS_DISTRO-tf-transformations
```

### 2. 相关驱动
> 测试使用是的`Original ArUco`字典的标定板和`piper`机械臂。
- [在线生成标定板]( https://chev.me/arucogen/)
- [相机识别程序](https://github.com/pal-robotics/aruco_ros/tree/humble-devel)
- [piper机械臂程序](https://github.com/agilexrobotics/piper_ros/tree/humble)


### 3. 安装编译
```
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src
$ git clone 
$ cd ..
$ colcon build --symlink-install
```

## 2、 使用
### 1.  启动相机节点
> 根据实际启动

### 2. 启动机械臂
```
$ ros2 launch piper start_single_piper.launch.py can_port:=can0
```
> 注意机械臂需要进入**示教模式**        

### 3.  启动相机识别
```
$ ros2 launch aruco_ros single.launch
```
> 需要提前在launch中更改marker的size和id，以及图像话题和frame

### 4. 启动相机标定程序
> 建议采集时缓慢移动机械臂，并且需要多采集角度变换。
使用方法：`enter`采集一组数据，`d`删除一组数据，`q`计算标定结果并打印退出，`c`直接退出。

- 眼在手上
```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_in_hand
```
> 采集说明：摄像头固定在机械臂末端，标定码平放在桌上。操作机械臂让摄像头能够识别出桌上的标定码。

- 眼在手外
```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_to_hand
```
> 采集说明：需要标定摄像头固定在某个位置，标定码固定在机械臂末端。操作机械臂让摄像头能够识别出机械臂末端的标定码。


- 相关参数

|参数|类型|默认值|说明|
|---|---|---|---|
|mode|string|eye_in_hand|手眼标定模式|
|min_num|int|10|最少采集次数|
|piper_topic|string|piper_ctrl_node/end_pose|机械臂末端位姿话题（geometry_msgs/Pose）|
|marker_topic|string|aruco_single/pose|摄像头识别标定板位姿话题（geometry_msgs/PoseStamped）|
