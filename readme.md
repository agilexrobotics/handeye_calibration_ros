# Hand-Eye Calibration Package Based on ROS2

|UBUNTU|ROS|PYTHON|OPENCV|STATE|
|---|---|---|---|---|
|![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![python](https://img.shields.io/badge/python-3.10-blue.svg)|![opencv](https://img.shields.io/badge/opencv-4.5.0-blue.svg)|![Pass](https://img.shields.io/badge/Pass-green.svg)|

## Hand-Eye Calibration

By collecting multiple sets of end-effector poses from the robotic arm and camera-recognized calibration board poses as input, two types of calibration results can be obtained:
- Eye in hand: The transformation matrix between the robotic arm's end-effector and the camera.
- Eye to hand: The transformation matrix between the robotic arm's base and the camera.

## 1. Installation
### 1.1 Dependencies
```
$ sudo apt install libopencv-dev python3-opencv
$ sudo apt-get install ros-$ROS_DISTRO-tf-transformations
```

### 1.2 Drivers
For testing, we used the `Original ArUco` dictionary calibration board and the `piper` robotic arm.

- [Online Calibration Board Generator]( https://chev.me/arucogen/)
- [Camera Recognition](https://github.com/pal-robotics/aruco_ros/tree/humble-devel)
- [Piper Robotic Arm Program](https://github.com/agilexrobotics/piper_ros/tree/humble)


### 1.3 Build from source
```
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src
$ git clone 
$ cd ..
$ colcon build --symlink-install
```

## 2. Directly run
### 2.1 Start Camera
- Start the camera node according to the actual setup.

### 2.2 Start Robot Arm
```
$ ros2 launch piper start_single_piper.launch.py can_port:=can0
```
- The robotic arm must be in **teaching mode**.

### 2.3 Start Camera Recognition
```
$ ros2 launch aruco_ros single.launch
```
- Need to correct the **marker's size** and **id**, as well as the **image topic** and **frame_id**.

### 2.4 Start Camera Calibration 
When collecting data, it is recommended to move the robot slowly and collect more angle information.

Usage: `enter` collects a set of data, `d` deletes a set of data, `q` calculates the calibration result and prints it out,  `c` exit.

#### 2.4.1 Eye in Hand
```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_in_hand
```
- Collection Instructions: The camera is fixed at the end of the robotic arm, and the calibration board is placed flat on the table. Operate the robotic arm to allow the camera to recognize the calibration board on the table.

#### 2.4.2 Eye to Hand

```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_to_hand
```
- Collection Instructions: The camera is fixed at a specific position, and the calibration board is fixed at the end of the robotic arm. Operate the robotic arm to allow the camera to recognize the calibration board at the arm's end.


#### 2.4.3 Parameters

|param|type|default|Description|
|---|---|---|---|
|mode|string|eye_in_hand|hand-eye calibration mode|
|min_num|int|10|minimum number of data sets|
|piper_topic|string|piper_ctrl_node/end_pose|robotic arm's end-effector(geometry_msgs/Pose)|
|marker_topic|string|aruco_single/pose|camera-recognized calibration board pose topic（geometry_msgs/PoseStamped）|
