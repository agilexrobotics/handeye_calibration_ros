#!/usr/bin/env python3
# -*-coding:utf8-*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
import tf_transformations
from scipy.spatial.transform import Rotation
import cv2
import numpy as np
import json
import datetime
import sys
import termios
import tty
import time
import os

def clear_input_buffer():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def create_dir(dir=""):
    if dir == "" or dir == "/":
        exit(-1)
    if not os.path.exists(dir) or not os.path.isdir(dir):
        try:
            if os.path.exists(dir):
                os.remove(dir)
            os.makedirs(dir)
        except IOError:
            print("cannot create directory: " + dir)
            exit(-2)

def create_pose_from_martix(translation_matrix, rotation_matrix) -> Pose:
    rotation_matrix_4x4 = rotation_matrix_4x4 = np.eye(4)
    rotation_matrix_4x4[:3, :3] = rotation_matrix
    quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix_4x4)

    pose = Pose()
    pose.position.x = translation_matrix[0][0]
    pose.position.y = translation_matrix[1][0]
    pose.position.z = translation_matrix[2][0]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

class PoseMartix():
    def __init__(self, pose_input: Pose, check_mode=False, mode='eye_in_hand'):
        position = pose_input.position
        orientation = pose_input.orientation
        self.position_list = [position.x, position.y, position.z]
        self.orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.rpy_list = [tf_transformations.euler_from_quaternion(self.orientation_list)]
        self.translation_matrix = np.array(self.position_list)
        self.rotation_matrix = Rotation.from_quat(self.orientation_list).as_matrix()
        # self.rotation_matrix = tf_transformations.quaternion_matrix(orientation)[:3, :3]
        if check_mode and mode == 'eye_to_hand':
            T = np.eye(4)
            T[:3, :3] = self.rotation_matrix
            T[:3, 3] = self.translation_matrix
            T_inv = np.linalg.inv(T)
            self.rotation_matrix = T_inv[:3, :3]
            self.translation_matrix = T_inv[:3, 3]

class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration")

        # Declare parameters
        self.declare_parameter('mode', 'eye_in_hand')
        self.declare_parameter('min_num', 10)
        self.declare_parameter('piper_topic', '/piper_ctrl_node/end_pose')
        self.declare_parameter('marker_topic', '/aruco_single/pose')
        self.declare_parameter('result_save_path', './result')
        # "/aruco_single/pose", "/piper_ctrl_node/end_pose"

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.min_num = self.get_parameter('min_num').get_parameter_value().integer_value
        self.piper_topic = self.get_parameter('piper_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.result_save_path = self.get_parameter('result_save_path').get_parameter_value().string_value

        print(f"mode: {self.mode}")
        print(f"min_num: {self.min_num}")
        print(f"piper_topic: {self.piper_topic}")
        print(f"marker_topic: {self.marker_topic}")
        print(f"result_save_path: {self.result_save_path}")

        self.piper_poses = []
        self.marker_poses = []

    def process_handeye(self):
        R_gripper2base = [data.rotation_matrix for data in self.piper_poses]
        t_gripper2base = [data.translation_matrix for data in self.piper_poses]
        R_target2cam = [data.rotation_matrix for data in self.marker_poses]
        t_target2cam = [data.translation_matrix for data in self.marker_poses]
        ret_R, ret_t = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)
        return PoseMartix(create_pose_from_martix(ret_t, ret_R))

    def run(self):
        if self.mode != "eye_in_hand" and self.mode != "eye_to_hand":
            raise ValueError("'mode' 必须为 eye_in_hand 或 eye_to_hand ")
        if self.min_num < 5:
            raise ValueError("最少采集次数不得小于5次")
        self.get_poses()
        result_pose = self.process_handeye()
        result = dict(
            position = result_pose.position_list,
            orientation = result_pose.orientation_list,
            rpy = result_pose.rpy_list
        )

        print("标定结果:")
        print("")
        print(json.dumps(result, indent=4))
        print("")
        create_dir(self.result_save_path)
        filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        with open(f"{self.result_save_path}/{filename}_calibration.json", 'w+') as json_file:
            json.dump(result, json_file, indent=4)
            print(f"writing to {self.result_save_path}/{filename}_calibration.json")

    def get_poses(self):
        count = 1
        while True:
            print(f"\n------- 第{count}次采集 ------")
            if count == 1:
                menu_str = "请输入(Enter-采集):"
            elif count <= self.min_num:
                menu_str = "请输入(Enter-采集, d-回退):"
            else:
                menu_str = "请输入(Enter-采集, d-回退, q-计算并退出):"
            try:
                clear_input_buffer()
                user_input = input(menu_str+" ")
                if user_input == '':
                    count += 1
                    marker_pose_raw, piper_pose_raw = self.subscribe_message()
                    piper_pose = PoseMartix(piper_pose_raw, True, self.mode)
                    marker_pose = PoseMartix(marker_pose_raw.pose, False, self.mode)
                    print("---")
                    print(f"piper: {piper_pose.position_list}")
                    print(f"marker: {marker_pose.position_list}")
                    self.piper_poses.append(piper_pose)
                    self.marker_poses.append(marker_pose)
                elif count > 1 and user_input == 'd':
                    count -= 1
                    self.piper_poses.pop()
                    self.marker_poses.pop()
                elif user_input == 'c':
                    print("exit")
                    exit(0)
                elif count > self.min_num and user_input == 'q':
                    print("开始计算标定结果")
                    break
                else:
                    print("无效输入")
            except Exception as e:  # Exception as e
                #exit(0)
                #pass
                #print(e)
                print("\n程序异常或中断退出")
                exit(-1)	# -3

    def callback(self, msg):
        self.get_msg=True
        self.call_msg = msg

    def subscribe_message(self):
        sys.stdout.write("wait marker data... ")
        sys.stdout.flush()
        marker_pose_raw = self.subcribe_one_message(self.marker_topic, PoseStamped)
        sys.stdout.write("[ok]\n")

        sys.stdout.write("wait piper data... ")
        sys.stdout.flush()
        piper_pose_raw = self.subcribe_one_message(self.piper_topic, Pose)
        sys.stdout.write("[ok]\n")

        return marker_pose_raw, piper_pose_raw

    def subcribe_one_message(self, topic, msg_type):
        self.get_msg=False
        sub = self.create_subscription(msg_type, topic, self.callback, 1)
        while not self.get_msg:
            rclpy.spin_once(self)
            time.sleep(0.05) 
        msg = self.call_msg
        self.destroy_subscription(sub)
        return msg

def main(args=None):
    rclpy.init(args=args)
    calibration_node = HandEyeCalibrationNode()
    calibration_node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
