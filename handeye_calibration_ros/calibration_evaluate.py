#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import os
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PosCmd
import pyrealsense2 as rs
from typing import Optional, Tuple, Dict, Any

T_EE_GRIPPER = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.145],
    [0.0, 0.0, 0.0, 1.0],
])

D435_DEFAULT_INTRINSICS = {
    'width': 640,
    'height': 480,
    'fx': 603.4638671875,
    'fy': 602.6591796875,
    'ppx': 320.9638671875,
    'ppy': 238.5915069580078
}

D435_DEFAULT_DISTORTION = [
    0.07273559250238304,    
    0.2619047184743646,     
    -0.0012743489202595337, 
    -0.0008457702528035415, 
    -1.4128852360755886     
]

class TransformUtils:
    
    @staticmethod
    def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        x, y, z, w = q
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        return np.array([
            [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
            [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
        ])
    
    @staticmethod
    def pose_to_transform(position, orientation) -> np.ndarray:
        if isinstance(position, (list, tuple, np.ndarray)):
            t = np.asarray(position).reshape(3, 1)
        else:
            t = np.array([position.x, position.y, position.z]).reshape(3, 1)
        
        if isinstance(orientation, (list, tuple, np.ndarray)):
            x, y, z, w = orientation
        else:
            x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        R = TransformUtils.quaternion_to_rotation_matrix([x, y, z, w])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3:] = t
        return T
    
    @staticmethod
    def transform_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
        return T @ point

class CameraManager:

    def __init__(self, 
                 custom_intrinsics: Optional[Dict] = None,
                 custom_distortion: Optional[np.ndarray] = None):
        self.pipeline = None
        self.align = None
        self.depth_scale = None
        self.intrinsics = None
        self.distortion = None
        self.undistort_enabled = False
        self.map1 = None
        self.map2 = None
        
        self._setup_camera_parameters(custom_intrinsics, custom_distortion)
        self._initialize_camera()
    
    def _setup_camera_parameters(self, custom_intrinsics, custom_distortion):
        if custom_intrinsics is not None and custom_distortion is not None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = custom_intrinsics['width']
            self.intrinsics.height = custom_intrinsics['height']
            self.intrinsics.fx = custom_intrinsics['fx']
            self.intrinsics.fy = custom_intrinsics['fy']
            self.intrinsics.ppx = custom_intrinsics['ppx']
            self.intrinsics.ppy = custom_intrinsics['ppy']
            self.intrinsics.model = rs.distortion.brown_conrady
            self.distortion = np.array(custom_distortion, dtype=np.float64)
            self.undistort_enabled = True
            print("使用用户提供的相机内参和畸变参数")
        else:
            print("将使用相机默认内参")
    
    def _initialize_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        
        profile = self.pipeline.start(config)
        
        # 获取深度比例
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # 创建对齐对象
        self.align = rs.align(rs.stream.color)
        
        if self.intrinsics is None:
            color_profile = profile.get_stream(rs.stream.color)
            self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
            print("从相机获取内参:")
            print(f"  Width: {self.intrinsics.width}, Height: {self.intrinsics.height}")
            print(f"  Fx: {self.intrinsics.fx}, Fy: {self.intrinsics.fy}")
            print(f"  Ppx: {self.intrinsics.ppx}, Ppy: {self.intrinsics.ppy}")
            print(f"  Distortion model: {self.intrinsics.model}")
            print(f"  Distortion coeffs: {self.intrinsics.coeffs}")

            # 检查是否需要去畸变
            if any(c != 0 for c in self.intrinsics.coeffs):
                self.distortion = np.array(self.intrinsics.coeffs, dtype=np.float64)
                self.undistort_enabled = True
        
        # 去畸变映射
        if self.undistort_enabled:
            self._setup_undistort_maps()
        
        # 等待相机稳定
        for _ in range(10):
            self.pipeline.wait_for_frames()
    
    def _setup_undistort_maps(self):
        camera_matrix = np.array([
            [self.intrinsics.fx, 0, self.intrinsics.ppx],
            [0, self.intrinsics.fy, self.intrinsics.ppy],
            [0, 0, 1]
        ])
        
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            camera_matrix, self.distortion,
            (self.intrinsics.width, self.intrinsics.height), 1,
            (self.intrinsics.width, self.intrinsics.height))
        
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            camera_matrix, self.distortion, None,
            new_camera_matrix,
            (self.intrinsics.width, self.intrinsics.height),
            cv2.CV_32FC1)
        
        print("图像去畸变已启用")
    
    def get_frames(self) -> Tuple[Optional[rs.depth_frame], Optional[rs.video_frame]]:
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        return aligned_frames.get_depth_frame(), aligned_frames.get_color_frame()
    
    def pixel_to_3d(self, depth_frame: rs.depth_frame, u: int, v: int) -> Optional[np.ndarray]:
        depth = depth_frame.get_distance(u, v)
        if depth <= 0:
            return None
        
        point = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [u, v], depth
        )
        return np.array([point[0], point[1], point[2], 1.0])
    
    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        if self.undistort_enabled and self.map1 is not None and self.map2 is not None:
            return cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
        return image
    
    def release(self):
        if self.pipeline:
            self.pipeline.stop()

class HandEyeCalibrator:
    
    def __init__(self, mode: str = "eye_in_hand"):
        self.mode = mode
        self.T_ee_cam = np.eye(4)   
        self.T_base_cam = np.eye(4) 
        self.T_base_ee = np.eye(4)  
    
    def load_calibration(self, filepath: str) -> bool:
        if not os.path.isfile(filepath):
            print(f"标定文件不存在: {filepath}")
            return False
        
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            if self.mode == "eye_in_hand":
                self.T_ee_cam = TransformUtils.pose_to_transform(
                    data["position"], data["orientation"])
            elif self.mode == "eye_to_hand":
                self.T_base_cam = TransformUtils.pose_to_transform(
                    data["position"], data["orientation"])
            else:
                print(f"未知的标定模式: {self.mode}")
                return False
            
            print(f"标定数据已加载: {filepath}")
            return True
            
        except Exception as e:
            print(f"加载标定文件时出错: {e}")
            return False
    
    def update_robot_pose(self, pose_msg: PoseStamped):
        self.T_base_ee = TransformUtils.pose_to_transform(
            pose_msg.pose.position, pose_msg.pose.orientation)
    
    def pixel_to_base(self, point_cam: np.ndarray) -> np.ndarray:
        if self.mode == "eye_in_hand":
            return TransformUtils.transform_point(
                self.T_base_ee @ self.T_ee_cam, point_cam)
        else:
            return TransformUtils.transform_point(self.T_base_cam, point_cam)
    
    def get_grasp_pose(self, point_base: np.ndarray) -> Tuple[np.ndarray, Tuple[float, float, float]]:
        position = point_base[:3].copy()
        
        if self.mode == "eye_in_hand":
            position[2] += T_EE_GRIPPER[2, 3]  
            orientation = (np.pi, 0.0, np.pi)  
        else:
            position[2] += T_EE_GRIPPER[2, 3]
            orientation = (np.pi, 0.0, np.pi)
        
        return position, orientation

class ArmController:
    
    def __init__(self, node: Node):
        self.node = node
        self.pose_publisher = node.create_publisher(PosCmd, '/pos_cmd', 10)
    
    def send_grasp_command(self, position: np.ndarray, orientation: Tuple[float, float, float]):
        """发送抓取指令"""
        msg = PosCmd()
        msg.x, msg.y, msg.z = position
        msg.roll, msg.pitch, msg.yaw = orientation
        msg.gripper = 0.0  
        self.pose_publisher.publish(msg)

class CalibrationEvalNode(Node):

    def __init__(self):
        super().__init__("calibration_evaluate")
        
        self.declare_parameter("mode", "eye_in_hand")
        self.declare_parameter("transform_file", "")
        
        mode = self.get_parameter("mode").value
        transform_file = self.get_parameter("transform_file").value

        self.window_name = "Color View"
        self.window_initialized = False
        self.clicked_pixel = None

        self.camera = CameraManager()
        self.calibrator = HandEyeCalibrator(mode)
        self.arm_controller = ArmController(self)
        
        if not self.calibrator.load_calibration(transform_file):
            self.get_logger().error("标定数据加载失败，节点将退出")
            raise RuntimeError("标定数据加载失败")
        
        self.create_subscription(
            PoseStamped, "/end_pose_stamped", 
            self._robot_pose_callback, 1)

    def _robot_pose_callback(self, msg: PoseStamped):
        """机器人位姿回调"""
        self.calibrator.update_robot_pose(msg)
    
    def _mouse_callback(self, event, x, y, flags, param):
        """鼠标点击回调"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_pixel = (x, y)
            self.get_logger().info(f"点击像素: ({x}, {y})")
    
    def _ensure_window_with_callback(self):
        """确保窗口存在且已设置鼠标回调"""
        try:
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.window_initialized = False
        except:
            self.window_initialized = False
        
        if not self.window_initialized:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self.window_name, self._mouse_callback)
            self.window_initialized = True

    def _process_click(self, depth_frame: rs.depth_frame, color_image: np.ndarray):
        """处理鼠标点击事件"""
        if self.clicked_pixel is None:
            return
        
        u, v = self.clicked_pixel
        
        point_cam = self.camera.pixel_to_3d(depth_frame, u, v)
        if point_cam is None:
            print(f"Warning: depth is invalid at ({u},{v})")
            self.clicked_pixel = None 
            return
        
        point_base = self.calibrator.pixel_to_base(point_cam)
        
        # self.get_logger().info(
        #     f"像素({u},{v}) → 相机点({point_cam[0]:.3f},{point_cam[1]:.3f},{point_cam[2]:.3f})m → "
        #     f"基座Z={point_base[2]:.3f}m"
        # )
        
        cv2.drawMarker(color_image, (u, v), (0, 0, 255),
                      markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)
        
        position, orientation = self.calibrator.get_grasp_pose(point_base)
        self.arm_controller.send_grasp_command(position, orientation)
        
        self.clicked_pixel = None
    
    def run(self):
        timer = self.create_timer(0.1, self._main_loop)  # 10Hz
        rclpy.spin(self)
    
    def _main_loop(self):
        depth_frame, color_frame = self.camera.get_frames()
        if not depth_frame or not color_frame:
            return
        
        color_image = np.asanyarray(color_frame.get_data())
        color_image = self.camera.undistort_image(color_image)
        
        self._ensure_window_with_callback()
        self._process_click(depth_frame, color_image)
        
        cv2.imshow(self.window_name, color_image)
        cv2.waitKey(1)
    
    def destroy_node(self):
        self.camera.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CalibrationEvalNode()
        node.run()
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
