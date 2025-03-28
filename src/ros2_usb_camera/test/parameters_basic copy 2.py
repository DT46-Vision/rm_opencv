#!/usr/bin/env python3
import time
import cv2
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header 
from sensor_msgs.msg import Image, CameraInfo      # 图像消息类型
from rcl_interfaces.msg import SetParametersResult  

from cv_bridge import CvBridge


camera_params = {
    'camera_id': 0,
    'image_width': 1280,
    'image_height': 720,
    'auto_exposure': 1,
    'exposure_time': 100,
    'fps': 60
}


def time_diff(last_time=[None]):
    """计算两次调用之间的时间差，单位为ns。"""
    current_time = time.time_ns()     # 获取当前时间（单位：ns）

    if last_time[0] is None:          # 如果是第一次调用，更新 last_time
        last_time[0] = current_time
        return 0.000_000_1            # 防止第一次调用时的除零错误
    
    else: # 计算时间差
        diff = current_time - last_time[0]  # 计算时间差
        last_time[0] = current_time         # 更新上次调用时间
        return diff                         # 返回时间差 ns

class ParametersBasicNode(Node):
    """
    创建一个ParametersBasicNode节点，并在初始化时输出一个话
    """

    def __init__(self, name):
        super().__init__(name)

        self.get_logger().info(f"节点已启动：{name}!")

        # 创建发布者    
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.cv_bridge = CvBridge()

        self.camera_id = camera_params.get('camera_id', 0)
        self.image_width = camera_params.get('image_width', 1280)
        self.image_height = camera_params.get('image_height', 720)
        self.auto_exposure = camera_params.get('auto_exposure', 1)
        self.exposure_time = camera_params.get('exposure_time', 100)
        self.fps = camera_params.get('fps', 60)

        # 注册 ROS2 参数
        self.declare_parameter('camera_id', self.camera_id)
        self.declare_parameter('image_width', self.image_width)
        self.declare_parameter('image_height', self.image_height)
        self.declare_parameter('fps', self.fps)

        # 添加参数变化回调
        self.add_on_set_parameters_callback(self.parameters_callback)

        # 初始化相机
        self.cap = cv2.VideoCapture(self.camera_id)
        self.set_camera_parameters() # 设置相机参数
        self.get_logger().info(f'初始化 {self.camera_id} 号相机')

        # 声明参数
        self.declare_parameter('rcl_log_level', 0)
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value
        # 设置参数
        self.get_logger().set_level(log_level)
        # 定时修改
        self.timer = self.create_timer(0.001, self.timer_callback)

    def parameters_callback(self, params):
        for param in params:

            if param.name == 'camera_id':
                self.camera_id = param.value
                self.cap = cv2.VideoCapture(self.camera_id)
                self.set_camera_parameters()
                self.get_logger().info(f'更新 相机参数: {param.name} = {param.value}')

            elif param.name == 'image_width':
                self.image_width = param.value
                self.set_camera_parameters()
                self.get_logger().info(f'更新 相机参数: {param.name} = {param.value}')

            elif param.name == 'image_height':
                self.image_height = param.value
                self.set_camera_parameters()
                self.get_logger().info(f'更新 相机参数: {param.name} = {param.value}')

            elif param.name == 'auto_exposure':
                self.auto_exposure = param.value
                self.set_camera_parameters()
                self.get_logger().info(f'更新 相机参数: {param.name} = {param.value}')

            elif param.name == 'exposure_time':
                self.exposure_time = param.value
                self.set_camera_parameters()
                self.get_logger().info(f'更新 相机参数: {param.name} = {param.value}')

            elif param.name == 'fps':
                self.fps = param.value
                self.set_camera_parameters()
                self.get_logger().info(f'更新 相机参数: {param.name} = {param.value}')

        return SetParametersResult(successful=True)  # 返回成功结果

    def set_camera_parameters(self):
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,self.auto_exposure)
        self.cao.set(cv2.CAP_PROP_EXPOSURE, self.exposure_time)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        print(f"设置的相机: {self.camera_id} 号相机")
        print(f"设置的分辨率: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)} x {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        print(f"设置的帧率: {self.cap.get(cv2.CAP_PROP_FPS)}")


    def timer_callback(self):
        """定时器回调函数"""
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value

        # 设置参数
        self.get_logger().set_level(log_level)
        print(f"========================{log_level}=============================")
        # self.get_logger().debug("我是DEBUG级别的日志，我被打印出来了!")
        # self.get_logger().info("我是INFO级别的日志，我被打印出来了!")
        # self.get_logger().warn("我是WARN级别的日志，我被打印出来了!")
        # self.get_logger().error("我是ERROR级别的日志，我被打印出来了!")
        # self.get_logger().fatal("我是FATAL级别的日志，我被打印出来了!")

        # 读取摄像头
        ret, frame = self.cap.read()
        if ret:
            if os.environ.get('DISPLAY') and os.isatty(0):  # 检查有无图形界面
                cv2.namedWindow("raw", cv2.WINDOW_NORMAL)
                cv2.imshow("raw", frame)
            
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break 

            dt = time_diff()         

            print(f"图像大小: {frame.shape}, 帧率 FPS: {1 / (dt/1e9) }, 帧时间: {dt/1e6}")    
            
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(ros_image)



def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ParametersBasicNode("parameters_basic")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
