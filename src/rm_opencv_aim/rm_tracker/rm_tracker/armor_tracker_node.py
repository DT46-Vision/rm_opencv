import time                             # 用于时间计算
import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import String, Header # 字符串消息类型和头部消息类型
from sensor_msgs.msg import Image       # 图像消息类型
from rm_interfaces.msg import ArmorsCppMsg, ArmorCppInfo, ArmorTracking, Decision  # 导入自定义消息类型
from rm_tracker.armor_tracker import select_tracking_armor, pixel_to_angle_and_deep, Armor
from rcl_interfaces.msg import SetParametersResult  # 导入 SetParametersResult 消息类型
from rm_tracker.Kalman import KalmanFilter
    
class ArmorTrackerNode(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化

        self.sub_armors = self.create_subscription(
            ArmorsCppMsg, '/detector/armors_info', self.listener_callback_armors, 10)  # 订阅装甲板信息

        self.sub_cam = self.create_subscription(
            Image, 'image_raw', self.listener_callback_cam, 10)

        self.sub_serial = self.create_subscription(
            Decision, '/nav/decision', self.listener_callback_serial, 10)  # 订阅串口数据

        self.pic_width = 1024       # 随便初始化一个图像宽度
        self.fov = 72
        self.center_last = (0, 0)   # 默认初始化中心点坐标为(0, 0)
        self.height_last = 0        # 初始化armor高度为0

        self.use_kf = False         # 是否使用卡尔曼滤波
        self.kf_cx = KalmanFilter()
        self.kf_cy = KalmanFilter()
        self.kf_h = KalmanFilter()

        self.lost = 0               # 初始化丢失帧数
        self.frame_add = 35         # 初始化补帧数
        self.reflection_height_tol = 20
        self.reflection_cx_tol = 10
        self.offset_yaw = 0.0
        self.offset_pitch = 0.0
        self.deep_buff = 0.000_001
        self.yaw_ratio = 1.00

        self.pub_tracker = self.create_publisher(ArmorTracking, '/tracker/target', 10) # 创建发布者/tracker/target

        self.tracking_color = 1     # 1蓝色表示, 0表示红色, 现初始化为蓝色
        self.follow_decision = 0
        self.tracking_armor = []    # 初始化追踪装甲板为列表

        self.declare_parameter('use_kf', self.use_kf)
        self.declare_parameter('frame_add', self.frame_add)
        self.declare_parameter('follow_decision', self.follow_decision)
        self.declare_parameter('tracking_color', self.tracking_color)
        self.declare_parameter('reflection_height_tol', self.reflection_height_tol)
        self.declare_parameter('reflection_cx_tol', self.reflection_cx_tol)
        self.declare_parameter('offset_yaw', self.offset_yaw)
        self.declare_parameter('offset_pitch', self.offset_pitch)
        self.declare_parameter('deep_buff', self.deep_buff)
        self.declare_parameter('yaw_ratio', self.yaw_ratio)
        self.add_on_set_parameters_callback(self.param_callback)
        self.get_logger().info('Armor Tracker Node has started.')

    def param_callback(self, params):
        for param in params:
            if param.name == 'use_kf':
                self.use_kf = param.value
            if param.name == 'frame_add':
                self.frame_add = param.value
            if param.name == 'tracking_color':
                self.tracking_color = param.value
            if param.name == 'follow_decision':
                self.follow_decision = param.value
            if param.name == 'reflection_height_tol':
                self.reflection_height_tol = param.value
            if param.name == 'reflection_cx_tol':
                self.reflection_cx_tol = param.value
            if param.name == 'offset_yaw':
                self.offset_yaw = param.value
            if param.name == 'offset_pitch':
                self.offset_pitch = param.value
            if param.name == 'deep_buff':
                self.deep_buff = param.value
            if param.name == 'yaw_ratio':
                self.yaw_ratio = param.value

        return SetParametersResult(successful=True)

    def listener_callback_cam(self, data):
        if self.pic_width != data.width:
            self.pic_width = data.width

    def listener_callback_armors(self, msg):
        try:
            # 计算时间差（基于 ROS 时间）
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9 if hasattr(self, 'last_time') else 0.1
            self.last_time = current_time
            
            # 更新卡尔曼滤波器时间步长
            self.kf_cx.dt = dt
            self.kf_cy.dt = dt
            self.kf_h.dt = dt

            # 选择要跟踪的装甲板
            self.tracking_armor = select_tracking_armor(msg, self.tracking_color, self.reflection_height_tol, self.reflection_cx_tol)

            # 日志记录
            if self.tracking_color == 0:
                color_str = '红色'
            elif self.tracking_color == 1:
                color_str = '蓝色'
            elif self.tracking_color == -1:
                color_str = '未知'
                self.get_logger().info("不需要追踪装甲板")
            else:
                color_str = '未知'
                self.get_logger().warn("颜色格式错误")

            # 检查是否有装甲板
            if not self.tracking_armor:  # 空列表表示未检测到装甲板
                self.get_logger().info("未检测到目标装甲板，返回默认值")
                if self.use_kf:
                    self.lost += 1
                    if self.lost <= self.frame_add:
                        self.kf_cx.predict()
                        self.kf_cy.predict()
                        self.kf_h.predict()
                        self.center_last = (self.kf_cx.get_state(), self.kf_cy.get_state())
                        self.height_last = self.kf_h.get_state()
                    else:
                        self.center_last = (0, 0)
                        self.height_last = 0
                else:
                    self.center_last = (0, 0)
                    self.height_last = 0
            else:
                # 从列表中提取第一个装甲板
                armor = self.tracking_armor[0]
                self.get_logger().info(f"得到需要追踪的 {color_str} 装甲板 id:{armor.class_id}, height:{armor.height}")
                
                self.center_last = armor.center
                self.height_last = armor.height
                if self.use_kf:
                    self.lost = 0
                    self.kf_cx.predict()
                    self.kf_cy.predict()
                    self.kf_h.predict()
                    self.kf_cx.update(self.center_last[0])
                    self.kf_cy.update(self.center_last[1])
                    self.kf_h.update(self.height_last)
                    self.center_last = (self.kf_cx.get_state(), self.kf_cy.get_state())
                    self.height_last = self.kf_h.get_state()
                    self.get_logger().info(f"预测的 cx: {self.center_last[0]}, cy: {self.center_last[1]}, h: {self.height_last}")

            # 计算角度和深度
            yaw, pitch, deep = pixel_to_angle_and_deep(self.height_last, self.center_last, self.fov, self.pic_width)
            buff = deep * self.deep_buff

            # 创建并填充消息
            tracking_armor_msg = ArmorTracking()
            tracking_armor_msg.header = Header()
            tracking_armor_msg.header.stamp = self.get_clock().now().to_msg()
            tracking_armor_msg.header.frame_id = 'tracking_armor_frame'

            # 如果有装甲板，使用其 id，否则设为默认值
            tracking_armor_msg.id = self.tracking_armor[0].class_id if self.tracking_armor else 0
            tracking_armor_msg.yaw = float((yaw + self.offset_yaw) * self.yaw_ratio)
            tracking_armor_msg.pitch = float(pitch + self.offset_pitch + buff)
            tracking_armor_msg.deep = float(deep)

            self.get_logger().info(f"发布的 tracking_armor_msg: {tracking_armor_msg}")
            self.pub_tracker.publish(tracking_armor_msg)

        except Exception as e:
            self.get_logger().error(f"处理装甲板消息时出错: {str(e)}")

    def listener_callback_serial(self, msg):
        # 获取 Decision 数据
        # self.get_logger().info(f'Received Decision data: {msg}')

        # 这里可以对串口数据进行进一步处理
        if self.follow_decision == 1:
            if self.tracking_color != msg.color:
                self.tracking_color = msg.color
                self.get_logger().warn(f'颜色改变为 {self.tracking_color} 号颜色')

def main(args=None):                              # ROS2节点主入口main函数
    rclpy.init(args=args)      # ROS2 Python接口初始化               
    node = ArmorTrackerNode("armor_tracker_node")     # 创建ROS2节点对象
    rclpy.spin(node)                              # 循环等待ROS2退出
    node.destroy_node()                           # 销毁节点对象
    rclpy.shutdown()                              # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()