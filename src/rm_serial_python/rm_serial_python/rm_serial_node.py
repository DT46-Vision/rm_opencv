import time
import serial
import threading
import struct
import zlib

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header  # 字符串消息类型和头部消息类型
from rm_interfaces.msg import ArmorTracking , Decision # 导入自定义消息类型


class RMSerialDriver(Node):
    def __init__(self, name):
        super().__init__(name)

        self.get_logger().info("启动 RMSerialDriver!")

        # 获取参数
        self.get_params()

        # 创建订阅者
        self.sub_tracker = self.create_subscription(
            ArmorTracking, "/tracker/target", self.send_data, 10
        )

        # 创建发布者
        self.pub_uart_receive = self.create_publisher(Decision, "/nav/decision", 10)
        
        # 创建变量
        self.tracking_color = -1
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=self.device_name,
                baudrate=self.baud_rate,
                timeout=1,
                write_timeout=1,
            )
            if self.serial_port.is_open:
                self.get_logger().info("创建串口 successfully.")
                self.receive_thread = threading.Thread(target=self.receive_data)
                self.receive_thread.start()
                # self.timer = self.create_timer(1.0, self.receive_data_callback)  #有 bug 弃用


        except serial.SerialException as e:
            self.get_logger().error(f"创建串口时出错: {self.device_name} - {str(e)}")
            raise e

    def get_params(self):
        """获取并设置串口相关的参数"""
        self.device_name  = self.declare_parameter("device_name", "/dev/ttyACM0").value
        self.baud_rate    = self.declare_parameter("baud_rate", 115200).value
        self.flow_control = self.declare_parameter("flow_control", "none").value
        self.parity       = self.declare_parameter("parity", "none").value
        self.stop_bits    = self.declare_parameter("stop_bits", "1").value
        

    def receive_data(self):
        """接收串口数据并处理"""
        serial_receive_msg = Decision()
        serial_receive_msg.header.frame_id = 'serial_receive_frame'
        serial_receive_msg.header = Header()
        serial_receive_msg.header.stamp = self.get_clock().now().to_msg()
        serial_receive_msg.color = -1  # 初始化颜色为-1
        self.pub_uart_receive.publish(serial_receive_msg)  # 发布初始化的消息

        while rclpy.ok():
            time.sleep(1)  # 控制循环频率
            try:
                # 更新消息头部
                serial_receive_msg.header.stamp = self.get_clock().now().to_msg()
                # # 打印接收到的任何数据并转换成 hex 字符串
                # str = self.serial_port.read(128)
                # self.get_logger().info(f"接收到的数据: {str}") 


                # 读取数据头部
                header = self.serial_port.read(1)
                if not header or len(header) != 1:
                    self.get_logger().warn("Header 读取失败或长度不匹配")
                    continue

                # 如果头部存在且等于 0xA5
                if header[0] == 0xA5:
                    # 读取颜色信息字节
                    color_byte = self.serial_port.read(1)
                    if not color_byte or len(color_byte) != 1:
                        self.get_logger().warn("颜色信息读取失败或长度不匹配")
                        continue

                    # 解包颜色信息
                    raw_color = struct.unpack('>B', color_byte)[0]
                    detect_color = raw_color  # 提取最低有效位

                    self.get_logger().info(f"解包收到的数据: detect_color: {detect_color}")

                    # 更新目标颜色参数
                    self.tracking_color = detect_color
                    serial_receive_msg.color = detect_color
                else:
                    self.get_logger().warn("Invalid header received, 没有数据")

                # 发送ROS消息
                self.pub_uart_receive.publish(serial_receive_msg)
                self.get_logger().info(f'Publishing: {serial_receive_msg}， tracking_color： {serial_receive_msg.color}')

            except (serial.SerialException, struct.error, ValueError) as e:
                self.get_logger().error(f"接收数据时出错: {str(e)}")
                self.reopen_port()
    
    

    def send_data(self, msg):
        """处理目标信息并通过串口发送"""
        try:
            # 目标ID对应的装甲板, 此列表无实际作用， 放在这里方便查看
            id_map =   ["B1", "B2", "B3", "B4", "B5", "B7", 
                        "R1", "R2", "R3", "R4", "R5", "R7"]
            
            # self.get_logger().info(f"发送数据: {msg}")

            header = 0x5A
            yaw    = msg.yaw
            pitch  = msg.pitch
            deep   = msg.deep

            # self.get_logger().info(f"yaw type: {yaw}, pitch type: {pitch}, deep type: {deep}")

            packet = struct.pack(
                "<Bfff",
                header,
                yaw,
                pitch,
                deep,
            )

            # 计算校验和
            checksum = zlib.crc32(packet) & 0xFFFF  # 取低16位作为校验和
            packet_with_checksum = packet + struct.pack("<H", checksum)

            self.get_logger().info(f"打包后的数据: {packet_with_checksum}")

            self.serial_port.write(packet_with_checksum)

        except Exception as e:
            self.get_logger().error(f"发送数据时出错: {str(e)}")
            self.reopen_port()

    def reopen_port(self):
        """重新打开串口"""
        self.get_logger().warn("尝试重新打开串口")
        try:
            if self.serial_port.is_open:
                self.serial_port.close()

            self.serial_port.open()
            self.get_logger().info("成功重新打开串口")
        except Exception as e:
            self.get_logger().error(f"重新打开串口时出错: {str(e)}")
            time.sleep(1)
            self.reopen_port()


def main(args=None):  # ROS2节点主入口main函数
    rclpy.init(args=args)  # ROS2 Python接口初始化
    node = RMSerialDriver("rm_serial_python")  # 创建ROS2节点对象
    rclpy.spin(node)  # 循环等待ROS2退出
    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS2 Python接口


if __name__ == "__main__":
    main()


