import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数文件路径
    params_file = os.path.join(
        get_package_share_directory("hik_camera"), "config", "camera_params.yaml"
    )
    
    # 相机信息URL
    camera_info_url = "package://hik_camera/config/camera_info.yaml"

    # 创建launch描述
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(name="params_file", default_value=params_file),
        DeclareLaunchArgument(name="camera_info_url", default_value=camera_info_url),
        DeclareLaunchArgument(name="use_sensor_data_qos", default_value="false"),
        
        # 启动 rm_serial_node
        # Node(
        #     package="rm_serial_python",
        #     executable="rm_serial_node",
        #     output="screen",
        #     emulate_tty=True,
        # ),        

        # 启动 armor_detector
        Node(
            package="rm_detector",
            executable="rm_detector_node",
            output="screen",
            emulate_tty=True,
        ),
        
        # 启动 rm_tracker_node
        Node(
            package="rm_tracker",
            executable="rm_tracker_node",
            output="screen",
            emulate_tty=True,
        ),
        
        # 启动 hik_camera_node
        Node(
            package="hik_camera",
            executable="hik_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
                },
            ],
        ),
    ])