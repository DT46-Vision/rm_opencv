#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "rm_detector/armor_detector_opencv.hpp"
#include "rm_interfaces/msg/armor_cpp_info.hpp"
#include "rm_interfaces/msg/armors_cpp_msg.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp" // 添加参数回调所需的头文件

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode() : Node("armor_detector_opencv_node") {
        // 创建 light_params 对象并初始化
        Light_params light_params = {
            5,    // light_area_min
            -35,  // light_angle_min
            35,   // light_angle_max
            2.0,  // light_red_ratio
            2.0,  // light_blue_ratio
            5,    // cy_tol
            1.3,   // height_rate_tol
            1.5,  // vertical_discretization
            2.1,   // height_multiplier_min
            2.6   // height_multiplier_max
        };

        // 模式参数字典
        int detect_color = 2;  // 颜色参数
        int display_mode = 0;  // 显示模式
        int binary_val = 20;

        // 创建 ArmorDetector 对象并初始化
        detector_ = std::make_shared<ArmorDetector>(detect_color, display_mode, binary_val, light_params);

        // 创建参数服务
        this->declare_parameter<int>("light_area_min", 5);
        this->declare_parameter<int>("light_angle_min", -35);
        this->declare_parameter<int>("light_angle_max", 35);
        this->declare_parameter<float>("light_red_ratio", 2.0);
        this->declare_parameter<float>("light_blue_ratio", 2.0);
        this->declare_parameter<int>("cy_tol", 5);
        this->declare_parameter<float>("height_rate_tol", 1.2);
        this->declare_parameter<float>("vertical_discretization", 1.5);
        this->declare_parameter<float>("height_multiplier_min", 1.8);
        this->declare_parameter<float>("height_multiplier_max", 2.6);
        this->declare_parameter<int>("binary_val", 20);
        this->declare_parameter<int>("detect_color", 2);
        this->declare_parameter<int>("display_mode", 0);

        // 注册参数回调并保存句柄
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ArmorDetectorNode::parameters_callback, this, std::placeholders::_1));
            
        // 创建图像消息订阅者
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ArmorDetectorNode::image_callback, this, std::placeholders::_1));

        // 订阅相机信息主题，不绑定具体回调，而是用 lambda 更新成员变量
        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                camera_info_ = msg; // 存储最新的相机信息
                RCLCPP_INFO(this->get_logger(), "Received camera info.");
            });

        // 创建发布者
        publisher_armors_ = this->create_publisher<rm_interfaces::msg::ArmorsCppMsg>("/detector/armors_info", 10);
        publisher_armors_img_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/armors_img", 10);
        publisher_bin_img_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/bin_img", 10);

        RCLCPP_INFO(this->get_logger(), "Armor Detector Node has been started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        
        cv::Mat bin;
        cv::Mat drawn;
        std::vector<Armor_info> armors_info;

        // 使用 ArmorDetector 对象处理图像
        armors_info = detector_->detect_armors(cv_image);
        std::tie(bin, drawn) = detector_->display();

        // 示例：发布装甲信息
        rm_interfaces::msg::ArmorsCppMsg armors_msg;
        armors_msg.header.stamp = this->get_clock()->now();
        armors_msg.header.frame_id = "camera_frame";

        if (armors_info.empty()) {
            RCLCPP_INFO(this->get_logger(), "No armors detected.");
        }
        else{
            for (const auto& armor : armors_info) {
                rm_interfaces::msg::ArmorCppInfo armor_info;
                armor_info.height = armor.height;
                armor_info.class_id = armor.class_id;
                armor_info.cx = armor.cx;
                armor_info.cy = armor.cy;
                RCLCPP_INFO(this->get_logger(), "[class_id: %d, cx: %f, cy: %f, height: %f]\n", 
                    armor.class_id, armor.cx, armor.cy, armor.height);
                armors_msg.armors.push_back(armor_info);
            }
        }
        
        publisher_armors_->publish(armors_msg);

        sensor_msgs::msg::Image bin_img_msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", bin).toImageMsg();
        publisher_bin_img_->publish(bin_img_msg);

        sensor_msgs::msg::Image armors_img_msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawn).toImageMsg();
        publisher_armors_img_->publish(armors_img_msg);
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto& param : parameters) {
            if (param.get_name() == "light_area_min") {
                detector_->update_light_area_min(param.as_int());
            } else if (param.get_name() == "light_angle_min") {
                detector_->update_light_angle_min(param.as_int());
            } else if (param.get_name() == "light_angle_max") {
                detector_->update_light_angle_max(param.as_int());
            } else if (param.get_name() == "light_red_ratio") {
                detector_->update_light_red_ratio(param.as_double());
            } else if (param.get_name() == "light_blue_ratio") {
                detector_->update_light_blue_ratio(param.as_double());
            } else if (param.get_name() == "cy_tol") {
                detector_->update_cy_tol(param.as_int());
            } else if (param.get_name() == "height_rate_tol") {
                detector_->update_height_rate_tol(param.as_double());
            } else if (param.get_name() == "vertical_discretization") {
                detector_->update_vertical_discretization(param.as_double());
            } else if (param.get_name() == "height_multiplier_min") {
                detector_->update_height_multiplier_min(param.as_double());
            } else if (param.get_name() == "height_multiplier_max") {
                detector_->update_height_multiplier_max(param.as_double());
            } else if (param.get_name() == "binary_val") {
                detector_->update_binary_val(param.as_int());
            } else if (param.get_name() == "detect_color") {
                detector_->update_detect_color(param.as_int());
            } else if (param.get_name() == "display_mode") {
                detector_->update_display_mode(param.as_int());
            }
        }
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Publisher<rm_interfaces::msg::ArmorsCppMsg>::SharedPtr publisher_armors_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_armors_img_; // 添加声明
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_bin_img_;    // 添加声明
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_; // 添加声明
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::shared_ptr<ArmorDetector> detector_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}