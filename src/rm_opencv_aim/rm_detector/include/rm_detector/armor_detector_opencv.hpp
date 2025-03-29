#ifndef ARMOR_DETECTOR_OPENCV_HPP
#define ARMOR_DETECTOR_OPENCV_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <set>

double calculate_distance(const cv::Point2f& p1, const cv::Point2f& p2);
std::pair<cv::Size2f, double> adjust(const cv::Size2f& w_h, double angle);
double angle_to_slope(double angle_degrees);

class Light {
public:
    int cx;
    int cy;
    double height;
    cv::Point2f up;
    cv::Point2f down;
    double angle;
    int color;

    Light(const cv::Point2f& up, const cv::Point2f& down, double angle, int color);
};

class Armor {
public:
    cv::Point center;
    cv::Point2f light1_up;
    cv::Point2f light1_down;
    cv::Point2f light2_up;
    cv::Point2f light2_down;
    int color;
    double height;
    int type;

    Armor(const Light& light1, const Light& light2, double height, int type);
    int type_class() const;
};

// 定义 Armor_info 结构体
struct Armor_info {
    float height; // 装甲板高度
    int class_id; // 装甲板类别 ID
    float cx;     // 中心 x 坐标
    float cy;     // 中心 y 坐标
};

// 定义 Light_params 结构体
struct Light_params {
    int light_area_min;
    int light_angle_min;
    int light_angle_max;
    float light_red_ratio;
    float light_blue_ratio;
    int cy_tol;
    float height_rate_tol;
    int light_angle_tol;
    float vertical_discretization;
    float height_multiplier_min;
    float height_multiplier_max;
};

class ArmorDetector {
public:
    cv::Mat img;
    cv::Mat img_binary;
    cv::Mat img_drawn;

    std::vector<Light> lights;
    std::vector<Armor> armors;
    std::vector<Armor_info> armors_info;

    int binary_val;
    int color;
    int display_mode;
    Light_params light_params;

    ArmorDetector(int detect_color, int display_mode, int binary_val, const Light_params& light_params);

    void update_light_area_min(int new_light_area_min);
    void update_light_angle_min(int new_light_angle_min);
    void update_light_angle_max(int new_light_angle_max);
    void update_light_red_ratio(float new_light_red_ratio);
    void update_light_blue_ratio(float new_light_blue_ratio);
    void update_cy_tol(int new_cy_tol);
    void update_height_rate_tol(float new_height_rate_tol);
    void update_light_angle_tol(int new_light_angle_tol);
    void update_vertical_discretization(float new_vertical_discretization);
    void update_height_multiplier_min(float new_height_multiplier_min);
    void update_height_multiplier_max(float new_height_multiplier_max);
    void update_binary_val(int new_binary_val);
    void update_detect_color(int new_color);
    void update_display_mode(int new_display_mode);

    cv::Mat process(const cv::Mat& img_input);
    std::vector<Light> find_lights(const cv::Mat& img_binary_input);
    std::pair<int, float> is_close(const Light& light1, const Light& light2);
    std::vector<Armor> is_armor(const std::vector<Light>& lights);
    std::vector<Armor_info> id_armor();
    cv::Mat draw_lights(cv::Mat img_draw);
    cv::Mat draw_armors(cv::Mat img_draw);
    cv::Mat draw_img();
    std::tuple<cv::Mat, cv::Mat> display();
    std::vector<Armor_info> detect_armors(const cv::Mat& img_input);
};

#endif // ARMOR_DETECTOR_OPENCV_HPP