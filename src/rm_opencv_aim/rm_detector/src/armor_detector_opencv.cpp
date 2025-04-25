#include "rm_detector/armor_detector_opencv.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <set>

// 计算两个点之间的距离
double calculate_distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return cv::norm(p1 - p2);
}

// 调整宽高和角度的函数
std::pair<cv::Size2f, double> adjust(const cv::Size2f& w_h, double angle) {
    float w = w_h.width;
    float h = w_h.height;

    if (w > h) {
        std::swap(w, h);
        if (angle >= 0) {
            angle -= 90;
        } else {
            angle += 90;
        }
    }

    return std::make_pair(cv::Size2f(w, h), angle);
}

// 将角度转换为斜率的函数
double angle_to_slope(double angle_degrees) {
    double angle_radians = angle_degrees * M_PI / 180.0;
    return std::tan(angle_radians);
}

// Light 类的构造函数实现
Light::Light(const cv::Point2f& up, const cv::Point2f& down, double angle, int color)
    : up(up), down(down), angle(angle), color(color) {
    cx = static_cast<int>(std::abs(up.x - down.x) / 2 + std::min(up.x, down.x));
    cy = static_cast<int>(std::abs(up.y - down.y) / 2 + std::min(up.y, down.y));
    height = calculate_distance(up, down);
}

// Armor 类的构造函数实现
Armor::Armor(const Light& light1, const Light& light2, double height, int type)
    : light1_up(light1.up), light1_down(light1.down),
      light2_up(light2.up), light2_down(light2.down),
      height(height), type(type) {
    int armor_cx = static_cast<int>(std::abs(light1.cx - light2.cx) / 2 + std::min(light1.cx, light2.cx));
    int armor_cy = static_cast<int>(std::abs(light1.cy - light2.cy) / 2 + std::min(light1.cy, light2.cy));
    center = cv::Point(armor_cx, armor_cy);
    color = light1.color;
}

int Armor::type_class() const {
    if (color == 0) {
        if (type == 0) return 7;
        else if (type == 1) return 6;
    } else if (color == 1) {
        if (type == 0) return 1;
        else if (type == 1) return 0;
    }
    return -1;
}

// ArmorDetector 类的实现
ArmorDetector::ArmorDetector(int detect_color, int display_mode, int binary_val, const Light_params& light_params)
    : color(detect_color), binary_val(binary_val), display_mode(display_mode), light_params(light_params) {}

void ArmorDetector::update_light_area_min(int new_light_area_min) {
    light_params.light_area_min = new_light_area_min;
}

void ArmorDetector::update_light_angle_min(int new_light_angle_min) {
    light_params.light_angle_min = new_light_angle_min;
}
void ArmorDetector::update_light_angle_max(int new_light_angle_max) {
    light_params.light_angle_max = new_light_angle_max;
}

void ArmorDetector::update_light_red_ratio(float new_light_red_ratio) {
    light_params.light_red_ratio = new_light_red_ratio;
}

void ArmorDetector::update_light_blue_ratio(float new_light_blue_ratio) {
    light_params.light_blue_ratio = new_light_blue_ratio;
}

void ArmorDetector::update_cy_tol(int new_cy_tol) {
    light_params.cy_tol = new_cy_tol;
}

void ArmorDetector::update_height_rate_tol(float new_height_rate_tol) {
    light_params.height_rate_tol = new_height_rate_tol;
}

void ArmorDetector::update_vertical_discretization(float new_vertical_discretization) {
    light_params.vertical_discretization = new_vertical_discretization;
}

void ArmorDetector::update_height_multiplier_min(float new_height_multiplier_min) {
    light_params.height_multiplier_min = new_height_multiplier_min;
}

void ArmorDetector::update_height_multiplier_max(float new_height_multiplier_max) {
    light_params.height_multiplier_max = new_height_multiplier_max;
}

void ArmorDetector::update_binary_val(int new_binary_val) {
    binary_val = new_binary_val;
}

void ArmorDetector::update_detect_color(int new_color) {
    color = new_color;
}

void ArmorDetector::update_display_mode(int new_display_mode) {
    display_mode = new_display_mode;
}

cv::Mat ArmorDetector::process(const cv::Mat& img_input) {
    img = img_input.clone();
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_img, img_binary, binary_val, 255, cv::THRESH_BINARY);
    return img_binary;
}

std::vector<Light> ArmorDetector::find_lights(const cv::Mat& img_binary_input) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::RotatedRect> is_lights;
    std::vector<Light> lights_found;

    cv::findContours(img_binary_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) >= light_params.light_area_min) {
            cv::RotatedRect min_rect = cv::minAreaRect(contour);
            cv::Size2f w_h = min_rect.size;
            double angle = min_rect.angle;

            std::tie(w_h, angle) = adjust(w_h, angle);

            if (angle >= light_params.light_angle_min && angle <= light_params.light_angle_max) {
                cv::RotatedRect rect(min_rect.center, w_h, static_cast<float>(angle));
                is_lights.push_back(rect);
            }
        }
    }

    for (const auto& rect : is_lights) {
        cv::Point2f box[4];
        rect.points(box);

        int up_x = static_cast<int>(std::abs(box[0].x - box[3].x) / 2 + std::min(box[0].x, box[3].x));
        int up_y = static_cast<int>(std::abs(box[0].y - box[3].y) / 2 + std::min(box[0].y, box[3].y));
        cv::Point2f up(up_x, up_y);

        int down_x = static_cast<int>(std::abs(box[1].x - box[2].x) / 2 + std::min(box[1].x, box[2].x));
        int down_y = static_cast<int>(std::abs(box[1].y - box[2].y) / 2 + std::min(box[1].y, box[2].y));
        cv::Point2f down(down_x, down_y);

        int length = static_cast<int>(calculate_distance(up, down));
        cv::Mat roi(1, length, CV_8UC3, cv::Scalar(0, 0, 0));

        for (int i = 0; i < length; ++i) {
            float t = static_cast<float>(i) / length;
            int current_x = static_cast<int>(up_x + (down_x - up_x) * t);
            int current_y = static_cast<int>(up_y + (down_y - up_y) * t);

            if (current_x >= 0 && current_x < img.cols && current_y >= 0 && current_y < img.rows) {
                roi.at<cv::Vec3b>(0, i) = img.at<cv::Vec3b>(current_y, current_x);
            }
        }

        int sum_b = cv::sum(roi)[0];
        int sum_r = cv::sum(roi)[2];

        if ((color == 1 || color == 2) && sum_b > sum_r * light_params.light_blue_ratio) {
            lights_found.push_back(Light(up, down, rect.angle, 1));
        } else if ((color == 0 || color == 2) && sum_r > sum_b * light_params.light_red_ratio) {
            lights_found.push_back(Light(up, down, rect.angle, 0));
        }
    }

    lights = lights_found;
    return lights;
}

std::pair<int, float> ArmorDetector::is_close(const Light& light1, const Light& light2) {
    // 计算公共变量
    float height = std::max(light1.height, light2.height);
    float height_rate = height / std::min(light1.height, light2.height);
    // 如果高度比例不符合，直接返回
    if (height_rate >= light_params.height_rate_tol) {
        return std::make_pair(-1, -1.0f);
    }
    float distance = calculate_distance({light1.cx, light1.cy}, {light2.cx, light2.cy});

    // 检查是否在同一水平线上
    if (std::abs(light1.cy - light2.cy) < light_params.cy_tol) {
        // 水平情况下的距离检查
        if (distance > height * light_params.height_multiplier_min &&
         distance < height * light_params.height_multiplier_max) {
            return std::make_pair(0, height);
        } else if (distance > height * 1.56f * light_params.height_multiplier_min &&
         distance < height * 1.76f * light_params.height_multiplier_max) {
            return std::make_pair(1, height);
        }
    } else {
        float light1_angle = std::atan2(light1.up.y - light1.down.y, light1.up.x - light1.down.x) * 180.0 / M_PI;
        float light2_angle = std::atan2(light2.up.y - light2.down.y, light2.up.x - light2.down.x) * 180.0 / M_PI;
        float line_angle = std::atan2(light1.cy - light2.cy, light1.cx - light2.cx) * 180.0 / M_PI;

        float slope1 = angle_to_slope(light1_angle);
        float slope2 = angle_to_slope(light2_angle);
        float slope_line = angle_to_slope(line_angle);

        // 检查垂直性
        if (std::abs(slope1 * slope_line + 1) < light_params.vertical_discretization ||
            std::abs(slope2 * slope_line + 1) < light_params.vertical_discretization) {
            if (distance > height * light_params.height_multiplier_min &&
                distance < height * light_params.height_multiplier_max) {
                return std::make_pair(0, height);
            } else if (distance > height * 1.56f * light_params.height_multiplier_min &&
                distance < height * 1.76f * light_params.height_multiplier_max) {
                return std::make_pair(1, height);
            }
        }
    }

    return std::make_pair(-1, -1.0f);
}

std::vector<Armor> ArmorDetector::is_armor(const std::vector<Light>& lights) {
    std::vector<Armor> armors_found;
    std::set<int> processed_indices;
    
    if (lights.size() < 2) {
        return armors_found;
    }

    for (size_t i = 0; i <= lights.size() - 2; i++) {
        if (processed_indices.count(i)) continue;
        const Light& light = lights[i];
            if (lights[i + 1].color == light.color) {
                int type;
                float height;
                std::tie(type, height) = is_close(light, lights[i + 1]);

                if (type >= 0) {
                    armors_found.push_back(Armor(light, lights[i + 1], height, type));
                    processed_indices.insert(i);
                    processed_indices.insert(i + 1);
                }
            }
        }

    armors = armors_found;
    return armors;
}

std::vector<Armor_info> ArmorDetector::id_armor() {
    std::vector<Armor_info> info_found;
    int img_height = img.rows;
    int img_width = img.cols;

    for (const auto& armor : armors) {
        cv::Point2f center = armor.center;
        float center_x = center.x - (img_width / 2);
        float center_y = -center.y + (img_height / 2);

        Armor_info info;
        info.class_id = armor.type_class();
        info.height = armor.height;
        info.cx = center_x;
        info.cy = center_y;
        info_found.push_back(info);
    }

    armors_info = info_found;
    return armors_info;
}

cv::Mat ArmorDetector::draw_lights(cv::Mat img_draw) {
    for (const auto& light : lights) {
        if (light.color == 0) {
            cv::line(img_draw, light.up, light.down, cv::Scalar(0, 100, 255), 1);
            cv::circle(img_draw, cv::Point(light.cx, light.cy), 1, cv::Scalar(255, 0, 0), -1);
        } else if (light.color == 1) {
            cv::line(img_draw, light.up, light.down, cv::Scalar(200, 71, 90), 1);
            cv::circle(img_draw, cv::Point(light.cx, light.cy), 1, cv::Scalar(0, 0, 255), -1);
        }
    }
    return img_draw;
}

cv::Mat ArmorDetector::draw_armors(cv::Mat img_draw) {
    for (const auto& armor : armors) {
        int img_height = img_draw.rows;
        int img_width = img_draw.cols;
        cv::Point center = armor.center;
        int center_x = static_cast<int>(center.x - (img_width / 2));
        int center_y = static_cast<int>(-(center.y - (img_height / 2)));

        if (armor.color == 0) {
            cv::line(img_draw, armor.light1_up, armor.light2_down, cv::Scalar(128, 0, 128), 1);
            cv::line(img_draw, armor.light2_up, armor.light1_down, cv::Scalar(128, 0, 128), 1);
            cv::putText(img_draw, "(" + std::to_string(center_x) + ", " + std::to_string(center_y) + ")",
                        center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(120, 255, 255), 2);
        } else if (armor.color == 1) {
            cv::line(img_draw, armor.light1_up, armor.light2_down, cv::Scalar(255, 255, 0), 1);
            cv::line(img_draw, armor.light2_up, armor.light1_down, cv::Scalar(255, 255, 0), 1);
            cv::putText(img_draw, "(" + std::to_string(center_x) + ", " + std::to_string(center_y) + ")",
                        center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(120, 255, 255), 2);
        }
    }
    return img_draw;
}

cv::Mat ArmorDetector::draw_img() {
    cv::Mat img_draw = img.clone();
    img_draw = draw_armors(img_draw);
    img_drawn = draw_lights(img_draw);
    return img_drawn;
}

std::tuple<cv::Mat, cv::Mat> ArmorDetector::display() {
    if (display_mode == 1) {
        return std::make_tuple(img_binary, cv::Mat(1, 1, CV_8UC1, cv::Scalar(0)));
    } else if (display_mode == 2) {
        img_drawn = draw_img();
        return std::make_tuple(img_binary, img_drawn);
    } else if (display_mode == 0) {
        return std::make_tuple(cv::Mat(1, 1, CV_8UC1, cv::Scalar(0)), cv::Mat(1, 1, CV_8UC1, cv::Scalar(0)));
    } else {
        std::cerr << "Invalid display mode" << std::endl;
        return std::make_tuple(cv::Mat(1, 1, CV_8UC1, cv::Scalar(0)), cv::Mat(1, 1, CV_8UC1, cv::Scalar(0)));
    }
}

std::vector<Armor_info> ArmorDetector::detect_armors(const cv::Mat& img_input) {
    img_binary = process(img_input);
    lights = find_lights(img_binary);
    armors = is_armor(lights);
    armors_info = id_armor();
    return armors_info;
}