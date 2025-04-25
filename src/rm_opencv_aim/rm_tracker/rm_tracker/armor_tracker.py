import math

# 定义常量, 弧度转角度
RAD2DEG = 180 / math.pi
DEG2RAD = math.pi / 180

class Armor:
    def __init__(self, class_id, center, height):
        self.class_id = class_id
        self.center = center
        self.height = height

def select_tracking_armor(msg, color, track_height_tol):
    """
    从 ArmorsCppMsg 中选择目标装甲板。
    
    Args:
        msg: ArmorsCppMsg 类型的消息
        color: 目标颜色 (1: 蓝色, 0: 红色)
        track_height_tol: 高度差阈值
        cx_tol: 中心 x 坐标差阈值
    
    Returns:
        list: 包含单个目标 Armor 对象的列表，若无目标则为空列表
    """
    # 解包装甲板信息
    armor_info = [Armor(info.class_id, (info.cx, info.cy), info.height) for info in msg.armors]
    
    # 如果没有装甲板，直接返回空列表
    if not armor_info:
        return []

    # 根据颜色筛选：1 表示蓝色 (class_id < 6)，0 表示红色 (class_id > 5)
    if color == 1:
        filtered_color_data = [armor for armor in armor_info if armor.class_id < 6]
    elif color == 0:
        filtered_color_data = [armor for armor in armor_info if armor.class_id > 5]
    else:
        return []  # 非法颜色值

    # 如果筛选后没有装甲板，返回空列表
    if not filtered_color_data:
        return []

    # 如果只有一个装甲板，直接返回
    if len(filtered_color_data) == 1:
        return filtered_color_data

    # 按高度排序，取前两个
    top_two = sorted(filtered_color_data, key=lambda armor: armor.height, reverse=True)[:2]
    height_diff = top_two[0].height - top_two[1].height

    # 高度差不多，就返回 x 坐标最小的装甲板
    if height_diff <= track_height_tol:
        return [top_two[0]] if abs(top_two[0].center[0]) < abs(top_two[1].center[0]) else [top_two[1]]

    # 否则，返回 y 坐标最高的装甲板
    return [top_two[0]]

def deep_cal(x):
    if x <= 0:
        return 0
    # https://mycurvefit.com/ 
    # 测了若干组像素对应距离的数据，输入上述网站计算得到的像素-距离公式
    a = -12.75855
    b = 2162.459
    c = 4.823192
    d = 1.161739
    
    y = a + (b - a) / (1 + (x / c) ** d)
    return y

def pixel_to_angle_and_deep(height, center, vfov, pic_width):

    deep = deep_cal(height) # 估计距离
    # 确保 vfov 是以弧度为单位
    vfov_radians = vfov * DEG2RAD
    # 相机 x, y 坐标系下投影面的 Z 轴距离(单位: 像素)
    focal_pixel_distance = (pic_width / 2) / math.tan(vfov_radians / 2)
    # 确保 focal_pixel_distance 不为零
    if focal_pixel_distance == 0:
        focal_pixel_distance = 0.000_000_1
    # 计算角度
    yaw   = math.atan(center[0] / focal_pixel_distance) * RAD2DEG
    pitch = math.atan(center[1] / focal_pixel_distance) * RAD2DEG
    return yaw, pitch, deep

if __name__ == "__main__":

    armors_dict = {
        "179":  {"class_id": 7, "height": 429, "center": [ 147,  333]},
        "-143": {"class_id": 1, "height": 288, "center": [-143, -35]},
        "175":  {"class_id": 7, "height": 1, "center": [ 149,  36]},
        "-113": {"class_id": 1, "height": 300, "center": [ 91, -35]},
    }

    result = select_tracking_armor(armors_dict, 0, 20, 10)
    if result:
        yaw, pitch, deep = pixel_to_angle_and_deep(result["height"], result["center"], 55, 72)
    