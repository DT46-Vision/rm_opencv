import numpy as np
import cv2

class KalmanFilter:
    def __init__(self):
        self.dt = 0.1
        # 初始化卡尔曼滤波器
        self.kf = cv2.KalmanFilter(2, 1)  # 状态维度为2，观测维度为1
        
        # 状态转移矩阵
        self.kf.transitionMatrix = np.array([[1, self.dt],
                                              [0, 1]], np.float32)
        
        # 观测矩阵
        self.kf.measurementMatrix = np.array([[1, 0]], np.float32)
        
        # 过程噪声协方差矩阵
        self.kf.processNoiseCov = np.eye(2, dtype=np.float32) * 0.1
        
        # 观测噪声协方差矩阵
        self.kf.measurementNoiseCov = np.array([[0.5]], np.float32)
        
        # 初始状态
        self.kf.statePost = np.zeros((2, 1), np.float32)

    def predict(self):
        # 进行预测
        return self.kf.predict()

    def update(self, yaw):
        # 更新状态
        measurement = np.array([[yaw]], np.float32)
        self.kf.correct(measurement)

    def get_state(self):
        # 获取当前状态（yaw）
        return self.kf.statePost[0, 0]  # 仅返回yaw

# 示例用法
if __name__ == "__main__":
    dt = 0.1  # 时间步长（秒）
    kf = KalmanFilter()

    # 模拟传入的yaw数据
    measurements = [1.0, 1.2, 1.1]

    for yaw in measurements:
        kf.predict()  # 进行预测
        kf.update(yaw)  # 更新状态
        predicted_yaw = kf.get_state()  # 获取预测的yaw
        print(f"预测的 yaw: {predicted_yaw:.2f}")