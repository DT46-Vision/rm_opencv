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
        
        # 初始协方差矩阵（较大的不确定性）
        self.kf.errorCovPost = np.eye(2, dtype=np.float32) * 1000  # 初始不确定性较大

    def predict(self):
        # 进行预测
        return self.kf.predict()

    def update(self, yaw):
        # 更新状态
        measurement = np.array([[yaw]], np.float32)
        self.kf.correct(measurement)

    def reset(self):
        """重置卡尔曼滤波器状态"""
        # 重置状态向量为零
        self.kf.statePost = np.zeros((2, 1), np.float32)
        # 重置协方差矩阵为初始值（较大的不确定性）
        self.kf.errorCovPost = np.eye(2, dtype=np.float32) * 1000

    def get_state(self):
        # 获取当前状态（yaw）
        return self.kf.statePost[0, 0]  # 仅返回yaw