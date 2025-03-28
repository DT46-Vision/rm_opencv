# 相机测试
import cv2
import time
import os


cap = cv2.VideoCapture(0)

# 设置分辨率
width = 1280  # 你想要的宽度
height = 720   # 你想要的高度
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

# 设置帧率
fps = 180  # 你想要的帧率
cap.set(cv2.CAP_PROP_FPS, fps)

w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)

# 检查设置是否成功
print(f"设置的分辨率: {w} x {h}")
print(f"设置的帧率: {fps}")

while True:
    start = time.time_ns()

    ret, frame = cap.read()

    end = time.time_ns()

    dt = (end - start) / 1e6  # 转为 ms

    print(f"延迟: {dt} ms, fps: {int(1/dt*1000)} Hz, 图像尺寸为 {frame.shape}, 设置的帧率是 {fps}")

    if os.environ.get('DISPLAY') and os.isatty(0):
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()