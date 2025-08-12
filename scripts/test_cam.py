import pyrealsense2 as rs
import numpy as np
import cv2

def test_realsense():
    # 配置 RealSense 流
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 配置颜色流
    pipeline.start(config)

    try:
        # 持续获取图像帧并显示
        while True:
            frames = pipeline.wait_for_frames()  # 获取帧
            color_frame = frames.get_color_frame()  # 获取颜色帧
            if not color_frame:
                print("Failed to capture image.")
                break

            # 转换为 NumPy 数组
            frame = np.asanyarray(color_frame.get_data())

            # 使用 OpenCV 显示图像
            cv2.imshow('RealSense', frame)

            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 释放资源
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_realsense()
