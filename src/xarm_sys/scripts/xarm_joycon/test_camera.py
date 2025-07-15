import cv2

# 简单测试摄像头
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print("成功读取帧!")
        cv2.imshow('test.jpg', frame)
        cv2.waitKey(0)
    cap.release()
else:
    print("无法打开摄像头")