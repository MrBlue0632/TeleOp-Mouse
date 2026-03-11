#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DualCameraNode:
    def __init__(self):
        rospy.init_node('dual_cam_node')
        self.bridge = CvBridge()

        # 摄像头设备路径
        self.cam0 = cv2.VideoCapture(5, cv2.CAP_V4L2)  # 假设摄像头0连接在/dev/video5

        # 检查摄像头是否成功打开
        if not self.cam0.isOpened():
            rospy.logerr("无法打开摄像头！")
            exit()

        # 创建窗口
        cv2.namedWindow("Camera 0", cv2.WINDOW_NORMAL)

    def capture_and_display(self):
        while not rospy.is_shutdown():
            # 从摄像头获取一帧图像
            ret0, frame0 = self.cam0.read()

            if not ret0:
                rospy.logerr("获取摄像头帧失败！")
                break

            # 转换成灰度图像（可选，如果需要其他图像处理，可以在这里处理）
            gray0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)

            # 显示图像
            cv2.imshow("Camera 0", gray0)

            # 等待按键，退出程序
            key = cv2.waitKey(1)
            if key == 27:  # 按 "Esc" 键退出
                break

        # 释放摄像头和关闭窗口
        self.cam0.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    node = DualCameraNode()
    node.capture_and_display()
