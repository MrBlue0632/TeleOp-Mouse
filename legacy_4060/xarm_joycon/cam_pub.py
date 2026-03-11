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
        self.cam0 = cv2.VideoCapture(4, cv2.CAP_V4L2)   # 腕部摄像头
        self.cam6 = cv2.VideoCapture(10, cv2.CAP_V4L2)  # 上方摄像头
 
        # # 设置分辨率（可选）
        # self.cam6.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cam6.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cam0.isOpened():
            rospy.logerr("❌ Failed to open /dev/video1")
            raise RuntimeError("Cannot open /dev/video1")
        if not self.cam6.isOpened():
            rospy.logerr("❌ Failed to open /dev/video6")
            raise RuntimeError("Cannot open /dev/video6")

        # 两个摄像头各自发布到不同的 topic
        self.pub_cam1 = rospy.Publisher('/cam_1', Image, queue_size=10)
        self.pub_cam2 = rospy.Publisher('/cam_2', Image, queue_size=10)

        # 定时发布 ~30Hz
        rospy.Timer(rospy.Duration(0.03), self.publish_images)

    def publish_images(self, event):
        ret0, frame0 = self.cam0.read()
        ret6, frame6 = self.cam6.read()
        # print(frame6.shape)
        if ret0:
            ros_img0 = self.bridge.cv2_to_imgmsg(frame0, encoding='bgr8')
            self.pub_cam1.publish(ros_img0)
            # rospy.loginfo("Published real image from camera0")
        else:
            rospy.logwarn("⚠️ Failed to capture image from /dev/video1")

        if ret6:
            ros_img6 = self.bridge.cv2_to_imgmsg(frame6, encoding='bgr8')
            self.pub_cam2.publish(ros_img6)
            # rospy.loginfo("Published real image from camera6")
        else:
            rospy.logwarn("⚠️ Failed to capture image from /dev/video6")

    def __del__(self):
        if self.cam0.isOpened():
            self.cam0.release()
        if self.cam6.isOpened():
            self.cam6.release()
        rospy.loginfo("🎥 Cameras released.")

if __name__ == '__main__':
    try:
        DualCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# #!/usr/bin/env python3
# import rospy
# import cv2
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# class CameraNode:
#     def __init__(self):
#         rospy.init_node('cam_1')
#         self.bridge = CvBridge()

#         # 打开摄像头（默认设备0），GoPro一般也能识别为VideoCapture设备
#         self.cap = cv2.VideoCapture(0)
#         if not self.cap.isOpened():
#             rospy.logerr("Failed to open camera.")
#             raise RuntimeError("Cannot open camera.")

#         self.pub = rospy.Publisher('/cam_1', Image, queue_size=10)
#         rospy.Timer(rospy.Duration(0.03), self.publish_image)  # 10Hz 可调

#     def publish_image(self, event):
#         ret, frame = self.cap.read()
#         if not ret:
#             rospy.logwarn("Failed to capture image.")
#             return

#         # OpenCV 默认是 BGR 格式
#         ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
#         self.pub.publish(ros_img)
#         # rospy.loginfo("Published real image from camera")

#     def __del__(self):
#         if self.cap.isOpened():
#             self.cap.release()
#             rospy.loginfo("Camera released.")

# if __name__ == '__main__':
#     try:
#         CameraNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
