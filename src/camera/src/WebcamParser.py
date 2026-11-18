import sys
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage

class st_WebcamParser:
    def __init__(self):

        self.st_Publisher = rospy.Publisher("/usb_cam/image/compressed", CompressedImage, queue_size=10)

        self.CameraID = "/dev/video0"  # USB Port에 따라 가변적
        self.Quality = 95
        self.st_VideoCapture = cv2.VideoCapture(self.CameraID, cv2.CAP_V4L2)

        # Maximum Resolution
        self.st_VideoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 1020)         # Original 640
        self.st_VideoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)        # Original 480

        if not self.st_VideoCapture.isOpened():
            rospy.logerr("Failed to open camera")
            sys.exit(1)

        self.PublishWebcamImage()

    def CreateCompressedImage(self, frame):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.Quality]
        msg.data = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tobytes()
        return msg

    def PublishWebcamImage(self):

        while not rospy.is_shutdown():
            ret, frame = self.st_VideoCapture.read()
            
            if ret:
                CompressedImageMsg = self.CreateCompressedImage(frame)
                self.st_Publisher.publish(CompressedImageMsg)
                cv2.imshow("Webcam", frame)
                cv2.waitKey(1)
            else:
                rospy.logwarn("Failed to capture image")

if __name__ == '__main__':
    rospy.init_node('PublishWebcamImage', anonymous=True)
    st_WebcamParser()
# import sys
# import cv2
# import rospy
# import numpy as np
# import time
# from sensor_msgs.msg import CompressedImage

# class st_WebcamParser:
#     def __init__(self):

#         self.st_Publisher = rospy.Publisher("/usb_cam/image/compressed", CompressedImage, queue_size=10)

#         self.CameraID = "/dev/video3"  # USB Port에 따라 가변적
#         self.Quality = 95
#         self.st_VideoCapture = cv2.VideoCapture(self.CameraID, cv2.CAP_V4L2)

#         # Maximum Resolution
#         self.st_VideoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)         # Original 640
#         self.st_VideoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)        # Original 480

#         if not self.st_VideoCapture.isOpened():
#             rospy.logerr("Failed to open camera")
#             sys.exit(1)

#         # 프레임 속도 측정을 위한 변수 초기화
#         self.frame_count = 0
#         self.start_time = time.time()

#         self.PublishWebcamImage()

#     def CreateCompressedImage(self, frame):
#         msg = CompressedImage()
#         msg.header.stamp = rospy.Time.now()
#         msg.format = "jpeg"
#         encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.Quality]
#         msg.data = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tobytes()
#         return msg

#     def PublishWebcamImage(self):

#         while not rospy.is_shutdown():
#             ret, frame = self.st_VideoCapture.read()
            
#             if ret:
#                 # 프레임 카운트 증가
#                 self.frame_count += 1

#                 # FPS 계산 (1초마다)
#                 elapsed_time = time.time() - self.start_time
#                 if elapsed_time >= 1.0:
#                     fps = self.frame_count / elapsed_time
#                     rospy.loginfo(f"FPS: {fps:.2f}")
#                     self.frame_count = 0
#                     self.start_time = time.time()

#                 CompressedImageMsg = self.CreateCompressedImage(frame)
#                 self.st_Publisher.publish(CompressedImageMsg)
#                 cv2.imshow("Webcam", frame)
#                 cv2.waitKey(1)
#             else:
#                 rospy.logwarn("Failed to capture image")

# if __name__ == '__main__':
#     rospy.init_node('PublishWebcamImage', anonymous=True)
#     st_WebcamParser()
