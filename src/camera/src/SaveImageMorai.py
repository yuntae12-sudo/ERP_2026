#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
import os

class SaveMoraiImage:
    def __init__(self):
        # MORAI Callback
        self.st_SubscribeImage = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_MORAI)
        
        self.st_BGRImg = None
        self.width = 1020
        self.height = 300

        self.image_save_path = "/home/autonav/Pictures"  # 저장할 경로를 설정해 주세요.
        self.image_count = 577  # 이미지 파일명을 위한 카운터 추가

        # Hz 측정을 위한 변수
        self.last_time = rospy.get_time()  # 마지막 콜백 호출 시간
        self.callback_count = 0  # 콜백 호출 횟수
        self.hz_interval = 10  # 몇 번 호출마다 Hz를 계산할지 설정

    # For MORAI Sim
    def callback_MORAI(self, msg):
        try:
            # 현재 시간 기록
            current_time = rospy.get_time()
            self.callback_count += 1

            # Hz 계산 (10번마다 한 번씩)
            if self.callback_count % self.hz_interval == 0:
                elapsed_time = current_time - self.last_time
                hz = self.hz_interval / elapsed_time
                print(f"Current Hz: {hz:.2f} Hz")
                self.last_time = current_time

            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img_bgr is not None:
                self.st_BGRImg = img_bgr

                # 이미지를 화면에 표시
                cv2.imshow('MORAI Image', self.st_BGRImg)
                cv2.waitKey(1)  # 1ms 동안 대기, 키보드 입력은 받지 않음
                
                # 이미지 저장
                image_file_path = os.path.join(self.image_save_path, f"Virtual_80_{self.image_count}.jpg")
                cv2.imwrite(image_file_path, self.st_BGRImg)
                print(f"Image saved at {image_file_path}")
                
                # 이미지 카운터 증가
                self.image_count += 1
        
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('SaveMoraiImage', anonymous=True)
    SaveMoraiImage()
    rospy.spin()
