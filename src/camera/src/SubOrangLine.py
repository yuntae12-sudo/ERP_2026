#!/usr/bin/env python3
import cv2
import math
import rospy
import numpy as np
import copy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from camera.msg import N_image, objsInfo, objInfo, coeff, lane, tunneloutpoint

class SubOrangLine:
    def __init__(self):
        # FLIR Callback
        self.st_SubscribeImage=rospy.Subscriber("raw_image", N_image, self.callback_FILR)
    
        #Lidar Callback
        self.st_PlnCtrlSubscriber = rospy.Subscriber("/LiDARcom", objsInfo, self.LidarCallback)
        
        self.st_OriginalImg = None
        self.width = 1020
        self.height = 720
        self.ar_LaneInfo = []

    # For FLIR callback
    def callback_FILR(self,st_NImage):
        self.st_OriginalImg = np.ndarray(shape=(st_NImage.height, st_NImage.width, st_NImage.n_channels),
                            dtype=eval(st_NImage.dtype), buffer=st_NImage.data)
        
        self.IMGSZ = [st_NImage.width, st_NImage.height]

        self.WholeProcessing()

    # For Lidar Callback
    def LidarCallback(self, arst_LidarObject):
        self.ar_LaneInfo = arst_LidarObject.lane
        print("arst_LidarObject.coeff")
        print(arst_LidarObject.coeff)

    def WholeProcessing(self):
        while self.st_OriginalImg is None:
            print("-------------self.st_OriginalImg is None------------------")
            pass




if __name__ == '__main__':
    rospy.init_node('SubOrangLine', anonymous=True)
    SubOrangLine()
    rospy.spin()