#!/usr/bin/env python3
import cv2
import math
import rospy
import numpy as np
import copy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from camera.msg import raw4cam_arr, N_image
import yaml
import os

class OragneLine:
    def __init__(self):
        # MORAI Callback
        self.st_SubscribeImage = rospy.Subscriber("/image_jpeg/compressed", CompressedImage,self.callback_MORAI)
        
        #FLIR Callback
        # self.st_SubscribeImage=rospy.Subscriber("raw_image", N_image, self.callback_FILR)

        #Lidar Callback
        self.st_PlnCtrlSubscriber = rospy.Subscriber("/Lane_point", raw4cam_arr, self.LidarCallback)

        # For Lidar
        self.st_PublishFussionInfo = rospy.Publisher("/MiddleLineDetecion", raw4cam_arr, queue_size=10)
        
        self.st_BGRImg = None
        self.width = 1020
        self.height = 720
        self.orangeImage = None
        self.orangemask = None
        self.InOrangeLineIndex = []

        # ERP parameter
        # self.st_IntrinsicParameter = np.array([[   1390   ,      0    ,    1020   ,0], #1020
        #                                      [     0    ,    1390   ,     570   ,0],
        #                                      [     0    ,      0    ,      1    ,0]],dtype=np.float64) 
                                            
        # self.st_TranslationMatrix=np.array([[1,0,0,  0.53],
        #                                     [0,1,0,   0.2 ],
        #                                     [0,0,1, -0.65],
        #                                     [0,0,0,   1  ]])
        
        
        # For MORAI Sim
        self.width=1020
        self.height=720
        self.fov_degree=90
        self.fov_radians=math.radians(self.fov_degree / 2)
        self.focal_length=self.width / (2 * (math.tan(self.fov_radians)))
        
        self.st_IntrinsicParameter=np.array(
            [[self.focal_length,0,510,0],
            [0,self.focal_length,360,0],
            [0,0,1,0]])

        self.st_TranslationMatrix=np.array(
            # LiDAR 좌표계 - Camera 좌표계
            # LiDAR 좌표계 x=1.0, y=0.00, z=0.8
            # camera: x:0.5 y:0.00 z:1.00 
            # 1020 X 720, FOV:90
            
            [
                [1,0,0,0.5],
                [0,1,0,0.0],
                [0,0,1,-0.2],
                [0,0,0,1]
            ]
        )

        # Lidar
        self.u16_ObjCntLidar = 0
        self.ar_LidarObject3d = []
        self.ar_LidarObject2d = np.empty((0,2)) #LidarObject의 x,y,z 캘리브레이션 결과를 담을 배열

    # For MORAI Sim
    def callback_MORAI(self,msg):
        
        try:
            np_arr=np.frombuffer(msg.data,np.uint8)
            img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)

            if img_bgr is not None:
                self.st_OriginalImg = img_bgr
                self.st_BGRImg = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
        except CvBridgeError as e:
            print(e)
        
        self.WholeProcessing()

    # For FLIR callback
    # def callback_FILR(self,st_NImage):
    #     self.st_OriginalImg = np.ndarray(shape=(st_NImage.height, st_NImage.width, st_NImage.n_channels),
    #                         dtype=eval(st_NImage.dtype), buffer=st_NImage.data)
        
    #     self.st_BGRImg = cv2.cvtColor(self.st_OriginalImg,cv2.COLOR_BGR2HSV)
        
    #     self.IMGSZ = [st_NImage.width, st_NImage.height]

    #     self.WholeProcessing()

    # For Lidar Callback
    def LidarCallback(self, arst_LidarObject):
        self.u16_ObjCntLidar = arst_LidarObject.Num 
        self.ar_LidarObject3d = arst_LidarObject.raw_data

    def CalculateTransformationMatrix(self, roll, pitch, yaw):

        Rotation_x = np.array([[     1    ,      0        ,        0       ],
                                [     0    , math.cos(roll), -math.sin(roll)],
                                [     0    , math.sin(roll),  math.cos(roll)]])
        
        Rotation_y = np.array([[ math.cos(pitch) ,  0  ,  math.sin(pitch)  ],
                                [        0        ,  1  ,        0          ],
                                [-math.sin(pitch) ,  0  ,  math.cos(pitch)  ]])
        
        Rotation_z = np.array([[math.cos(yaw), -math.sin(yaw) ,  0   ],
                                [math.sin(yaw),  math.cos(yaw) ,  0   ],
                                [     0       ,       0        ,  1   ]])
        

        R_P = np.matmul(np.matmul(Rotation_z,Rotation_y),Rotation_x)

        R_P = self.MakeZero(R_P)

        R_P = np.array([[R_P[0][0],R_P[0][1],R_P[0][2],0],
                        [R_P[1][0],R_P[1][1],R_P[1][2],0],
                        [R_P[2][0],R_P[2][1],R_P[2][2],0],
                        [0,0,0,1]])

        return np.matmul(np.matmul(self.st_IntrinsicParameter, R_P), self.st_TranslationMatrix)
    
    def MakeZero(self, arr):
        arr[abs(arr)<=6.123234e-16] = 0
        return arr

    def WholeProcessing(self):
        self.st_OriginalImg = np.copy(self.st_OriginalImg) # img(이미지)가 수정 가능하지 않아서 발생함
        st_LineMsg = raw4cam_arr()

        if self.u16_ObjCntLidar != 0:
            self.LidarProcessing()
        
        if self.st_OriginalImg is not None:
            self.OragneLineDetection()
            
            if self.PntInOrangeLine():
                # print("self.InOrangeLineIndex", self.InOrangeLineIndex)
                for TrueLineIndex in self.InOrangeLineIndex:
                    point = self.ar_LidarObject2d[TrueLineIndex]
                    x, y = int(point[0]), int(point[1])

                    st_LineMsg.raw_data.append(self.ar_LidarObject3d[TrueLineIndex])
                    st_LineMsg.Num = len(self.InOrangeLineIndex)
                    
                    cv2.circle(self.st_OriginalImg, (x, y), 10, (0, 255, 0), -1)  # green

            cv2.imshow("self.st_OriginalImg", self.st_OriginalImg)
            cv2.imshow("orangeImage", self.orangeImage)
            cv2.imshow("orangemask", self.orangemask)
            cv2.waitKey(1)

        self.st_PublishFussionInfo.publish(st_LineMsg)

        self.InOrangeLineIndex = []
        self.st_OriginalImg = None
        self.u16_ObjCntLidar = 0
        self.ar_LidarObject3d = []
        self.ar_LidarObject2d = np.empty((0,2))
    
    def PntInOrangeLine(self):
        self.InOrangeLineIndex = []

        for index, point in enumerate(self.ar_LidarObject2d):
            x, y = int(point[0]), int(point[1])

            if self.orangemask.shape[0] > y and self.orangemask.shape[1] > x:
                if self.orangemask[y, x] == 255:  
                    self.InOrangeLineIndex.append(index)
        
        if self.InOrangeLineIndex:
            return True, self.InOrangeLineIndex
        
        return False, []

    def LidarProcessing(self):
        OriginalCalibrationImage = self.st_OriginalImg.copy()
        
        while self.st_BGRImg is None:
            print("-----------LidarProcessing self.st_BGRImg is None------------")
            pass

        FinalMatrix = self.CalculateTransformationMatrix(np.deg2rad(90.), np.deg2rad(-90.), 0)

        for s32_I in range(self.u16_ObjCntLidar):
            # Calculate Lidar center Point
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[s32_I].x, self.ar_LidarObject3d[s32_I].y, self.ar_LidarObject3d[s32_I].z, 1]])  # 동차 행렬 생성
            st_Lidar2Camera = np.matmul(FinalMatrix, np.transpose(st_LidarPosition3d))  # 최종 캘리브레이션 결과

            # st_Lidar2Image = [abs(int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0])-500), abs(int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0]))-240] # in ERP
            st_Lidar2Image = [abs(int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0])), abs(int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0]))] # in Morai
            self.ar_LidarObject2d = np.vstack((self.ar_LidarObject2d, st_Lidar2Image))

        for point in self.ar_LidarObject2d:
            x, y = int(point[0]), int(point[1])
            if OriginalCalibrationImage.shape[0] > y and OriginalCalibrationImage.shape[1] > x:
                 
                print("in OriginalCalibrationImage")
                print("x: ", x, " y: ", y)
                # cv2.circle(OriginalCalibrationImage, (x, y), 10, (0, 0, 255), -1)  # red
            
            else:
                print("There is no point in range")
        # cv2.imshow("OriginalCalibrationImage", OriginalCalibrationImage)
        # cv2.waitKey(1)


    def OragneLineDetection(self):
        while self.st_BGRImg is None:
            print("--------------self.st_BGRImg is None-----------------")
            pass
        
        st_FinalBGRImg = copy.deepcopy(self.st_BGRImg)

        lower_white = np.array([15, 100, 157])  # B_low, G_low, R_low 값
        upper_white = np.array([100, 255, 255])  # B_high, G_high, R_high 값
        self.orangemask = cv2.inRange(st_FinalBGRImg, lower_white, upper_white)
        
        # 팽창을 통해 마스크 확장 (커널 크기를 조절하여 팽창 정도를 설정)
        kernel = np.ones((9, 9), np.uint8) 
        self.orangemask = cv2.dilate(self.orangemask, kernel, iterations=1)  # 1번 팽창
        
        self.orangeImage = cv2.bitwise_and(st_FinalBGRImg, st_FinalBGRImg, mask=self.orangemask)


if __name__ == '__main__':
    rospy.init_node('OragneLine', anonymous=True)
    OragneLine()
    rospy.spin()