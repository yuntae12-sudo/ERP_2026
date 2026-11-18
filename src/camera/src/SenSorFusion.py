#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import math
import rospy
import numpy as np
from std_msgs.msg import String
import torch
import copy
from sensor_msgs.msg import CompressedImage
from camera.msg import TrafficSign, TrafficSign_Arr,TrafficSign_bbox,object_msg_arr,object_msg,N_image, fusInfo, fussInfo
from cv_bridge import CvBridgeError
import torchvision.transforms as transforms
import os
import sys

class Fusion: 
    def __init__(self):
        #detectSign 코드
        self.st_SubDetectSign=rospy.Subscriber("/YOLOv8/TrafficSign",TrafficSign_Arr,self.DetectSignaObjectCallback)
        #Lidar cluestering.cpp
        self.PlnCtrlSubscriber = rospy.Subscriber("/4_Clustering_OMA", object_msg_arr, self.LidarCallback)
        
        # #MORAI image sub
        # self.st_moraiSub=rospy.Subscriber("/image_jpeg/compressed", CompressedImage,self.callback_MORAI)
        
        # FILR
        self.st_SubscribeImage=rospy.Subscriber("raw_image", N_image,self.callback_FILR)
        
        #pub to 판단/제어
        self.st_PublishFussionInfo=rospy.Publisher("FusionInfo",fussInfo, queue_size=10)
        
        self.width=1020
        self.height=720
        self.fov_degree=90
        self.fov_radians=math.radians(self.fov_degree / 2)
        self.focal_length=self.width / (2 * (math.tan(self.fov_radians)))
        
        self.s64_ThresholdIoU = 0.005
        self.TSSize = []
        self.bins = np.arange(0,12.0,0.5)
        self.first_index = 0
        
        self.IntervalValueMap = {
            (0.0, 2.0) : [90000 , math.inf],

            (2.0, 3.0) : [53800, 90000],

            (3.0, 4.0) : [32200, 53800],

            (4.0, 5.0) : [21310, 32200],

            (5.0, 6.0) : [14640, 21310],

            (6.0, 7.0) : [11230, 14640],

            (7.0, 8.0) : [8830, 11230],

            (8.0, 9.0) : [6880, 8830],

            (9.0, 10.0) : [5770, 6880],

            (10.0, 11.0) : [4900, 5770],         

            (11.0, 12.0) : [4160,4900],

            (12.0, 20.0) : [0, 4160]
        }


        # parameter
        self.st_IntrinsicParameter = np.array([[   1390   ,      0    ,    1020   ,0], #1020
                                             [     0    ,    1390   ,     570   ,0],
                                             [     0    ,      0    ,      1    ,0]],dtype=np.float64) 
                                            
        self.st_TranslationMatrix=np.array([[1,0,0,  0.53],
                                            [0,1,0,   0.2 ],
                                            [0,0,1, -0.65],
                                            [0,0,0,   1  ]])

        # # For MORAI Sim
        # self.st_IntrinsicParameter=np.array(
        #     [[self.focal_length,0,510,0],
        #     [0,self.focal_length,360,0],
        #     [0,0,1,0]])

        # self.st_TranslationMatrix=np.array(
        #     #LiDAR 좌표계 - Camera 좌표계
        #     #real test LiDAR 좌표계 x=2.92, y=0.00, z=0.66
        #     #camera
        #     # x:3.00 y:0.00 z:1.00 # 1020 X 720, FOV:90
        #     [
        #         [1,0,0,-0.08],
        #         [0,1,0,0],
        #         [0,0,1,-0.34],
        #         [0,0,0,1]
        #     ]
        # )

        # Hz
        self.s64_StartTime = 0
        self.s64_EndTime = 0

        # Camera
        self.st_DecodedImage = None
        self.ObjCntCamera = 0
        self.ar_Object = []
        self.CameraBBoxSize = []
        self.arst_CameraMinMax = []

        # Lidar
        self.u16_ObjCntLidar = 0
        self.ar_LidarObject3d = []
        self.ar_LidarObject2d = np.empty((0, 2)) 
        self.arst_LidarObjectMinMax = np.empty((0, 4))

        # Fusion
        self.ars32_Cases = []

        #exception
        self.st_ValidFusionInfo = [False] * 6
        self.ClassList = []

        self.st_Colors = {
            0: (255, 0, 0),   # UL: Blue
            1: (0, 255, 0),   # UM: Green
            2: (0, 0, 255),   # UR: Red
            3: (255, 255, 0), # ML: Cyan
            4: (255, 0, 255), # MM: Magenta
            5: (0, 255, 255), # MR: Yellow
            6: (128, 0, 0),   # DL: Dark Blue
            7: (0, 128, 0),   # DM: Dark Green
            8: (0, 0, 128)    # DR: Dark Red
        }

    def callback_FILR(self,st_NImage):
        self.st_DecodedImage = np.ndarray(shape=(st_NImage.height, st_NImage.width, st_NImage.n_channels),
                              dtype=eval(st_NImage.dtype), buffer=st_NImage.data)
        self.IMGSZ = [st_NImage.width, st_NImage.height]
        
    def callback_MORAI(self, st_CompressedImage):
        np_arr = np.frombuffer(st_CompressedImage.data, np.uint8)
        self.st_DecodedImage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def LidarCallback(self, arst_LidarObject):
        # print("--------------------------LiDAR Callback-----------------------------")
        self.u16_ObjCntLidar = arst_LidarObject.objc
        self.ar_LidarObject3d = arst_LidarObject.object_msg_arr
        self.st_DetectedLiDARInfo = [False] * self.u16_ObjCntLidar
        # print("u16_ObjCntLidar ",self.u16_ObjCntLidar)
        # print("self.ar_LidarObject3d",self.ar_LidarObject3d)
        # print("-----------------------------------------------------------------")

    def DetectSignaObjectCallback(self, arst_CamObject):
        self.u16_ObjCntCamera = arst_CamObject.u16_ObjCnt
        self.arst_CamObject = arst_CamObject.ar_Object

        # for st_BBox in self.arst_CamObject:

        #     s32_CamBBoxSize = (st_BBox.xMax - st_BBox.xMin) * (st_BBox.yMax - st_BBox.yMin)
        #     for (lower_bound, upper_bound), values in self.IntervalValueMap.items():
        #         if(s32_CamBBoxSize >= values[0] and s32_CamBBoxSize <= values[1]):
        #             self.arst_CameraMinMax.append([lower_bound,upper_bound])
                    
        #             print("Cls: ", st_BBox.classname, ", Size: ", s32_CamBBoxSize, ", Interval: ", lower_bound, "~", upper_bound) 

        # self.CameraBBoxSize.append(CamBBox)
        self.WholeProcessiong()    

    def DrawCameraBBoxes(self):

        st_TmpImage = self.st_DecodedImage.copy()  # 이미지를 복사하여 읽기/쓰기 가능하도록 설정

        for st_CamObj in self.arst_CamObject:

            s32_xMin = st_CamObj.xMin
            s32_xMax = st_CamObj.xMax
            s32_yMin = st_CamObj.yMin
            s32_yMax = st_CamObj.yMax

            s_ClassName = st_CamObj.classname

            cv2.rectangle(st_TmpImage, (s32_xMin, s32_yMin), (s32_xMax, s32_yMax), (255, 0, 0), 2)
            cv2.putText(st_TmpImage, s_ClassName, (s32_xMin, s32_yMin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
        
        self.st_DecodedImage = st_TmpImage  # 변경된 이미지를 저장
    
    def CheckCase(self, st_ObjectMsg: object_msg):
    # U = up, M = middle, D = down, L = left, R = right
    # UL = 0, UM = 1, UR = 2, ML = 3, MM = 4, ML = 5, DL = 6, DM = 7, DR = 8
    # x is always positive, each case depends on y,z sign
    
        #s32_Case = 2
        
        if st_ObjectMsg.zMin >= 0 and st_ObjectMsg.zMax >= 0:                       # U?
            if st_ObjectMsg.yMin >= 0 and st_ObjectMsg.yMax >= 0: s32_Case = 0          # UL
            elif st_ObjectMsg.yMin <= 0 and st_ObjectMsg.yMax >= 0: s32_Case = 1        # UM
            else: s32_Case = 2                                        # UR
        elif st_ObjectMsg.zMin <= 0 and st_ObjectMsg.zMax >= 0:                     # M?
            if st_ObjectMsg.yMin >= 0 and st_ObjectMsg.yMax >= 0: s32_Case = 3          # ML
            elif st_ObjectMsg.yMin <= 0 and st_ObjectMsg.yMax >= 0: s32_Case = 4        # MM
            else: s32_Case = 5                                        # MR
        elif st_ObjectMsg.zMin <= 0 and st_ObjectMsg.zMax <= 0:                     # D?
            if st_ObjectMsg.yMin >= 0 and st_ObjectMsg.yMax >= 0: s32_Case = 6          # DL
            elif st_ObjectMsg.yMin <= 0 and st_ObjectMsg.yMax >= 0: s32_Case = 7        # DM
            else: s32_Case = 8 

        return s32_Case

    def DrawCalibratedLiDARObjs(self,image, objects, cases):

        colors = {
            0: (255, 0, 0),   # UL: Blue
            1: (0, 255, 0),   # UM: Green
            2: (0, 0, 255),   # UR: Red
            3: (255, 255, 0), # ML: Cyan
            4: (255, 0, 255), # MM: Magenta
            5: (0, 255, 255), # MR: Yellow
            6: (128, 0, 0),   # DL: Dark Blue
            7: (0, 128, 0),   # DM: Dark Green
            8: (0, 0, 128)    # DR: Dark Red
        }
        
        image = image.copy() 
        
        for (x, y), case in zip(objects, cases):
            color = colors.get(case, (255, 255, 255))  
            cv2.circle(image, (int(x), int(y)), 5, color, -1)
            cv2.putText(image, str(case), (int(x) + 10, int(y) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return image

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
    
    def LidarProcessing(self):
        
        while self.st_DecodedImage is None:
            rospy.sleep(0.1) # 0.1초 동안 대기
            
        st_TmpImage = self.st_DecodedImage.copy()

        FinalMatrix = self.CalculateTransformationMatrix(np.deg2rad(90.), np.deg2rad(-90.), 0.)

        for s32_I in range(self.u16_ObjCntLidar):

            # Calculate LiDAR Center Pnt

            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[s32_I].x, self.ar_LidarObject3d[s32_I].y, self.ar_LidarObject3d[s32_I].z, 1]])  # 동차 행렬 생성
            st_Lidar2Camera = np.matmul(FinalMatrix, np.transpose(st_LidarPosition3d))  # 최종 캘리브레이션 결과
            # st_Lidar2Image = [abs(int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0])), abs(int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0]))] #in MORI
            st_Lidar2Image = [abs(int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0])-500), abs(int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0]))-240] #in ERP
            self.ar_LidarObject2d = np.vstack((self.ar_LidarObject2d, st_Lidar2Image))  # 수직으로 최종 pixel 좌표와 obj 쌓음

            # Calculate LiDAR Object MinMax

            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[s32_I].xMax,self.ar_LidarObject3d[s32_I].yMax,self.ar_LidarObject3d[s32_I].zMax,1]])
            st_Lidar2Camera = np.matmul(FinalMatrix, np.transpose(st_LidarPosition3d))
            # st_LidarObjectMinMax = [abs(int(st_Lidar2Camera[0][0]/st_Lidar2Camera[2][0])),abs(int(st_Lidar2Camera[1][0]/st_Lidar2Camera[2][0]))] #in MORI 
            st_LidarObjectMinMax = [abs(int(st_Lidar2Camera[0][0]/st_Lidar2Camera[2][0]))-500,abs(int(st_Lidar2Camera[1][0]/st_Lidar2Camera[2][0]))-240] #in ERP

            
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[s32_I].xMin,self.ar_LidarObject3d[s32_I].yMin,self.ar_LidarObject3d[s32_I].zMin,1]])
            st_Lidar2Camera = np.matmul(FinalMatrix, np.transpose(st_LidarPosition3d))

            #in MORI
            # st_LidarObjectMinMax.append(abs(int(st_Lidar2Camera[0][0]/st_Lidar2Camera[2][0])))
            # st_LidarObjectMinMax.append(abs(int(st_Lidar2Camera[1][0]/st_Lidar2Camera[2][0])))

            # in ERP
            st_LidarObjectMinMax.append(abs(int(st_Lidar2Camera[0][0]/st_Lidar2Camera[2][0]))-500)
            st_LidarObjectMinMax.append(abs(int(st_Lidar2Camera[1][0]/st_Lidar2Camera[2][0]))-240)

            self.arst_LidarObjectMinMax = np.vstack((self.arst_LidarObjectMinMax, st_LidarObjectMinMax))
            
            # # LiDAR Object의 Case 및 BBox 표현
            # cv2.rectangle(st_TmpImage, 
            #           (int(st_LidarObjectMinMax[0]), int(st_LidarObjectMinMax[1])), 
            #           (int(st_LidarObjectMinMax[2]), int(st_LidarObjectMinMax[3])), 
            #           (255, 255, 0), 2)

        self.st_DecodedImage = st_TmpImage

    def MakeZero(self, arr):
        arr[abs(arr)<=6.123234e-16] = 0
        return arr

    def LiDARBasedIou(self,LidarBBox:list, CameraBBox:TrafficSign_bbox):
        
        if((max(LidarBBox[2],CameraBBox.xMax)-min(LidarBBox[0],CameraBBox.xMin))>(CameraBBox.xMax - CameraBBox.xMin)+(LidarBBox[2] - LidarBBox[0]) 
           or (max(LidarBBox[3],CameraBBox.yMax)-min(LidarBBox[1],CameraBBox.yMin))>(CameraBBox.yMax - CameraBBox.yMin)+(LidarBBox[3] - LidarBBox[1])):
            IoU = 0

        else:
            Lidarbbox=(LidarBBox[2] - LidarBBox[0])*(LidarBBox[3] - LidarBBox[1])

            intersection_xmin = max(CameraBBox.xMin,LidarBBox[0])
            intersection_xmax = min(CameraBBox.xMax,LidarBBox[2])
            intersection_ymin = max(CameraBBox.yMin,LidarBBox[1])
            intersection_ymax = min(CameraBBox.yMax,LidarBBox[3])

            intersection_bbox_size = (intersection_xmax-intersection_xmin)*(intersection_ymax-intersection_ymin)
            IoU = intersection_bbox_size / (Lidarbbox)

        return IoU

    def DrawFusionBBox(self, LidarBBox:list , CameraBBox: TrafficSign_bbox):
        cv2.rectangle(self.st_DecodedImage,(int(LidarBBox[0]),int(LidarBBox[1])),(int(LidarBBox[2]),int(LidarBBox[3]+30)),(0,255,  0),2)
        cv2.rectangle(self.st_DecodedImage,(CameraBBox.xMin,CameraBBox.yMin),(CameraBBox.xMax,CameraBBox.yMax),(0,255,255),2)             

    def LiDarBasedFusionProcessing(self, st_FusionMsg: fussInfo):
        st_TmpImage = self.st_DecodedImage.copy()

        print("----------Fusion Processing function----------")

        for st_BBox in self.arst_CamObject:

            s32_CamBBoxSize = (st_BBox.xMax - st_BBox.xMin) * (st_BBox.yMax - st_BBox.yMin)

            for (lower_bound, upper_bound), values in self.IntervalValueMap.items():
                if(s32_CamBBoxSize >= values[0] and s32_CamBBoxSize <= values[1]):
                    self.arst_CameraMinMax.append([lower_bound,upper_bound])
                    
                    print("Cls: ", st_BBox.classname, ", Size: ", s32_CamBBoxSize, ", Interval: ", lower_bound, "~", upper_bound)   # 현재 카메라에 넘어오는 bbox 크기 측정      
        
        # print("self.arst_CamObject\n", self.arst_CamObject)

        TmpValid = []
        TmpNum = 0
        NoValuesTmpList = []

        TmpPubList = []

        for CameraIndex, CameraBBox in enumerate(self.arst_CamObject): 

            for LiDarIndex, LiDarBBox in enumerate(self.arst_LidarObjectMinMax):
                
                # print("CameraIndex: ", CameraIndex, "LiDarIndex: ", LiDarIndex)
                # print("CameraBBox: ", CameraBBox)
                # print("LiDarBBox: ", LiDarBBox)

                s64_IoU = self.LiDARBasedIou(LiDarBBox, CameraBBox) #iou 계산하고
                
                # print("s64_IoU: ", s64_IoU)
                
                if s64_IoU > self.s64_ThresholdIoU:
                    TmpPubList.append([s64_IoU, CameraIndex, LiDarIndex])
                    # cv2.rectangle(st_TmpImage,(int(LiDarBBox[0]),int(LiDarBBox[1])), (int(LiDarBBox[2]), int(LiDarBBox[3])), (0,255,200),2) #머스타드 색깔

        self.ClassList = self.FilterIngClasses(TmpPubList)
        TmpPubListSorted = sorted(TmpPubList, key=lambda x: x[0],reverse=True) # TmpPubList_Sorted = [s64_IoU,CameraIndex,LiDarIndex]
        
        # print("TmpPubListSorted\n", TmpPubListSorted)
        # print("self.arst_CameraMinMax\n", self.arst_CameraMinMax)
        # print("TmpPubList\n", TmpPubList)

        for i, TSList in enumerate(TmpPubListSorted):

            Lidar_Distance = self.ar_LidarObject3d[TSList[2]].x     # Matching 된 LiDAR의 X Distance
            print("Lidar_Distance: ",Lidar_Distance)
            print("Camera Distanc: ", self.arst_CameraMinMax[TSList[1]])
            if(Lidar_Distance >= (self.arst_CameraMinMax[TSList[1]][0] - 1.5) and Lidar_Distance <= (self.arst_CameraMinMax[TSList[1]][1] + 1.5)):
                TmpNum = TmpNum + 1
                TmpValid.append(TSList)
            else:
                NoValuesTmpList.append(TSList)
                    
        CheckFlag = self.CheckPusMsg(TmpValid)
        
        # print("TmpValid \n", TmpValid)
        # print("NoValuesTmpList \n", NoValuesTmpList)

        if CheckFlag == True:
            st_FusionMsg.objNum = TmpNum

            for TS in TmpValid:
                self.MakePublishMsg(TS, st_FusionMsg)
                cv2.rectangle(st_TmpImage,(int(self.arst_LidarObjectMinMax[TS[2]][0]),int(self.arst_LidarObjectMinMax[TS[2]][1])), (int(self.arst_LidarObjectMinMax[TS[2]][2]), int(self.arst_LidarObjectMinMax[TS[2]][3])), (0,0,255),2) #빨간 색깔

        else:
            self.FalseCase(TmpValid, st_FusionMsg, st_TmpImage)
        
        self.st_DecodedImage = st_TmpImage

    def MakePublishMsg(self, TS, st_FusionMsg):
        tmp = fusInfo()
        tmp.posX = self.ar_LidarObject3d[TS[2]].x
        tmp.posY = self.ar_LidarObject3d[TS[2]].y
        tmp.posZ = self.ar_LidarObject3d[TS[2]].z
        tmp.size = (self.ar_LidarObject3d[TS[2]].xMax - self.ar_LidarObject3d[TS[2]].xMin)*(self.ar_LidarObject3d[TS[2]].yMax - self.ar_LidarObject3d[TS[2]].yMin)
        tmp.distance = math.sqrt(tmp.posX**2 + tmp.posY**2)
        tmp.classes = self.ClassToIntPubMsg(self.arst_CamObject[TS[1]].classname)

        st_FusionMsg.data.append(tmp)

    def ClassToInt(self,class_name):
        # [A1, A2, A3, B1, B2, B3]
        # 클래스 이름과 대응하는 숫자 값을 매핑하는 딕셔너리 생성
        class_to_int_map = {
            "A1": 0,
            "A2": 1,
            "A3": 2,
            "B1": 3,
            "B2": 4,
            "B3": 5
        }

        return class_to_int_map.get(class_name, -1)  # 잘못된 이름에 대해 -1 반환
    
    def ClassToIntPubMsg(self,class_name):
        if class_name == 'A1':
            return 101
        elif class_name == 'A2':
            return 102
        elif class_name == 'A3':
            return 103
        elif class_name == 'B1':
            return 111
        elif class_name == 'B2':
            return 112
        elif class_name == 'B3':
            return 113

    def FalseCase(self, TmpValid, st_FusionMsg, st_TmpImage):


        st_FusionInfo = [[] for _ in range(6)]

        TmpValid.sort(key=lambda x: x[1])

        for tmpvalid in TmpValid:
            TmpClassName = self.arst_CamObject[tmpvalid[1]].classname
            TmpClassInt = self.ClassToInt(TmpClassName)
            self.st_ValidFusionInfo[TmpClassInt] = True  # self.st_ValidFusionInfo를 True로 바꿈
            
            st_FusionInfo[TmpClassInt].append(tmpvalid)
            # st_FusionInfo[tmpvalid[1]].append(tmpvalid)


        while(any(self.st_ValidFusionInfo)):

            for Index, TSInfo in enumerate(st_FusionInfo):
                RemoveList = []

                # print("Index\n",Index)
                # print("self.st_ValidFusionInfo\n",self.st_ValidFusionInfo)

                if len(TSInfo) != 0 and self.st_ValidFusionInfo[Index]:
                    CameraIndex = TSInfo[0][1]
                    CameraIndexTS = self.ClassToInt(self.arst_CamObject[CameraIndex].classname)
                    LidarIndex = TSInfo[0][2]

                    if (self.st_DetectedLiDARInfo[LidarIndex] == True):
                        st_FusionInfo[CameraIndexTS].pop(0)
                        break

                    for i in range(Index+1, 6):

                        if len(st_FusionInfo[i]) != 0 and LidarIndex == st_FusionInfo[i][0][2]:
                            if TSInfo[0][0] < st_FusionInfo[i][0][0]:
                                RemoveList.append(CameraIndex)

                                CameraIndex = st_FusionInfo[i][0][1]
                                LidarIndex = st_FusionInfo[i][0][2]

                            else:
                                RemoveList.append(st_FusionInfo[i][0][1])
                    
                    st_FusionMsg.objNum += 1
                    # self.MakePublishMsg(TS, st_FusionMsg)
                    TmpTS = []
                    TmpTS.append(0)
                    TmpTS.append(CameraIndex)
                    TmpTS.append(LidarIndex)
                    self.MakePublishMsg(TmpTS, st_FusionMsg)

                    cv2.rectangle(st_TmpImage, (int(self.arst_LidarObjectMinMax[LidarIndex][0]),int(self.arst_LidarObjectMinMax[LidarIndex][1]))
                                  ,(int(self.arst_LidarObjectMinMax[LidarIndex][2]),int(self.arst_LidarObjectMinMax[LidarIndex][3])),(0,0,0),5)
                    
                    self.st_ValidFusionInfo[CameraIndexTS] = False
                    self.st_DetectedLiDARInfo[LidarIndex] = True

                    for CamIndex in RemoveList:
                        st_FusionInfo[self.ClassToInt(self.arst_CamObject[CamIndex].classname)].pop(0)

                    if Index != CameraIndexTS:
                        break

                if len(TSInfo) == 0 and self.st_ValidFusionInfo[Index]:
                    self.st_ValidFusionInfo[Index] = False

        # print("st_FusionMsg")
        # print(st_FusionMsg)


    def CheckPusMsg(self,TmpValid):
        false_list = [False] * len(self.ar_LidarObject3d)
        TmpValidClasses = [self.arst_CamObject[tmpvalid[1]].classname for tmpvalid in TmpValid]

        if TmpValidClasses == self.ClassList:
            #ClassList 랑 TmpValidClasses랑 맞을 때

            for Valid in TmpValid:
                if false_list[self.ar_LidarObject3d[Valid[2]].idx] == False:
                    false_list[self.ar_LidarObject3d[Valid[2]].idx] = True
                
                else:
                    return False
        else:
            return False
        
        return True

    def FilterIngClasses(self, TmpPubList):
        unique_elements = []
        first_elements = set()

        for item in TmpPubList:
            CameraIndex = item[1]
            classname = self.arst_CamObject[CameraIndex].classname

            if classname not in first_elements:
                unique_elements.append(classname)
                first_elements.add(classname)

        return unique_elements
    
    def SaveFusionInfoToFile(self, st_FusionMsg):
        # 저장할 파일 경로 설정
        file_path = os.path.join(os.path.expanduser('~'), 'fusion_output.txt')

        with open(file_path, 'a') as file:  # 파일을 append 모드로 열기
            file.write("Fusion Data:\n")
            file.write(f"Number of Objects: {st_FusionMsg.objc}\n")
            
            for obj in st_FusionMsg.object_msg_arr:
                file.write(f"Object ID: {obj.idx}, Class: {obj.classes}, Position: ({obj.x}, {obj.y}, {obj.z})\n")
            
            file.write("\n")  # 각 데이터 간 간격을 위해 빈 줄 추가

    def WholeProcessiong(self):

        st_FusionMsg = fussInfo()

        if self.u16_ObjCntLidar != 0:
            self.LidarProcessing()
            self.LiDarBasedFusionProcessing(st_FusionMsg)
        
        self.st_PublishFussionInfo.publish(st_FusionMsg)
        # self.SaveFusionInfoToFile(st_FusionMsg)

        if self.st_DecodedImage is not None:

            self.DrawCameraBBoxes()
            cv2.imshow("Sensor Fusion", self.st_DecodedImage)
            cv2.waitKey(1)

        # Member Variable Reset
        self.u16_ObjCntCamera = 0
        self.arst_CamObject = []
        self.u16_ObjCntLidar = 0
        self.ar_LidarObject3d = []
        self.ar_LidarObject2d = np.empty((0, 2)) 
        self.arst_LidarObjectMinMax = np.empty((0, 4))
        self.ars32_Cases = []
        self.TSSize = []
        self.CameraBBoxSize = []
        self.arst_CameraMinMax = []

if __name__ =='__main__':
    rospy.init_node("sensorFusion",anonymous=True)
    Fusion()
    rospy.spin()