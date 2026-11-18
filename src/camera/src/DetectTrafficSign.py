#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import math
import rospy
import numpy as np
import torch
import copy
from sensor_msgs.msg import CompressedImage
from camera.msg import TrafficSign, TrafficSign_Arr,TrafficSign_bbox,N_image
from cv_bridge import CvBridgeError
import torchvision.transforms as transforms
from torchvision.ops import nms
import rospkg
import sys
import yaml
import os

#x:3.0 y:0 z:1.0
#1020 x 720, FOV:90

class st_SignDetection:
    def __init__(self):
        # Yaml
        self.RelativePath = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configuration.yaml")
        self.st_Config = self.LoadConfiguration(self.RelativePath)
    
        #For Pub
        self.st_Publisher=rospy.Publisher("/YOLOv8/TrafficSign",TrafficSign_Arr,queue_size=10)

        
        # Parameter
        self.model_path = None
        self.TrafficSignClass = []
        self.LoadParam()
        self.st_BGRImg=None
        self.st_SubscriberImage = None

        # GPU Based Object Detection
        self.device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model=YOLO(self.model_path).float().to(self.device) 
        self.transform = transforms.Compose([
            transforms.ToPILImage(),               # Convert OpenCV image to PIL image
            transforms.Resize((640, 640)),         # Resize image to fit model input    -> 이거 왜해요? 
            transforms.ToTensor(),                 # Convert image to tensor
            lambda x: x / 255,                     # Normalize pixel values from [0, 255] to [0, 1]
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Normalize
        ])

        self.rate = rospy.Rate(50)
        
        self.TrafficSignDetection()
        
    def LoadConfiguration(self, s_ConfigPath):
        with open(s_ConfigPath, 'r') as file:
            st_Config = yaml.safe_load(file)
        return st_Config        
    
    def LoadParam(self):
        model_choice = self.st_Config.get("ModelChoice", 0)

        if model_choice == 0:
            #ERP
            self.st_SubscriberImage = eval(self.st_Config["Parameters"]["ParamERP"])
            self.model_path = self.st_Config["Paths"]["ModelPathTSERP"]
            self.TrafficSignClass = eval(self.st_Config["TSClass"]["ERPClass"])

        elif model_choice == 1:
            #Morai Cams
            self.st_SubscriberImage = eval(self.st_Config["Parameters"]["ParamMoraiCam"])
            self.model_path = self.st_Config["Paths"]["ModelPathTSMorai"]
            self.TrafficSignClass = eval(self.st_Config["TSClass"]["MoraiCamClass"])

        else:
            print("error in GetParameterPath")
    
    # def GetModelPath(self):
    #     self.model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.st_Config["Paths"]["ModelPathTSERP"])
    #     self.model_path = os.path.abspath(self.model_path)  

    def ImageCallback_FILR(self,Image):
        self.st_BGRImg = np.ndarray(shape=(Image.height, Image.width, Image.n_channels),
                              dtype=eval(Image.dtype), buffer=Image.data)
        self.IMGSZ = [Image.width, Image.height]

    # For MORAI Sim    
    def ImageCallback_MORAI(self,Image):
        try:
            np_arr = np.frombuffer(Image.data, np.uint8)
            img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if img_bgr is not None:
                self.st_BGRImg=img_bgr
        except CvBridgeError as e:
            print(e)

    def TrafficSignDetection(self):

        while self.st_BGRImg is None:
            print("st_BGRImg is None")
            pass

        while not rospy.is_shutdown():

            print("----------- Start Object Detection -----------")

            st_CopyBGRImg = copy.deepcopy(self.st_BGRImg)

            #객체 탐지 수행
            st_PredictResult = self.model(st_CopyBGRImg)[0]

            #pub할 message 선언 -> cls만 주기
            PublishMsg = TrafficSign_Arr()

            for Result in st_PredictResult:

                st_Boxes=Result.boxes

                for st_Box in st_Boxes:

                    boxes = st_Box.xyxy[0]
                    f32_Confidence = st_Box.conf[0] #f32_Confidence extract

                    #boxes가 2D 텐서로 하게함
                    if boxes.dim()==1:
                        boxes = boxes.unsqueeze(0)

                    if f32_Confidence.dim()==0:
                        f32_Confidence = f32_Confidence.unsqueeze(0)

                    arst_TensorBoxe = boxes.clone().detach()
                    arst_TensorConfidence = f32_Confidence.clone().detach()

                    # Apply NMS
                    # keep = 중복되지 않은 박스의 인덱스 목록
                    keep = nms(arst_TensorBoxe, arst_TensorConfidence, iou_threshold=0.5) 

                    for i in keep:
                        if arst_TensorConfidence[i].item() > 0.5:

                            st_ValidBBox = arst_TensorBoxe[i].cpu().numpy()
                            st_ValidClsIndex = st_Boxes[i].cls.item()

                            # Make Publish Message
                            TS_BBox=TrafficSign_bbox()

                            TS_BBox.xMin = int(st_ValidBBox[0])
                            TS_BBox.yMin = int(st_ValidBBox[1])
                            TS_BBox.xMax = int(st_ValidBBox[2])
                            TS_BBox.yMax = int(st_ValidBBox[3])

                            if((TS_BBox.yMax-TS_BBox.yMin) / 2 > (TS_BBox.xMax-TS_BBox.xMin)):
                                continue

                            # 판단/제어랑 상의 이후 필요없다면 Msg 내에서도 빼고 코드 상에서도 수정할 것
                            TS_BBox.cls = int(st_ValidClsIndex)
                            TS_BBox.classname = self.TrafficSignClass[TS_BBox.cls]

                            PublishMsg.ar_Object.append(TS_BBox)
                            PublishMsg.u16_ObjCnt+=1

                            # Visualization
                            st_CopyBGRImg.flags.writeable = True 
                            cv2.rectangle(st_CopyBGRImg, (TS_BBox.xMin, TS_BBox.yMin), (TS_BBox.xMax, TS_BBox.yMax), (255, 0, 0), 3)

                            # Make Text Position
                            st_Position = [TS_BBox.xMin,TS_BBox.yMin]
                        
                            # Put Text on Image
                            if TS_BBox.cls < len(self.TrafficSignClass):
                                #TS_BBox.cls가 self.TrafficLightClass 리스트의 크기보다 크거나 같을 때 오류 발생
                                cv2.putText(st_CopyBGRImg, self.TrafficSignClass[TS_BBox.cls], st_Position, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
                            else:
                                print(f"Invalid class index {TS_BBox.cls}")                         
            
            self.st_Publisher.publish(PublishMsg)
            
            print("PublishMsg")
            print(PublishMsg)
            self.rate.sleep()
            print("----------- Finish Object Detection -----------")
            cv2.imshow("st_BGRImg",st_CopyBGRImg)
            cv2.waitKey(1)

        rospy.spin()
                                        
if __name__ == '__main__':
    rospy.init_node("DetecSign",anonymous=True)
    st_SignDetection()