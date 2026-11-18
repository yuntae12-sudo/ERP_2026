#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import math
import rospy
import numpy as np
import torch
import copy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
import torchvision.transforms as transforms
from torchvision.ops import nms
# from Total_msgs.Camera import TrafficLight, TrafficLight_Arr,N_image
from camera.msg import TrafficLight, TrafficLight_Arr,N_image
import yaml
import os

class st_TrafficDetection:
    def __init__(self):
        # Yaml
        self.RelativePath = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configuration.yaml")
        self.st_Config = self.LoadConfiguration(self.RelativePath)

        #For Pub
        self.st_Publisher=rospy.Publisher("/YOLOv8/TrafficerLight_Arr",TrafficLight_Arr,queue_size=10)

        # WebCam
        # self.st_SubscribeImage=rospy.Subscriber("/usb_cam/image/compressed",CompressedImage,self.ImageCallback)
        
        # #MORAI Callback
        # self.st_SubscribeImage=rospy.Subscriber("/image_jpeg/compressed", CompressedImage,self.callback_MORAI)

        #FILR
        # self.st_SubscriberImage=rospy.Subscriber("raw_image", N_image,self.ImageCallback_FILR)

        # Parameter
        self.model_path = None
        self.LoadParam()
        self.TrafficLightClass = []
        self.st_BGRImg = None
        self.st_SubscribeImage = None

        self.device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model=YOLO(self.model_path).float().to(self.device) #->model_path 생기면 넣고 주석 풀기
        self.TrafficLightClass=['Green', 'LeftGreen','Red','RedLeft','RedYellow','Yellow']

        self.transform = transforms.Compose([
            transforms.ToPILImage(),               # Convert OpenCV image to PIL image
            transforms.Resize((640, 640)),         # Resize image to fit model input -> 이거 왜해주는거임?
            transforms.ToTensor(),                 # Convert image to tensor
            lambda x: x / 255,                     # Normalize pixel values from [0, 255] to [0, 1]
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Normalize
        ])

        self.rate = rospy.Rate(50) #TrafficLightDetection 메서드안의 루프가 초당 50번의 속도로 제어된 속도로 실행
        self.traffic_light_history={}
        self.object_tracker={}
        self.next_id=0
        self.window_size=20
        self.st_BGRImg=None
        
        self.TrafficLightDetection()

    def LoadConfiguration(self, s_ConfigPath):
        with open(s_ConfigPath, 'r') as file:
            st_Config = yaml.safe_load(file)
        return st_Config 
    
    def LoadParam(self):
        model_choice = self.st_Config.get("TLChoice", 2)

        if model_choice == 2:
            #ERP
            self.st_SubscriberImage = eval(self.st_Config["Parameters"]["ParamERPTL"])
            self.model_path = self.st_Config["Paths"]["ModelPathTLERPMorai"]
            self.TrafficLightClass = eval(self.st_Config["TLClass"]["ERPMoraiClass"])
        
        elif model_choice == 3:
            #Morai
            self.st_SubscriberImage = eval(self.st_Config["Parameters"]["ParamMoraiCam"])
            self.model_path = self.st_Config["Paths"]["ModelPathTLERPMorai"]
            self.TrafficLightClass = eval(self.st_Config["TLClass"]["ERPMoraiClass"])     
    
    # For MORAI Sim    
    def callback_MORAI(self,msg):
        try:
            np_arr=np.frombuffer(msg.data,np.uint8)
            img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if img_bgr is not None:
                self.st_BGRImg=img_bgr
        except CvBridgeError as e:
            print(e)

    # For WebCam
    def ImageCallback(self,st_CompressedImg):
        try:
            NpArr=np.frombuffer(st_CompressedImg.data,np.uint8)
            Img=cv2.imdecode(NpArr,cv2.IMREAD_COLOR)
            if Img is not None:
                self.st_BGRImg=Img
        except CvBridgeError as e:
            print(e)
    
    def TrafficLightDetection(self):
        while self.st_BGRImg is None:
            #print("--------------self.st_BGRImg is None-----------------")
            pass

        #st_BGRImg 생성까지 기다리기
        while not rospy.is_shutdown():

            print("----------------------Start Detect TL---------------------")
            st_CopyBGRImg = copy.deepcopy(self.st_BGRImg)

            #YOLOV8을 이용해서 예측 수행
            st_PredictResult = self.model(st_CopyBGRImg)[0]

            #message 선언
            PubMsg = TrafficLight_Arr()
            PubMsg.ObjectNum = 0

            for Result in st_PredictResult:
                st_Boxes=Result.boxes

                for st_Box in st_Boxes:
                    boxes=st_Box.xyxy[0] #bbbox list
                    confidence=st_Box.conf[0] #confidence extract

                    #boxes가 2D 텐서로 하게함
                    if boxes.dim()==1:
                        boxes=boxes.unsqueeze(0)

                    if confidence.dim()==0:
                        confidence=confidence.unsqueeze(0)

                    boxes_tensor = boxes.clone().detach()
                    confidence_tensor = confidence.clone().detach()

                    # Apply NMS
                    # keep = 중복되지 않은 박스의 인덱스 목록
                    keep = nms(boxes_tensor, confidence_tensor, iou_threshold=0.5)  # You can adjust the threshold
                    for i in keep:
                        if confidence_tensor[i].item() > 0.7:
                            valid_box=boxes_tensor[i].cpu().numpy()
                            # print("-----------valid_box-------------------")
                            # print(valid_box)
                            # print("-------------------------------------")
                            
                            valid_class_index = st_Boxes[i].cls.item()  # 유효한 박스의 클래스 인덱스
                            # print("-----------valid_class_index-------------------")
                            # print(valid_class_index)
                            # print("-------------------------------------------")
                            tmp = TrafficLight()
                            tmp.xMin = int(valid_box[0])
                            tmp.yMin = int(valid_box[1])
                            tmp.xMax = int(valid_box[2])
                            tmp.yMax = int(valid_box[3])
                            tmp.size = (tmp.xMax - tmp.xMin) * (tmp.yMax - tmp.yMin)
                            valid_class_index=int(valid_class_index)
                            tmp.cls = valid_class_index  # 클래스 인덱스 설정
                            tmp.classesname = self.TrafficLightClass[valid_class_index]

                            # 메시지에 트래픽 라이트 객체 추가
                            PubMsg.arst_TrafficLight.append(tmp)
                            PubMsg.ObjectNum += 1

                            #st_BGRImg에 Detection Result Visualization
                            cv2.rectangle(self.st_BGRImg, (tmp.xMin, tmp.yMin), (tmp.xMax, tmp.yMax), (255, 0, 0), 3)

                            #position
                            position=[tmp.xMin,tmp.yMin]
                            font=cv2.FONT_HERSHEY_SIMPLEX
                            fontScale=1
                            color=(0,255,255)
                            thick=2
                        
                            #put the text
                            if tmp.cls < len(self.TrafficLightClass):
                                #tmp.cls가 self.TrafficLightClass 리스트의 크기보다 크거나 같을 때 오류 발생
                                cv2.putText(self.st_BGRImg, self.TrafficLightClass[tmp.cls], position, font, fontScale, color, thick)
                            else:
                                print("Invalid class index ",tmp.cls)

            print("----------------PubMsg------------")
            print(PubMsg)
            print("---------------------------------------")

            self.st_Publisher.publish(PubMsg)
            print("----------------------End TL-------------------------")
            self.rate.sleep()

            cv2.imshow("DetectTrafficLight", self.st_BGRImg)
            cv2.waitKey(1)

        

if __name__ == '__main__':
    rospy.init_node('TrafficLightDetection', anonymous=True)
    st_TrafficDetection()
    rospy.spin()