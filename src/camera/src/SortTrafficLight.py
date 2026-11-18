#!/usr/bin/env python3
import rospy
from camera.msg import TrafficLight_Arr,signalInfo

class SortingTrafficLight:
    def __init__(self):
        self.st_Subscriber=rospy.Subscriber("/YOLOv8/TrafficerLight_Arr",TrafficLight_Arr,self.SortTrafficLightInfo)
        self.st_Publisher=rospy.Publisher("/data_process/Camcom",signalInfo,queue_size=10)        
        self.arst_TrafficLight = []
        self.ars32_Classes = []
        self.cnt = 0

        self.TmpsignIndex = 0
        self.Tmpclassname = "None"

    def SortTrafficLightInfo(self, data):
        msg = signalInfo()
        sorted_traffic_lights = []

        # 내림차순(큰 -> 작)으로 정렬하여 리스트에 추가
        sorted_traffic_lights.extend(sorted(data.arst_TrafficLight, key=lambda x: x.size, reverse=True))

        if len(data.arst_TrafficLight) == 0:
            msg.signIndex = -1
            msg.classname = "None"

        elif (len(data.arst_TrafficLight) != 0) and len(sorted_traffic_lights) == 1:
            msg.signIndex = sorted_traffic_lights[0].cls
            msg.classname = sorted_traffic_lights[0].classesname

            self.TmpsignIndex = msg.signIndex
            self.Tmpclassname = msg.classname

            
        else: # data.arst_TrafficLight가 비어있지 않음 and sorted_traffic_lights에서 정렬할 게 있음
            xMin_sorted = []
            for index in range(len(sorted_traffic_lights) - 1):
                if abs(sorted_traffic_lights[index].size - sorted_traffic_lights[index + 1].size) < 500:
                    self.ars32_Classes.append(sorted_traffic_lights[index])
                    self.ars32_Classes.append(sorted_traffic_lights[index + 1])

            if len(xMin_sorted) != 0:

                # xMin을 기준으로 오름차순 정렬
                xMin_sorted = sorted(self.ars32_Classes, key=lambda x: x.xMin, reverse=False)

                msg.signIndex = xMin_sorted[0].cls
                msg.classname = xMin_sorted[0].classesname
                
                self.TmpsignIndex = msg.signIndex
                self.Tmpclassname = msg.classname

            else: 
                #len(xMin_sorted) ==0 이면 -> 앞에 신호등이랑 뒤에 신호등이 감지되어서 size 차이가 엄청 날 때
                msg.signIndex = sorted_traffic_lights[0].cls
                msg.classname = sorted_traffic_lights[0].classesname

                self.TmpsignIndex = msg.signIndex
                self.Tmpclassname = msg.classname

        if msg.classname == "None":
            self.cnt += 1

            if self.cnt == 1:
                msg.signIndex = self.TmpsignIndex
                msg.classname = self.Tmpclassname

            else:
                msg.signIndex = -1
                msg.classname = "None"

            self.st_Publisher.publish(msg)

        else:
            self.st_Publisher.publish(msg)
            self.cnt = 0

        self.ars32_Classes = []
        
if __name__=='__main__':
    rospy.init_node('TrafficLightInfo')
    SortingTrafficLight()
    rospy.spin()
