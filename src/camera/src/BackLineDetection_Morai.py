import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from camera.msg import LaneInfo
from cv_bridge import CvBridgeError
import copy
import time
import math
import random


class LaneInformation:
    def __init__(self, slope=0.0, intercept=0.0, coords=None, angle=0.0):
        self.f32_Slope = slope
        self.f32_InterceptY = intercept
        self.arst_Coords = coords if coords is not None else []  # 리스트가 기본값인 경우 None 체크
        self.f32_Angle = angle


class BackLaneDetection():
    def __init__(self):
        self.st_SubscribeImage = rospy.Subscriber("/image_jpeg/compressed", CompressedImage,self.callback_MORAI)
        self.st_Publisher = rospy.Publisher("OptimalLane", LaneInfo, queue_size=10)

        self.st_BGRImg = None
        self.width = 1024
        self.height = 576

        self.OptimalLine = None
        self.b_IsFirstLine = True
        self.s32_CntNoDetection = 0

        self.PubNum = 0
        self.LaneDetection()

    # For Morai
    def callback_MORAI(self,msg):
        try:
            np_arr=np.frombuffer(msg.data,np.uint8)
            img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if img_bgr is not None:
                self.st_BGRImg=img_bgr
        except CvBridgeError as e:
            print(e)

    # For Webcam
    # def ImageCallback(self, st_CompressedImg):
    #     try:
    #         NpArr = np.frombuffer(st_CompressedImg.data, np.uint8)
    #         Img = cv2.imdecode(NpArr, cv2.IMREAD_COLOR)

    #         if Img is not None:
    #             self.st_BGRImg = Img

    #     except CvBridgeError as e:
    #         print(e)

    def LaneDetection(self):

        while self.st_BGRImg is None:
            print("--------------self.st_BGRImg is None-----------------")
            pass

        while not rospy.is_shutdown():
            st_PubImgae = None
            st_FinalBGRImg = copy.deepcopy(self.st_BGRImg)
            st_GroupImage = self.st_BGRImg.copy()
            st_AverageImage = self.st_BGRImg.copy()
            st_HoughImage = self.st_BGRImg.copy()
            st_PubImgae = self.st_BGRImg.copy()

            lower_white = np.array([149,0,0])
            upper_white = np.array([255,255,255])

            white_mask = cv2.inRange(st_FinalBGRImg, lower_white, upper_white)
            white_image = cv2.bitwise_and(st_FinalBGRImg, st_FinalBGRImg, mask=white_mask)

            Gray = cv2.cvtColor(white_image, cv2.COLOR_BGR2GRAY)

            kernel = np.ones((3, 3), np.uint8)
            closing = cv2.morphologyEx(Gray, cv2.MORPH_CLOSE, kernel)

            # Blur = cv2.GaussianBlur(closing, (9, 9), sigmaX = 10, sigmaY = 30)

            Edges = cv2.Canny(closing, 700, 700)
            lines = cv2.HoughLinesP(Edges, 1, np.pi/180, threshold=60, minLineLength=100, maxLineGap=50)

            GroupedLines = []

            #### Hough Line 검출 후 후처리 -> Clustering
            if lines is not None:

                LineNum = 0
                for line_idx, line in enumerate(lines):
                    LineNum += 1
                    
                    # Line Info -> List
                    line_coords = line[0].tolist()
                    # cv2.line(st_HoughImage, (line_coords[0], line_coords[1]), (line_coords[2], line_coords[3]), (0, 122, 122), 10)

                    st_Line = self.CalculateLinesDetails(line_coords)  # Lane 정보 추출, 만약 수평 차선의 경우 Slope = None
                    # HoughAngle, slope, intercept = self.CalculateLinesDetails(line_coords)  # Lane 정보 추출

                    # NoneType 검사를 추가
                    if st_Line is None:
                        continue  # 기울기가 무한대인 경우 (None 반환 시) 건너뜀

                    # Filtering : 수직
                    if st_Line.f32_Angle is not None and (80 < abs(st_Line.f32_Angle) < 100):
                        continue

                    # Filtering : Hough Line 중 유효한 기울기만 걸러냄..파라미터 수정 혹은 굳이 필요한지 고민.....
                    if st_Line.f32_Angle is not None and not ((0 <= abs(st_Line.f32_Angle) <= 30) or (150 <= abs(st_Line.f32_Angle) <= 180) or
                            (35 <= abs(st_Line.f32_Angle) <= 55) or (125 <= abs(st_Line.f32_Angle) <= 145)):
                        continue

                    # Filtering : 옆 주차선 검출 방지
                    # if st_Line.f32_Slope < 0:
                    #     continue

                    matched = False

                    # print("Befor\n", GroupedLines,"\n")

                    for group in GroupedLines:
                        # print("group\n", group,"\n")
                        # print("group[0].slope\n", group[0].f32_Slope, " group[0].f32_InterceptY: ", group[0].f32_InterceptY, "\n")

                        # if (st_Line.f32_Slope * group[0][0] > 0) and abs(st_Line.f32_Slope - group[0][0]) < 0.01 and abs(st_Line.f32_InterceptY - group[0][1]) < 20:
                        if (st_Line.f32_Slope * group[0].f32_Slope >= 0) and abs(st_Line.f32_Slope - group[0].f32_Slope) < 0.1 and abs(st_Line.f32_InterceptY - group[0].f32_InterceptY) < 20:
                            group.append(st_Line)
                            matched = True
                            # print("After\n", GroupedLines,"\n")
                            break

                    if not matched:
                        GroupedLines.append([st_Line])
                        # print("After\n", GroupedLines,"\n")

                # print("----------------------------------------------\n\n")

                ### Draw grouped lines with their distinct colors
                # print("------------ Group Line ------------\n")

                # for i, arst_GroupLine in enumerate(GroupedLines):
                    # # print(i, "'s Group Lenght: ",len(arst_GroupLine), "\n")
                    # st_Color = self.random_color()

                    # for st_Line in arst_GroupLine:
                        # self.draw_line(st_GroupImage, st_Line, st_Color)
                # print("----------------------------------------------\n\n")

                #### 한 차선에 대한 Hough Line의 평균 계산
                AverageLines = []

                for group in GroupedLines:

                    # slopes
                    avg_slope = round((sum([item.f32_Slope for item in group]) / len(group)), 4)

                    # intercept
                    avg_intercept = round((sum([item.f32_InterceptY for item in group]) / len(group)), 4)

                    # Coords,,
                    avg_line_coords = [sum([item.arst_Coords[i] for item in group]) / len(group)
                                        for i in range(4)]

                    # Angle.. 수평 차선 검출 시 0으로 나누는 경우 Filtering
                    valid_items = [item.f32_Angle for item in group if item.f32_Angle is not None]

                    if len(valid_items) > 0:
                        avg_Angle = round(sum(valid_items) / len(valid_items), 4)

                    else:
                        avg_Angle = 0 

                    AverageLines.append(LaneInformation(avg_slope, avg_intercept, avg_line_coords, avg_Angle))

                #### Draw Average lines with their distinct colors
                # print("------------ Average Line ------------\n")

                # for i, st_Line in enumerate(AverageLines):
                #     print(i, "'s st_Line Slope: ",st_Line.f32_Slope, " Intercept Y: ",st_Line.f32_InterceptY, "\n")
                #     self.draw_line(st_FinalBGRImg, st_Line, self.random_color())

                # print("----------------------------------------------\n\n")

                #### Merge Same Line among Average Lines
                CheckFlag = [False] * len(AverageLines)
                MergeLines = []
                MergeIndex = 0

                for index, averageline in enumerate(AverageLines):

                    if CheckFlag[index] == True:
                        continue

                    CheckFlag[index] = True

                    MergeLines.append([averageline])  # averageline는 LaneInformation Object

                    for i in range(index + 1, len(AverageLines)):
                        
                        if CheckFlag[i] == True:
                            continue

                        # Average Lines를 Merge 하는 조건문
                        # averageline: 현재, AverageLines[i]: 비교대상

                        # ################### Check Average 조건 ###################
                        # print("averageline.f32_Slope: ", averageline.f32_Slope, ", AverageLines[i].f32_Slope: ",AverageLines[i].f32_Slope)
                        # print("averageline.f32_InterceptY: ", averageline.f32_InterceptY, ", AverageLines[i].f32_InterceptY: ",AverageLines[i].f32_InterceptY, "\n")
                        # #########################################################

                        if (averageline.f32_Slope * AverageLines[i].f32_Slope >= 0) and abs(averageline.f32_Slope - AverageLines[i].f32_Slope) < 0.3 and abs(averageline.f32_InterceptY - AverageLines[i].f32_InterceptY) < 30:
                            CheckFlag[i] = True
                            MergeLines[MergeIndex].append(AverageLines[i])

                    MergeIndex += 1


                #### Draw Merge lines with their distinct colors
                # print("------------  Merge Line ------------\n")

                # for i, arst_MergeLine in enumerate(MergeLines):
                #     print(i, "'s  MergeLine Lenght: ",len(arst_MergeLine), "\n")
                #     st_Color = self.random_color()

                #     for st_Line in arst_MergeLine:
                #         self.draw_line(st_HoughImage, st_Line, st_Color)
                # print("----------------------------------------------\n\n")


                #### Merge 한 Line들 끼리 평균화(Final)
                AverageLines = []

                for group in MergeLines:

                    # slopes
                    avg_slope = round((sum([item.f32_Slope for item in group]) / len(group)), 4)

                    # intercept
                    avg_intercept = round((sum([item.f32_InterceptY for item in group]) / len(group)), 4)

                    # Coords,,
                    avg_line_coords = [sum([item.arst_Coords[i] for item in group]) / len(group)
                                        for i in range(4)]

                    # Angle.. 수평 차선 검출 시 0으로 나누는 경우 Filtering
                    valid_items = [item.f32_Angle for item in group if item.f32_Angle is not None]

                    if len(valid_items) > 0:
                        avg_Angle = round(sum(valid_items) / len(valid_items), 4)

                    else:
                        avg_Angle = 0 

                    AverageLines.append(LaneInformation(avg_slope, avg_intercept, avg_line_coords, avg_Angle))

                #### Draw Average lines with their distinct colors
                # print("------------ Average Line ------------\n")

                # for i, st_Line in enumerate(AverageLines):
                #     print(i, "'s st_Line Slope: ",st_Line.f32_Slope, " Intercept Y: ",st_Line.f32_InterceptY, "\n")
                #     self.draw_line(st_FinalBGRImg, st_Line, self.random_color())

                # print("----------------------------------------------\n\n")
                               
                #### Sorting Lines
                SortInterceptLines = []
                SortInterceptLines = sorted(AverageLines, key=lambda x: x.f32_InterceptY, reverse=True)

                # ### Draw Sorted Average lines with their distinct colors
                # print("------------ Sorted Average Line ------------\n")

                # for i, st_Line in enumerate(SortInterceptLines):
                #     print(i, "'s st_Line Slope: ",st_Line.f32_Slope, " Intercept Y: ",st_Line.f32_InterceptY, "\n")
                #     self.draw_line(st_FinalBGRImg, st_Line, self.random_color())

                # print("----------------------------------------------\n\n")
                
                
                # if len(SortInterceptLines) != 0:
                #     self.draw_line(st_HoughImage, SortInterceptLines[0], self.random_color())


                #### Optimal Line Extraction
                if len(SortInterceptLines) != 0:                
                    # if self.b_IsFirstLine:
                    #     self.b_IsFirstLine = False
                    self.OptimalLine = SortInterceptLines[0]

                    # else:
                    #     if(abs(self.OptimalLine.f32_Slope - SortInterceptLines[0].f32_Slope) < 0.5):
                    #         self.OptimalLine = SortInterceptLines[0]

                    if self.OptimalLine != None:
                        
                        # print("before self.OptimalLine")
                        # print("self.OptimalLine.f32_Slope", self.OptimalLine.f32_Slope, " self.OptimalLine.f32_InterceptY: ", self.OptimalLine.f32_InterceptY)
                        # print()

                        self.draw_line(st_PubImgae, self.OptimalLine, self.random_color())

                else:
                    # print("len(SortInterceptLines): ", len(SortInterceptLines))
                    self.OptimalLine = None
                st_PublishMsg = LaneInfo()

                # print("after self.OptimalLine")
                # print("self.OptimalLine.f32_Slope", self.OptimalLine.f32_Slope, " self.OptimalLine.f32_InterceptY: ", self.OptimalLine.f32_InterceptY)
                print(self.OptimalLine)
                # print()

                if self.OptimalLine == None:
                    st_PublishMsg.f32_Slope = 0
                    st_PublishMsg.f32_InterceptY = 0
                else:
                    st_PublishMsg.f32_Slope = self.OptimalLine.f32_Slope
                    st_PublishMsg.f32_InterceptY = self.OptimalLine.f32_InterceptY

                self.st_Publisher.publish(st_PublishMsg)

                # print()
                # print("st_PublishMsg")
                # print(st_PublishMsg)
                # print()
                
            else:
                print("There is no Hough Lines")

            cv2.imshow("st_PubImgae", st_PubImgae)
            # cv2.imshow("st_HoughImage", st_HoughImage)
            # cv2.imshow("Edge", Edges)
            # cv2.imshow("st_GroupImage", st_GroupImage)
            cv2.waitKey(1)

    def random_color(self):
        return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

    def SlopeIsDifferent(self, previous_slope, current_slope, threshold = 1.0):
        # 두 기울기 사이의 차이가 threshold 이상일 때 해당 라인을 건너뜀
        return abs(previous_slope - current_slope) > threshold
    
    def draw_line(self, img, line, color):
        
        avg_slope, avg_intercept = line.f32_Slope, line.f32_InterceptY
        # print("avg_slope: ", avg_slope, "avg_intercept: ", avg_intercept)
        x_min, x_max = 0, self.width

        y_min = avg_slope * x_min + avg_intercept
        y_max = avg_slope * x_max + avg_intercept

        if not math.isnan(y_min) and not math.isnan(y_max):
            if y_min < 0:
                y_min = 0
                x_min = (y_min - avg_intercept) / avg_slope
            elif y_min > self.height:
                y_min = self.height
                x_min = (y_min - avg_intercept) / avg_slope

            if y_max < 0:
                y_max = 0
                x_max = (y_max - avg_intercept) / avg_slope
            elif y_max > self.height:
                y_max = self.height
                x_max = (y_max - avg_intercept) / avg_slope

            x_min = max(0, min(self.width, x_min))
            x_max = max(0, min(self.width, x_max))

            cv2.line(img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
            
            cv2.circle(img, (int(x_min), int(y_min)), 5, (0, 0, 255), -1)  # 빨간색 원을 그리기
            
            # y절편 값을 텍스트로 표시 (소수점 제거 후 정수로 표시)
            y_intercept_text = f"y: {int(y_min)}"
            cv2.putText(img, y_intercept_text, (int(x_min) + 10, int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    def CalculateLinesDetails(self, Lines):

        x1, y1, x2, y2 = Lines

        if (x2 - x1) != 0:
            slope = round((y2 - y1) / (x2 - x1), 4)
            HoughAngle = round(np.degrees(np.arctan(slope)), 4)
            intercept = round((y1 - slope * x1), 4)
        else:
            return None

        return LaneInformation(slope, intercept, Lines, HoughAngle)


if __name__ == '__main__':
    rospy.init_node("BackLane", anonymous=True)
    BackLaneDetection()
    rospy.spin()