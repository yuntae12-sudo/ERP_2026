#!/usr/bin/env python3
import os
import sys
import rospy
import cv2
import torch
import numpy as np
import math
from pathlib import Path
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Float64 
import torch, gc
#from Total_msgs.msg import ModeCmd, DriveCmd
from seojin_pkg.msg import DriveCmd

# yolopv2_code 경로 추가
base_path = Path(__file__).resolve().parents[0]
print(base_path)
yolo_path = base_path/"yolopv2_code"
weights_path = yolo_path / "yolopv2.pt"
sys.path.append(os.path.join(os.path.dirname(__file__), yolo_path))
from utils.utils import (
    split_for_trace_model,
    driving_area_mask,
    lane_line_mask,
    non_max_suppression,
    show_seg_result,
    letterbox
)

class YOLOPv2ROSNode:
    def __init__(self):

        self.st_BGRImg = None
        self.width = 1280
        self.height = 720
        # ===== Subscriber =====
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image)
        self.st_Publisher = rospy.Publisher('/steering_angle', Float64, queue_size=1)
        self.st_drive_pub = rospy.Publisher('/drive_cmd_serial', DriveCmd, queue_size=10)

        # ===== model =====
        self.model_path = rospy.get_param("~model_path", weights_path)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"모델 로딩 중: {self.model_path}")
        self.model = torch.jit.load(self.model_path, map_location=self.device)
        self.model.eval()

        # ===== parameters =====
        self.last_process_time = rospy.Time.now()
        self.min_interval = rospy.Duration(0.05)  # 0.2초 (5 FPS)
        self.bridge = CvBridge()
        self.img_size = 640
        self.conf_thres = 0.8
        self.iou_thres = 0.8
        # 전 프레임 중심선 저장 변수 선언
        self.prev_smooth_center = []
        # 소실점 저장용 전역 변수 (초기값은 None)
        self.prev_vanish_point = None
        self.alpha = 0.2  # 보간 계수

        # ===== process =====
        self.rate = rospy.Rate(20) 
        rospy.on_shutdown(self.cleanup)
        self.YolopDetection()

    def cleanup(self):
        print("[INFO] Shutting down, closing OpenCV windows.")
        torch.cuda.empty_cache()
        gc.collect()
        cv2.destroyAllWindows()
    
    def callback_Image(self, msg):
        """
        이미지를 수신하고, 하단에 맞춰 자른 후 1280x720으로 리사이즈합니다.
        """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img_bgr is not None:
                h, w, _ = img_bgr.shape
                target_h, target_w = self.height, self.width

                # 이미지 크기가 목표(1280x720)와 다를 경우에만 변환 수행
                if w != target_w or h != target_h:
                    # 1. 목표 비율(16:9)에 맞게 원본 이미지(1020x720)를 잘라낼 높이 계산
                    #    (1020 / (1280 / 720)) = 573.75 -> 574
                    target_aspect = target_w / float(target_h)
                    new_h = int(w / target_aspect)

                    # 2. 이미지 상단 잘라내기 (하단 정렬)
                    #    원본 높이(720)에서 새 높이(574)를 빼서 위에서 잘라낼 양(146)을 구함
                    crop_top = h - new_h
                    img_cropped = img_bgr[crop_top:h, :] # y축은 crop_top부터 끝까지, x축은 전체

                    # 3. 잘라낸 이미지를 최종 목표 해상도로 리사이즈
                    #    (1020x574 이미지를 1280x720으로 확대)
                    img_resized = cv2.resize(img_cropped, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
                    
                    self.st_BGRImg = img_resized
                else:
                    # 크기가 이미 맞으면 그대로 사용
                    self.st_BGRImg = img_bgr
                    
        except CvBridgeError as e:
            print(e)

    def preprocess(self, img):
        resized_img, ratio, pad = letterbox(img, new_shape=self.img_size, stride=32)
        img_rgb = cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB)
        img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
        return img_tensor.unsqueeze(0).to(self.device), ratio, pad
    
    def restore_mask(self, mask, ratio, pad, original_shape):

        h_ori, w_ori = original_shape[:2]

        # → 이미 원본 해상도라면 그대로 리턴
        if mask.shape[:2] == (h_ori, w_ori):
            return mask

        # 그렇지 않으면 letterbox 역연산
        pad_w, pad_h = int(pad[0]), int(pad[1])
        h_mask, w_mask = mask.shape[:2]

        # 1) padding 제거
        mask_cropped = mask[pad_h : h_mask - pad_h,
                            pad_w : w_mask - pad_w]

        # 2) 원본 크기로 리사이즈
        mask_resized = cv2.resize(
            mask_cropped,
            (w_ori, h_ori),                    # (width, height)
            interpolation=cv2.INTER_NEAREST
        )
        print(mask_resized.shape)

        return mask_resized

    def fit_poly(self, points, degree=2, num_points=100):
        if len(points) < degree + 1:
            return points
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        poly = np.polyfit(y, x, degree)
        y_new = np.linspace(y.min(), y.max(), num_points)
        x_new = np.polyval(poly, y_new)
        return list(zip(x_new.astype(int), y_new.astype(int)))
    
    def draw_dotted_line_with_circles(self, img, pt1, pt2, color, radius=3, gap=15):
        """pt1에서 pt2까지 원으로 점선을 그림"""
        pt1 = np.array(pt1)
        pt2 = np.array(pt2)
        direction = pt2 - pt1
        length = np.linalg.norm(direction)
        direction = direction / length  # 단위 벡터

        num_dots = int(length // gap)
        for i in range(num_dots + 1):
            dot_center = pt1 + direction * i * gap
            center = tuple(dot_center.astype(int))
            cv2.circle(img, center, radius, color, -1)

    def clamp(self, value, min_val, max_val):
        """Clamps a value between a minimum and maximum."""
        return max(min_val, min(value, max_val))

    def is_abnormal_center(self, current, previous, threshold=80):
        if not current or not previous or len(current) != len(previous):
            return True
        diffs = [abs(a[0] - b[0]) for a, b in zip(current, previous)]
        return np.mean(diffs) > threshold

    def YolopDetection(self):

        while self.st_BGRImg is None:
            rospy.sleep(0.05)
            pass

        try:
            while not rospy.is_shutdown():
                image = self.st_BGRImg.copy()
                #cv2.imshow("YOLO Detection", image)

                input_tensor, ratio, pad = self.preprocess(image)

                with torch.no_grad():
                    [pred, anchor_grid], seg, ll = self.model(input_tensor)

                pred = split_for_trace_model(pred, anchor_grid)
                pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)

                da_mask = driving_area_mask(seg)
                #ll_mask = lane_line_mask(ll)

                # 1. 모델로부터 나온 마스크 (640x640)
                # 모델 예측 결과
                drivable_mask = da_mask.astype(np.uint8) * 255
                #lane_mask     = ll_mask.astype(np.uint8) * 255

                # letterbox 역변환으로 복원
                drivable_resized = self.restore_mask(drivable_mask, ratio, pad, image.shape)
                #lane_resized     = self.restore_mask(lane_mask, ratio, pad, image.shape)

                # 오버레이 생성
                overlay = np.zeros_like(image)
                #overlay[lane_resized > 127] = [255, 255, 255]   
                overlay[drivable_resized > 127] = [0, 255, 0]  

                # 합성
                result = cv2.addWeighted(image, 0.7, overlay, 0.3, 0)

                # Bounding box 시각화
                # for det in pred:
                #     if det is not None and len(det):
                #         for *xyxy, conf, cls in det:
                #             xyxy = torch.tensor(xyxy).view(1, 4)
                #             xyxy = xyxy.cpu().numpy().astype(int).flatten()
                #             cv2.rectangle(result, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 255), 2)

                # ==============================================================================

                # 2. 마스크 정제 (Blur + Contour + Convex Hull)
                mask_blur = cv2.GaussianBlur(overlay, (5, 5), 0)
                gray_mask = cv2.cvtColor(mask_blur, cv2.COLOR_BGR2GRAY)

                # 1. 결과를 하나의 변수에 모두 담아서 unpacking 에러를 원천 차단합니다.
                contours, _ = cv2.findContours(gray_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                mask_clean = np.zeros_like(mask_blur)
                for cnt in contours:
                    if cv2.contourArea(cnt) > 5000:
                        hull = cv2.convexHull(cnt)
                        cv2.drawContours(mask_clean, [hull], -1, 255, -1)
                mask = mask_clean  # 이후 흐름에서 사용


                # 3. 중심선 계산: 수직 슬라이스 안에서 좌/우 경계 추출
                slice_width = 300
                x_center = self.width // 2
                x_start = x_center - slice_width // 2
                x_end = x_center + slice_width // 2

                center_points = []
                for y in range(self.height - 1, 0, -10):
                    row = mask[y, x_start:x_end]
                    white = np.where(row > 0)[0]
                    if len(white) < 100:
                        continue
                    left_x = white[0] + x_start
                    right_x = white[-1] + x_start
                    center_x = (left_x + right_x) // 2
                    center_points.append((center_x, y))

                # 4. 곡선 보간 + 튐 방지 처리
                smooth_center = self.fit_poly(center_points, degree=2, num_points=100)

                # 중심선 튐 방지 및 초기 설정
                if not self.prev_smooth_center:
                    self.prev_smooth_center = smooth_center
                else:
                    if self.is_abnormal_center(smooth_center, self.prev_smooth_center):
                        smooth_center = self.prev_smooth_center
                    else:
                        self.prev_smooth_center = smooth_center


                # 임시 차선용 L/R → 소실점 곡선 표시
                row_range = 20  # 하단 몇 줄을 볼지
                bottom_y_range = range(self.height - 1, self.height - row_range, -1)

                left_xs, right_xs, ys = [], [], []

                x_shift = 250  # 시작점 조절 세기 (픽셀 단위)

                for y in bottom_y_range:
                    row = mask[y, :]
                    white = np.where(row > 0)[0]
                    if len(white) > 0:
                        # 좌우 경계 조절: 좌는 살짝 오른쪽으로, 우는 살짝 왼쪽으로
                        left_xs.append(x_shift)
                        right_xs.append(1280 - x_shift)
                        ys.append(y)

                # 정제된 마스크에서 소실점 계산
                ys_mask, xs_mask = np.where(mask[:, :, 0] > 0)

                if len(ys_mask) > 0:
                    min_y = np.min(ys_mask)  # 가장 위쪽 y
                    x_on_min_y = np.where(mask[min_y, :] > 0)[0]
                    if len(x_on_min_y) > 0:
                        left_x = x_on_min_y[0]
                        right_x = x_on_min_y[-1]
                        vanish_x = (left_x + right_x) // 2
                        raw_vanish_point = (vanish_x, min_y)
                    else:
                        raw_vanish_point = (self.width // 2, self.height // 3)  # fallback
                else:
                    raw_vanish_point = (self.width // 2, self.height // 3)  # fallback
                    
                if self.prev_vanish_point is None:
                    vanish_point = raw_vanish_point
                else:
                    vanish_point = (
                        int((1 - self.alpha) * self.prev_vanish_point[0] + self.alpha * raw_vanish_point[0]),
                        int((1 - self.alpha) * self.prev_vanish_point[1] + self.alpha * raw_vanish_point[1])
                    )

                # 갱신
                self.prev_vanish_point = vanish_point    


                # 1. 차량의 현재 위치(이미지 하단 중앙) 설정
                car_center_x = self.width / 2
                
                # 2. 소실점과 차량 중심 사이의 수평 오차(pixel) 계산
                #    오차가 양수(+)이면 우회전 필요, 음수(-)이면 좌회전 필요
                x_offset = vanish_point[0] - car_center_x
                
                # 3. 수직 거리를 이용해 각도 계산 (atan2 사용)
                #    차량 위치(이미지 맨 아래)에서 소실점까지의 수직 거리
                y_offset = self.height - vanish_point[1]
                
                # atan2(수평 오차, 수직 거리) -> 라디안(radian) 값 반환
                angle_rad = math.atan2(x_offset, y_offset)
                
                # 라디안을 도(degree)로 변환
                angle_deg = math.degrees(angle_rad)
                
                # 4. 계산된 조향각을 Float64 메시지로 Publish
                angle_deg
                f32_TargetSteer = angle_deg * 74.074
                f32_TargetSteer = self.clamp(f32_TargetSteer, -2000.0, 2000.0)
                Drive_msg = DriveCmd()
                Drive_msg.Deg = int(f32_TargetSteer)
                Drive_msg.KPH = int(10)
                self.st_drive_pub.publish(Drive_msg)

                # (디버깅용) 콘솔에 조향각 출력
                # rospy.loginfo(f"Steering Angle: {angle_deg:.2f} degrees")
                # ===========================================================

                if len(left_xs) > 0 and len(right_xs) > 0 and len(ys) > 0:
                    left_bottom = (left_xs[0], ys[0])
                    right_bottom = (right_xs[0], ys[0])

                cv2.line(result, (600,720), vanish_point, (0, 255, 255), 1)
                cv2.line(result, (1280-600,720), vanish_point, (0, 255, 255), 1)
                
                path_area = np.array([[(600,720), (680,720), vanish_point]], dtype=np.int32)
                overlay = result.copy()
                cv2.fillPoly(overlay, [path_area], (0, 255, 255))
                result = cv2.addWeighted(overlay, 0.3, result, 0.7, 0)


                #cv2.imshow("YOLOPv2", overlay)
                cv2.imshow("YOLOPv2 Result", result)
                cv2.waitKey(1)
                self.rate.sleep()

        except Exception as e:
            rospy.logerr(f"[YOLOPv2 Node] 오류 발생: {e}")

if __name__ == "__main__":
    rospy.init_node('YOLOPv2', anonymous=True)
    YOLOPv2ROSNode()
    rospy.spin()