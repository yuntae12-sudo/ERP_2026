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
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
import torch, gc
from camera.msg import DriveCmd,ModeCmd

# =========================
# YOLOPv2 + ROS Node
# =========================

# yolopv2_code 경로 추가 (기존 그대로)
base_path = Path(__file__).resolve().parents[0]
yolo_path = base_path / "yolopv2_code"
weights_path = yolo_path / "yolopv2.pt"
sys.path.append(str(yolo_path))

from utils.utils import (
    split_for_trace_model,
    driving_area_mask,
    non_max_suppression,
    letterbox
)

class YOLOPv2ROSNode:
    def __init__(self):

        self.width = 1280
        self.height = 720
        # ====== ROS Topics ======
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image)
        self.st_drive_pub = rospy.Publisher('/erp42_serial/drive', DriveCmd, queue_size=10)
        self.st_mode_pub = rospy.Publisher('/erp42_serial/mode', ModeCmd, queue_size=10)
        # ====== Model Load ======
        self.model_path = rospy.get_param("~model_path", str(weights_path))
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"모델 로딩 중: {self.model_path}")
        self.model = torch.jit.load(self.model_path, map_location=self.device)
        self.model.eval()

        # ====== Params ======
        self.bridge = CvBridge()
        self.img_size = 640
        self.conf_thres = 0.8
        self.iou_thres = 0.8
        self.alpha = 0.1  # 필터 반응 속도 (0.1로 설정하면 천천히 반응)
        self.prev_angle = None  # 이전 조향각
        # 원본 이미지 크기 (카메라 해상도)
        self.width = 1280
        self.height = 720

        # BEV 변환 후 크기 ★ (고정값)
        self.bev_w = 400
        self.bev_h = 800

        # LD / 히스토그램 관련 파라미터 ★
        self.base_ld_row = 650  # 기본적으로 "가까운 곳"을 본다고 한 y≈650 아이디어. (원본기준인데 BEV에서는 아래쪽 row로 사용할거야)
        # BEV 좌표계에서 row=bev_h-1 가 차량 바로 앞 바닥이라고 생각할 거라서,
        # base_ld_row는 BEV 하단 근처로 설정하는 게 맞아. 아래에서 다시 매핑할 거라 유연하게 둘게.

        self.hist_threshold = 400   # "히스토그램 값이 400 이상이면 길 넓음" 이 기준값
        self.ld_step = 50           # LD를 멀리/가깝게 조정할 때 한 번에 움직일 픽셀 수
        self.min_ld_row = 300       # 너무 가까운 곳만 보지 않도록 하단 제한 (BEV에서 하단은 큰 row)
        self.max_ld_row = 700       # 너무 멀리만 보지 않도록 상단 제한 (BEV에서 상단은 작은 row)

        # 투시변환用 source/dest point ★
        # - src_pts: 원본 이미지에서 뽑을 4점 (주행가능영역 바닥부 사다리꼴)
        #   이건 너가 튜닝할 수 있게 rosparam으로도 받을 수 있게 하고, 기본값은 대충 박아둘게
        # - dst_pts: BEV 평면에서의 직사각형 맵핑 영역
        self.src_pts = np.float32(rospy.get_param("~src_points", [
            [400, 400],   # 좌상
            [880, 400],   # 우상
            [1180, 700],  # 우하
            [100, 700]    # 좌하
        ]))
        self.dst_pts = np.float32(rospy.get_param("~dst_points", [
            [0, 0],
            [self.bev_w-1, 0],
            [self.bev_w-1, self.bev_h-1],
            [0, self.bev_h-1]
        ]))

        # 미리 homography 계산 ★
        self.H = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)

        # state
        self.st_BGRImg = None
        self.last_img_stamp = None   
        self.prev_ld_row = self.max_ld_row  # 초기 LD는 가까운 쪽(아래쪽 row)로 둠

        # rate & shutdown
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.cleanup)

        # main loop
        self.main_loop()

    def cleanup(self):
        print("[INFO] Shutting down, closing OpenCV windows.")
        torch.cuda.empty_cache()
        gc.collect()
        cv2.destroyAllWindows()

    def callback_Image(self, msg):
        """
        어떤 입력 해상도든 종횡비를 보존해 크롭한 뒤, 1280x720(self.width x self.height)으로 리사이즈.
        - 와이드 입력: 좌우(center) 크롭
        - 세로로 긴 입력: 하단 정렬(bottom) 크롭
        """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_bgr is None:
                return

            h, w = img_bgr.shape[:2]
            target_w, target_h = self.width, self.height  # 예: 1280, 720
            tgt_aspect = target_w / float(target_h)
            src_aspect = w / float(h)

            if w == target_w and h == target_h:
                self.st_BGRImg = img_bgr
                return

            if src_aspect > tgt_aspect:
                # 입력이 더 와이드 → 좌우(center) 크롭
                new_w = int(round(h * tgt_aspect))
                new_w = min(new_w, w)  # 안정장치
                x0 = max(0, (w - new_w) // 2)
                x1 = x0 + new_w
                img_cropped = img_bgr[:, x0:x1]
            else:
                # 입력이 더 세로로 김 → 하단 정렬 크롭
                new_h = int(round(w / tgt_aspect))
                new_h = min(new_h, h)  # 안정장치
                y0 = max(0, h - new_h)  # 하단 정렬
                y1 = y0 + new_h
                img_cropped = img_bgr[y0:y1, :]

            # 다운스케일은 INTER_AREA, 업스케일은 INTER_LINEAR
            ih, iw = img_cropped.shape[:2]
            interp = cv2.INTER_AREA if (iw > target_w or ih > target_h) else cv2.INTER_LINEAR
            img_resized = cv2.resize(img_cropped, (target_w, target_h), interpolation=interp)

            self.st_BGRImg = img_resized

        except Exception as e:
            print("callback_Image error:", e)

    def preprocess(self, img):
        resized_img, ratio, pad = letterbox(img, new_shape=self.img_size, stride=32)
        img_rgb = cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB)
        img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
        return img_tensor.unsqueeze(0).to(self.device), ratio, pad

    def restore_mask(self, mask, ratio, pad, original_shape):
        """ letterbox로 줄어든 640x640 마스크를 원본(1280x720) 크기로 복원 """
        h_ori, w_ori = original_shape[:2]

        if mask.shape[:2] == (h_ori, w_ori):
            return mask

        pad_w, pad_h = int(pad[0]), int(pad[1])
        h_mask, w_mask = mask.shape[:2]

        # padding 제거
        mask_cropped = mask[pad_h:h_mask-pad_h, pad_w:w_mask-pad_w]

        # 원본 크기로 리사이즈
        mask_resized = cv2.resize(
            mask_cropped, (w_ori, h_ori),
            interpolation=cv2.INTER_NEAREST
        )
        return mask_resized
    
    def low_pass_filter(self, angle_deg):
        # 저역통과 필터를 사용하여 조향각을 부드럽게 만듦
        if self.prev_angle is None:
            self.prev_angle = angle_deg
        # 필터 적용 (현재값과 이전값을 고려하여 부드럽게 변경)
        smoothed_angle = self.alpha * angle_deg + (1 - self.alpha) * self.prev_angle
        self.prev_angle = smoothed_angle
        return smoothed_angle

    def infer_drivable_mask(self, image_bgr):

        input_tensor, ratio, pad = self.preprocess(image_bgr)

        with torch.no_grad():
            [pred, anchor_grid], seg, ll = self.model(input_tensor)

        # bbox는 지금 안씀
        pred = split_for_trace_model(pred, anchor_grid)
        _ = non_max_suppression(pred, self.conf_thres, self.iou_thres)

        da_mask = driving_area_mask(seg)  # [0/1] mask in 640x640
        drivable_mask = (da_mask.astype(np.uint8) * 255)

        drivable_resized = self.restore_mask(
            drivable_mask, ratio, pad, image_bgr.shape
        )  # -> (720,1280)
        return drivable_resized  # single-channel uint8

    # =====================
    # 2) BEV 변환
    # =====================
    def warp_to_bev(self, mask_1280x720):

        bev = cv2.warpPerspective(
            mask_1280x720, self.H,
            (self.bev_w, self.bev_h),
            flags=cv2.INTER_NEAREST
        )
        # 이 bev는 single-channel (0~255)
        return bev

    # =====================
    # 3) 히스토그램 기반 조향
    # =====================
    def choose_look_distance_row(self, bev_mask):
        
        # '조향각 0' 조건 (기존 코드의 안전 로직 유지)
        # 맨 하단의 score를 먼저 확인
        bottom_row = int(self.max_ld_row)
        bottom_line_vals = bev_mask[bottom_row, :]
        bottom_score = np.count_nonzero(bottom_line_vals > 127)

        if bottom_score < self.hist_threshold // 2:
            # 아예 길이 안 보임. '조향각 0'을 위해 0 반환 (사용자 의도 추정)
            return 0, bottom_score
            
        # 안전한 기본값: 맨 하단 row
        # 만약 루프가 한 번도 '후보 LD'를 갱신 못하면 (즉, 맨 하단도 400이 안되면)
        # 이 값이 반환됨.
        chosen_ld_row = bottom_row
        chosen_score = bottom_score

        # 스캔 시작: 하단(max_ld_row) -> 상단(min_ld_row)
        # range(start, stop, step) -> stop은 포함되지 않으므로 min_ld_row - 1
        for row in range(int(self.max_ld_row), int(self.min_ld_row) - 1, -1):
            line_vals = bev_mask[row, :]
            score = np.count_nonzero(line_vals > 127)
            
            if score >= self.hist_threshold:
                # 임계값(400) 통과. 이 row를 '후보'로 저장하고 계속 위로 진행
                chosen_ld_row = row
                chosen_score = score
            else:
                # 임계값 미만. "여기서부터 길이 좁아짐"
                # 스캔을 중단하고, *직전에* 저장한 '후보' (chosen_ld_row)를 반환.
                break
                
        # 이 로직은 self.prev_ld_row를 사용하지 않음.
        return int(chosen_ld_row), int(chosen_score)
    
    def clamp(self, value, min_val, max_val):
        """Clamps a value between a minimum and maximum."""
        return max(min_val, min(value, max_val))

    def compute_center_x(self, bev_mask, row_y):

        line_vals = bev_mask[row_y, :]
        xs = np.where(line_vals > 127)[0]  # 주행가능(255)인 x 인덱스들

        if len(xs) < 10:
            # 거의 정보 없으면 그냥 화면 중앙 반환
            return self.bev_w // 2

        left_x = xs[0]
        right_x = xs[-1]
        center_x = (left_x + right_x) // 2
        return int(center_x)

    def steering_from_bev(self, bev_mask, ld_row):

        car_x = self.bev_w / 2.0      # 차량 중심 가정
        car_y = self.bev_h + 100        # 차량은 BEV 하단

        target_x = self.compute_center_x(bev_mask, ld_row)
        target_y = ld_row

        x_offset = (target_x - car_x)
        y_offset = (car_y - target_y)

        angle_rad = math.atan2(x_offset, y_offset)  # 좌우 편차 / 전방 거리
        angle_deg = math.degrees(angle_rad)

        return angle_deg, (int(target_x), int(target_y)), (int(car_x), int(car_y))

    # =====================
    # 4) 메인 루프
    # =====================
    def main_loop(self):
        # 첫 프레임 들어올 때까지 대기
        while self.st_BGRImg is None and not rospy.is_shutdown():
            rospy.sleep(0.01)

        while not rospy.is_shutdown():
            frame = self.st_BGRImg.copy()

            # 1) YOLOPv2로 drivable mask 얻기 (1280x720)
            drivable_mask = self.infer_drivable_mask(frame)  # uint8 [0 or 255]

            # 2) BEV로 warp
            bev_mask = self.warp_to_bev(drivable_mask)       # uint8 (800x400)

            # 3) LD 결정 (동적 Look Distance)
            ld_row, ld_score = self.choose_look_distance_row(bev_mask)

            # 4) 조향각 계산
            angle_deg, target_pt, car_pt = self.steering_from_bev(bev_mask, ld_row)

            angle_deg = self.low_pass_filter(angle_deg)
            angle_deg
            f32_TargetSteer = angle_deg * 32.074
            f32_TargetSteer = self.clamp(f32_TargetSteer, -2000.0, 2000.0)
            # 5) Publish
            Drive_msg = DriveCmd()
            Drive_msg.Deg = int(f32_TargetSteer)
            Drive_msg.KPH = int(100)
            Drive_msg.brake = int(10)
            self.st_drive_pub.publish(Drive_msg)

            Mode_msg = ModeCmd()
            Mode_msg.Gear = int(2)
            self.st_mode_pub.publish(Mode_msg)

            # ================= 디버깅/시각화 =================
            # 원본 오버레이 (초록색 주행가능 영역)
            overlay = np.zeros_like(frame)
            overlay[drivable_mask > 127] = [0, 255, 0]
            vis_orig = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)

            # BEV 시각화용 컬러 이미지
            bev_color = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)

            # LD row 라인
            cv2.line(bev_color, (0, ld_row), (self.bev_w-1, ld_row), (0, 0, 255), 2)

            # target point & car point
            cv2.circle(bev_color, target_pt, 5, (255, 0, 0), -1)  # 파란 점 (타겟)
            cv2.circle(bev_color, car_pt, 5, (0, 255, 255), -1)  # 차량 중심 (노란 점)
            cv2.putText(
                bev_color,
                f"angle:{angle_deg:.2f} deg score:{ld_score}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255,255,255),
                2
            )

            # 기준점과 타겟 점을 이어주는 선 (조향각 시각화)
            cv2.line(bev_color, car_pt, target_pt, (0, 255, 0), 2)  # 파란 점과 기준점 연결

            # homography 디버깅용: src_pts와 dst_pts를 원본/BEV에 그려서 튜닝할 때 도움
            debug_frame = frame.copy()
            for i, p in enumerate(self.src_pts):
                cv2.circle(debug_frame, tuple(map(int,p)), 8, (0,0,255), -1)
                cv2.putText(debug_frame, f"S{i}", tuple(map(int,p)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)

            for i, p in enumerate(self.dst_pts):
                cv2.circle(bev_color, tuple(map(int,p)), 5, (0,255,0), -1)
                cv2.putText(bev_color, f"D{i}", tuple(map(int,p)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)

            cv2.imshow("Original + Drivable Overlay", vis_orig)
            cv2.imshow("BEV Mask / Control View", bev_color)
            cv2.imshow("Homography Src Debug", debug_frame)

            cv2.waitKey(1)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('YOLOPv2', anonymous=True)
    YOLOPv2ROSNode()
    rospy.spin()
