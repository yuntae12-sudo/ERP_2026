import cv2
import os

def crop_image_from_left(image):
    # 이미지의 원본 크기
    height, width = image.shape[:2]

    # 자를 이미지의 목표 크기
    target_width = 640
    target_height = 480

    # 시작 X 좌표는 앞쪽 부분을 자르기 위해 320부터
    # start_x = width - 100 - target_width  # 960 - 640 = 320
    # end_x = width - 100  # 960

    start_x = 150
    end_x = target_width + start_x

    # 자를 Y 범위는 480으로 제한
    start_y = 0
    end_y = target_height + start_y  # 480

    # 이미지 자르기 (왼쪽을 자르고 뒷부분만 남김)
    cropped_image = image[start_y:end_y, start_x:end_x]

    return cropped_image

def process_images_in_folder(folder_path):
    # 폴더 내 모든 파일을 순회
    for filename in os.listdir(folder_path):
        # 이미지 파일의 전체 경로
        file_path = os.path.join(folder_path, filename)

        # 이미지 로드
        image = cv2.imread(file_path)

        # 이미지가 제대로 로드되었는지 확인
        if image is None:
            print(f"Failed to load image: {file_path}")
            continue

        # 이미지 크기 확인
        height, width = image.shape[:2]

        # 이미지가 640x480이 아닌 경우에만 처리
        if width != 640 or height != 480:
            print(f"Processing image: {file_path} (Size: {width}x{height})")

            # 이미지 자르기
            cropped_image = crop_image_from_left(image)

            # 자른 이미지를 저장할 파일 경로
            save_path = os.path.join(folder_path, f"cropped_{filename}")

            # 자른 이미지 저장
            cv2.imwrite(file_path, cropped_image)
            print(f"Image saved to {save_path}")
        else:
            print(f"Skipping image: {file_path} (Size: {width}x{height})")

# 이미지들이 저장된 폴더 경로
folder_path = '/home/kyd/Dataset_TrafficLight_240903/TrafficLight_4/Yellow/'

# 폴더 내 모든 파일 처리
process_images_in_folder(folder_path)
