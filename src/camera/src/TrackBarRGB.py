import cv2
import numpy as np

def on_trackbar_change(val):
    pass  # 이 함수는 트랙바 값이 바뀔 때마다 호출됩니다. 여기서는 아무 작업도 수행하지 않습니다.

def main():
    # 이미지 로드
    image = cv2.imread('/home/autonav/Pictures/image4.jpg')  # 이미지 경로를 올바르게 수정하세요.

    # 윈도우 생성
    cv2.namedWindow("Image")

    # BGR 트랙바 생성
    cv2.createTrackbar("B_low", "Image", 0, 255, on_trackbar_change)
    cv2.createTrackbar("B_high", "Image", 255, 255, on_trackbar_change)
    cv2.createTrackbar("G_low", "Image", 0, 255, on_trackbar_change)
    cv2.createTrackbar("G_high", "Image", 255, 255, on_trackbar_change)
    cv2.createTrackbar("R_low", "Image", 0, 255, on_trackbar_change)
    cv2.createTrackbar("R_high", "Image", 255, 255, on_trackbar_change)

    while True:
        # 트랙바 값을 읽어서 BGR 범위 설정
        b_low = cv2.getTrackbarPos("B_low", "Image")
        b_high = cv2.getTrackbarPos("B_high", "Image")
        g_low = cv2.getTrackbarPos("G_low", "Image")
        g_high = cv2.getTrackbarPos("G_high", "Image")
        r_low = cv2.getTrackbarPos("R_low", "Image")
        r_high = cv2.getTrackbarPos("R_high", "Image")

        # BGR 범위로 마스크 생성
        lower_bound = np.array([b_low, g_low, r_low])
        upper_bound = np.array([b_high, g_high, r_high])
        mask = cv2.inRange(image, lower_bound, upper_bound)

        # 마스크를 이용하여 원본 이미지에 색상 표시
        result = cv2.bitwise_and(image, image, mask=mask)

        # 결과 이미지 출력
        cv2.imshow("Image", result)

        # 키 입력 대기
        key = cv2.waitKey(1)
        if key == 27:  # ESC 키를 누르면 종료
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()