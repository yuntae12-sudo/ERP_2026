import sys
import cv2
import time

class WebcamParser:
    def __init__(self, camera_id="/dev/video5", save_interval=2):
        self.camera_id = camera_id
        self.save_interval = save_interval
        self.video_capture = cv2.VideoCapture(self.camera_id)
        self.image_count = 1  # 이미지 파일 이름을 위한 카운터

        # 해상도 설정
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.video_capture.isOpened():
            print("Failed to open camera")
            sys.exit(1)

    def run(self):
        last_time = time.time()

        while True:
            ret, frame = self.video_capture.read()

            if ret:
                current_time = time.time()

                # 저장 간격을 확인하고 이미지 저장
                if current_time - last_time >= self.save_interval:
                    file_name = f"/home/autonav/catkin_ws/src/Dataset/webcam_image_{self.image_count}.jpg"
                    cv2.imwrite(file_name, frame)
                    print(f"Saved {file_name}")
                    self.image_count += 1
                    last_time = current_time

                cv2.imshow("Webcam", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Failed to capture image")

        self.video_capture.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    webcam_parser = WebcamParser()
    webcam_parser.run()