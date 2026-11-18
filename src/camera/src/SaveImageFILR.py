import cv2
import numpy as np
import rospy
from camera.msg import N_image
from cv_bridge import CvBridgeError
import os
from sensor_msgs.msg import CompressedImage

class ImgSave:
    def __init__(self):
        self.image_sub=rospy.Subscriber("raw_image", N_image, self.ImageCallback_FILR)

        self.directory = '/home/autonav/Pictures'        
        self.ImgCout=self.CountExistingImages()
        print("now Images Number")
        print(self.ImgCout)

        self.st_BGRImg = None
        self.timer = rospy.Timer(rospy.Duration(1), self.SaveImgeCallBack)  # 3초마다 ImageCallback_MORAI 호출
    def CountExistingImages(self):
        # 해당 경로에 있는 jpg의 갯수 count
        return len([name for name in os.listdir(self.directory) if name.endswith('.jpg')])
    
    # For ERP
    def ImageCallback_FILR(self,Image):
        self.st_BGRImg = np.ndarray(shape=(Image.height, Image.width, Image.n_channels),
                              dtype=eval(Image.dtype), buffer=Image.data)
                
        cv2.imshow("Imagewindow", self.st_BGRImg)
        cv2.moveWindow("Image window", 0,0)
        cv2.waitKey(1)

    def SaveImgeCallBack(self,event):
        if self.st_BGRImg is not None:
            FileName=os.path.join(self.directory, 'image{}.jpg'.format(self.ImgCout))
            cv2.imwrite(FileName,self.st_BGRImg)
            self.ImgCout = self.ImgCout + 1
            print("Image Save")

if __name__ =='__main__':
    rospy.init_node('image_parer', anonymous=True)

    image_parser=ImgSave()

    rospy.spin()