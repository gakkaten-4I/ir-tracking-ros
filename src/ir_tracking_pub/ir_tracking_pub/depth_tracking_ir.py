import cv2
import sys
import numpy as np
from .module.realsense import Realsense
import cv2.aruco as aruco
import threading

import os

def dilation(dilationSize, kernelSize, img):  # 膨張した画像にして返す
    kernel = np.ones((kernelSize, kernelSize), np.uint8)
    element = cv2.getStructuringElement(
        cv2.MORPH_RECT, (2 * dilationSize + 1, 2 * dilationSize + 1), (dilationSize, dilationSize))
    dilation_img = cv2.dilate(img, kernel, element)
    return dilation_img

def detect(depth_diff, thresh_diff=30, dilationSize=9, kernelSize=20):  # 一定面積以上の物体を検出
    retval, black_diff = cv2.threshold(
        depth_diff, thresh_diff, 255, cv2.THRESH_BINARY)  # 2値化
    dilation_img = dilation(dilationSize, kernelSize, black_diff)  # 膨張処理
    img = dilation_img.copy()
    contours, hierarchy = cv2.findContours(
        dilation_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 境界線検出

    object_pos = []
    before_object_pos = [0,0]
    gradients = []

    for contour in contours[:1]:  # 重心位置を計算
        if cv2.contourArea(contour) < 750:
            continue
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            object_pos.append([cX, cY])
            if before_object_pos != [0, 0]:     #1ループ前の座標を用いて、傾きの計算をする
                gradient = (before_object_pos[1] - cY) / (before_object_pos[0] - cX) if (before_object_pos[0] - cX) != 0 else float('inf')
                gradients.append(gradient)
            before_object_pos = [cX, cY]
    return object_pos, img,gradients

def displayCircle(image, objectList, thickness=5):
    for obj in objectList:
        x = int(obj[0])
        y = int(obj[1])
        cv2.circle(image, (x, y), 10, (255, 0, 0), thickness)
    return image

def resizeImage(image, w=2, h=2):
    height = image.shape[0]
    width = image.shape[1]
    resizedImage = cv2.resize(image, (int(width / w), int(height / h)))
    return resizedImage

def blackToColor(bImage):
    colorImage = np.array((bImage, bImage, bImage))
    colorImage = colorImage.transpose(1, 2, 0)
    return colorImage


class IRTracker:
    def __init__(self):
        current_dir=os.path.dirname(__file__)
        self.m_depth=np.load(os.path.abspath(os.path.join(current_dir,os.pardir,os.pardir,os.pardir,os.pardir,'share','ir_tracking_pub','pts1.npy')))
        self.realsense = Realsense()
        self.width = 960 # 変形後画像サイズ
        self.height = 540
        self.true_coordinates   = np.float32([[0,0],[self.width,0],[self.width,self.height],[0,self.height]])
        self.frame_ir_pre = self.realsense.get_ir_frame()
        self.x = 0
        self.y = 0
        self.x_prev = 0
        self.y_prev = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while True:
            # frame_rgb = self.realsense.get_rgb_frame()
            frame_ir = self.realsense.get_ir_frame()

            trans_mat_ir = cv2.getPerspectiveTransform(self.m_depth,self.true_coordinates)
            frame_ir = cv2.warpPerspective(frame_ir,trans_mat_ir,(self.width, self.height))

            retval, frame_ir_bin= cv2.threshold(frame_ir, 240, 255, cv2.THRESH_BINARY)  # 2値化
            # frame_ir_next = frame_ir_bin.copy()
            try:
                depth_diff = cv2.absdiff(frame_ir_bin, self.frame_ir_pre)  # フレーム間の差分計算
            
                #gray_diff = cv2.cvtColor(depth_diff, cv2.COLOR_BGR2GRAY)  # グレースケール変換

                # objects, dilation_img ,gradient= detect(depth_diff, thresh_diff=1)
                objects=[]
                _,_,_,location =cv2.minMaxLoc(depth_diff) 
                objects.append(location)
                frame_ir = displayCircle(frame_ir, objects, 2)  # 丸で加工
                self.x_prev=self.x
                self.y_prev=self.y
                if(location[0]!=0 and location[1]!=0):
                    self.x=location[0]
                    self.y=location[1]
                # cv2.imshow("ori", frame_ir)
                cv2.imshow("IR",frame_ir)
                # cv2.imshow("gray_diff", depth_diff)
            except Exception as e:
                print(e)

            self.frame_ir_pre = frame_ir_bin.copy()
            #frame_rgb_pre = frame_rgb_next.copy()  # 次のフレームの読み込み
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def get_pos(self):
        # x,y=0,0
        # # frame_rgb = self.realsense.get_rgb_frame()
        # frame_ir = self.realsense.get_ir_frame()

        # trans_mat_ir = cv2.getPerspectiveTransform(self.m_depth,self.true_coordinates)
        # frame_ir = cv2.warpPerspective(frame_ir,trans_mat_ir,(self.width, self.height))

        # retval, frame_ir_bin= cv2.threshold(frame_ir, 240, 255, cv2.THRESH_BINARY)  # 2値化
        # # frame_ir_next = frame_ir_bin.copy()
        # try:
        #     depth_diff = cv2.absdiff(frame_ir_bin, self.frame_ir_pre)  # フレーム間の差分計算
        
        #     #gray_diff = cv2.cvtColor(depth_diff, cv2.COLOR_BGR2GRAY)  # グレースケール変換

        #     # objects, dilation_img ,gradient= detect(depth_diff, thresh_diff=1)
        #     objects=[]
        #     _,_,_,location =cv2.minMaxLoc(depth_diff) 
        #     objects.append(location)
        #     frame_ir = displayCircle(frame_ir, objects, 2)  # 丸で加工
        #     x=location[0]
        #     y=location[1]
        #     # cv2.imshow("ori", frame_ir)
        #     self.imshow("IR",frame_ir)
        #     self.imshow("gray_diff", depth_diff)
        # except Exception as e:
        #     print(e)
                
        
        

        
        # self.frame_ir_pre = frame_ir_bin.copy()
        # #frame_rgb_pre = frame_rgb_next.copy()  # 次のフレームの読み込み

        # # cv2.destroyAllWindows()

        return self.x,self.y



# if __name__ == '__main__':
#     run()