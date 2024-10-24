import cv2
from cv2 import aruco
import numpy as np
import pyrealsense2 as rs
from enum import IntEnum;

class REALSENSE_PARA(IntEnum):
    WIDTH = 848 # カメラの入力画像の幅
    HEIGHT = 480 # カメラの入力画像の高さ
    RGB_FRAMERATE = 60
    DEPTH_FRAMERATE = 90
    IR_FRAMERATE = 60

class Camera():
    def __init__(self):
        # カメラ初期設定
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, REALSENSE_PARA.WIDTH, REALSENSE_PARA.HEIGHT, rs.format.bgr8, REALSENSE_PARA.RGB_FRAMERATE)
        self.config.enable_stream(rs.stream.infrared, 1, REALSENSE_PARA.WIDTH, REALSENSE_PARA.HEIGHT, rs.format.y8, REALSENSE_PARA.IR_FRAMERATE)
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.config)

        device = self.profile.get_device()
        depth_sensor = device.query_sensors()[0]
        emitter = depth_sensor.set_option(rs.option.emitter_enabled,0)
        depth_sensor.set_option(rs.option.enable_auto_exposure, 1)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # マーカー四角形の指定
        self.preset_ids = [0, 1, 2, 3]              # 現実世界で貼るマーカーのid
        self.preset_corner_ids = [0, 1, 2, 3]       # 各マーカーのどの頂点の座標を取得するか


        self.frameW = REALSENSE_PARA.WIDTH
        self.frameH = REALSENSE_PARA.HEIGHT

        # 一つ前の有効画像の初期値　画像取得できなかったときに使う
        self.last_frame = np.zeros((self.frameH, self.frameW, 3), np.uint8)
        self.dic_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


    def get_rgb_frame(self):

        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_infrared_frame()
        if not color_frame:
            print("can't get color frame")
            return

        #imageをnumpy arrayに
        color_image = np.asanyarray(color_frame.get_data())

        return color_image
    
    def get_rect(self):
        
        frame = Camera.get_rgb_frame(self)
    
        # マーカー検出
        corners, ids, _ = aruco.detectMarkers(frame, self.dic_aruco)     # マーカー検出
        image1 = aruco.drawDetectedMarkers(frame.copy(), corners, ids)  # 検出結果を重ね書き

        # 4個のマーカーから四角形を得る
        list_ids = np.ravel(ids)                        # idsを一次元化する
        if set(self.preset_ids) <= set(list_ids):       # 検出結果に4個のidが含まれていたら
            pt = {}
            for id, corner in zip(list_ids, corners):   # まずは検出結果について
                pt[id] = corner[0]                      # idとcornerを紐づける
            pts = []                                    # 次に事前登録した4個のidについて
            for id, corner_id in zip(self.preset_ids, self.preset_corner_ids):
                pts.append(pt[id][corner_id])           # 特定の頂点の座標を順にリストに追加する
            pts1 = np.float32(pts)                      # 投影変換する前の四角形

            np.save("pts1.npy", pts1)                   # 4点の座標をバイナリ保存
            print(pts1)

        self.image1 = image1
    
    def show(self):
        cv2.imshow("camera", self.image1)

def main():
    camera = Camera()
    while True:
        camera.get_rect()
        camera.show()

        key = cv2.waitKey(1) & 0xFF
        if key == 27:                   # esc key
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()