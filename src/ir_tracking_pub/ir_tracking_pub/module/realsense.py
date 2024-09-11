import pyrealsense2 as rs
import numpy as np
import cv2
from enum import IntEnum;
from cv2 import aruco

class REALSENSE_PARA(IntEnum):
    WIDTH = 640
    HEIGHT = 360
    RGB_FRAMERATE = 60
    DEPTH_FRAMERATE = 90
    IR_FRAMERATE = 60
    
class Realsense:
    def __init__(self):
         # ストリーム(Depth/Color)の設定 (今回depthは使わない)
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, REALSENSE_PARA.WIDTH, REALSENSE_PARA.HEIGHT, rs.format.bgr8, REALSENSE_PARA.RGB_FRAMERATE)
        #self.config.enable_stream(rs.stream.depth, REALSENSE_PARA.WIDTH, REALSENSE_PARA.HEIGHT, rs.format.z16, REALSENSE_PARA.DEPTH_FRAMERATE)

        #irでのストリームの設定(カメラ2つあるので片方だけ使う)
        self.config.enable_stream(rs.stream.infrared, 1, REALSENSE_PARA.WIDTH, REALSENSE_PARA.HEIGHT, rs.format.y8, REALSENSE_PARA.IR_FRAMERATE)
        #self.config.enable_stream(rs.stream.infrared, 2, REALSENSE_PARA.WIDTH, REALSENSE_PARA.HEIGHT, rs.format.y8, REALSENSE_PARA.IR_FRAMERATE)

        # ストリーミング開始
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.config)
        device = self.profile.get_device()
        depth_sensor = device.query_sensors()[0]
        emitter = depth_sensor.set_option(rs.option.emitter_enabled,0)
        depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
        depth_sensor.set_option(rs.option.exposure, 500)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

         # マーカー四角形の指定
        self.preset_ids = [0, 1, 2, 3]              # 現実世界で貼るマーカーのid
        self.preset_corner_ids = [2, 3, 0, 1]       # 各マーカーのどの頂点の座標を取得するか
        self.rectW, self.rectH = 600, 360           # 長方形のサイズ
        self.pts2 = np.float32(
            [(0,0), (self.rectW,0), (self.rectW, self.rectH), (0,self.rectH)])   # 長方形座標

        self.M = np.zeros((3, 3), np.uint8)
        self.frameW = 640
        self.frameH = 360
        # 一つ前の有効画像の初期値　画像取得できなかったときに使う
        self.last_frame = np.zeros((self.frameH, self.frameW, 3), np.uint8)
        self.last_rect = np.zeros((self.rectH, self.rectW, 3), np.uint8)
        self.dic_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    #rgbフレームの獲得
    def get_rgb_frame(self):
        
        # self.M = np.load("transformation_matrix.npy")

        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            print("can't get color frame")
            return

        #imageをnumpy arrayに
        color_image = np.asanyarray(color_frame.get_data())
        #color_image = cv2.warpPerspective(color_image,self.M,(self.rectW,self.rectH))
        #画像リサイズ
        #color_image_s = cv2.resize(color_image, (640, 360))

        return color_image
    
    #depthフレームの獲得
    """def get_depth_frame(self):

        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            print("can't get depth frame")
            return

        #imageをnumpy arrayに
        depth_image = np.asanyarray(depth_frame.get_data())

        #画像リサイズ
        #depth_image_s = cv2.resize(depth_image, (640, 360))

        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

        return depth_image"""
    
    #irフレームの獲得
    def get_ir_frame(self):

        # self.M = np.load("transformation_matrix.npy")

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        ir_frame = aligned_frames.get_infrared_frame(1)

        
        if not ir_frame:
            print("can't get ir frame")
            return

        #imageをnumpy arrayに
        ir_image = np.asanyarray(ir_frame.get_data())

        #ir_rect = cv2.warpPerspective(ir_image,self.M,(self.rectW,self.rectH))
        #画像リサイズ
        #depth_image_s = cv2.resize(depth_image, (640, 360))

        #convertScaleAbsでコントラストを調整
        #applyColorMapでカラーマップを適用
        #ir_image = cv2.applyColorMap(cv2.convertScaleAbs(ir_image, alpha=0.3),cv2.COLORMAP_MAGMA)

        return ir_image