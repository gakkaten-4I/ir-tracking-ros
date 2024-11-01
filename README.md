# 起動
Ubuntu
```
colcon build --packages-select ir_tracking_pub
source /opt/ros/humble/setup.sh
ros2 run ir_tracking_pub talker
```
Windows(CMD)
https://github.com/IntelRealSense/librealsense/releases/tag/v2.56.2
```
pip install opencv-contrib-python
pip install pyrealsense2

call C:\dev\ros2_humble\local_setup.bat
ir-tracking-ros直下で
colcon build --packages-select ir_tracking_pub
call install/setup.bat
ros2 run ir_tracking_pub talker
```
キャリブレーションファイルpts1.npyの作成方法（pts1.npyをdepth_tracking_ir.pyと同じ場所に置く）
```
python src/ir_tracking_pub/ir_tracking_pub/calibration_realsense.py
python calibration_realsense.py
```