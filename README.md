# 起動
Ubuntu
```
colcon build --packages-select ir_tracking_pub
source /opt/ros/humble/setup.sh
ros2 run ir_tracking_pub talker
```
Windows(CMD)
```
pip install opencv-contrib-python
pip install pyrealsense2

call C:\dev\ros2_humble\local_setup.bat
ir-tracking-ros直下で
colcon build --packages-select ir_tracking_pub
ros2 run ir_tracking_pub talker
```