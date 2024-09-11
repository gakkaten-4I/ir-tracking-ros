# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import geometry_msgs.msg
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from .depth_tracking_ir import IRTracker

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'chatter', 10)
        timer_period = 0.0111  # seconds 90fps
        self.tracker = IRTracker()
        self.timer = self.create_timer(timer_period, self.publish_pos)


    def publish_pos(self):
        msg = Vector3()
        x,y=self.tracker.get_pos()
        msg.y = float(y*(-1))
        msg.x = float(x)
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing: ({msg.x}, {msg.y})')


def main(args=None):
    rclpy.init(args=args)

    position_publisher = PositionPublisher()

    rclpy.spin(position_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
