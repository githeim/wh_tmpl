#!/bin/python3
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):
  def __init__(self):
    super().__init__('WW_ProjectName_WW')
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    # Call on_timer function every second
    self.timer = self.create_timer(1.0, self.on_timer)

  def on_timer(self):
    from_frame_rel = 'base_link'
    to_frame_rel = 'map'        
    print ("chk")
    try:
      t = self.tf_buffer.lookup_transform(
          to_frame_rel,
          from_frame_rel,
          rclpy.time.Time())
    except TransformException as ex:
      self.get_logger().info(
          f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
    print ("base_link location (%f,%f,%f) (%f,%f,%f,%f)" % 
           (t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
            t.transform.rotation.x ,
            t.transform.rotation.y ,
            t.transform.rotation.z ,
            t.transform.rotation.w)
           )

def main():
  print ("Start WW_ProjectName_WW Ver WW_MajorVer_WW . WW_MinorVer_WW")
  rclpy.init()
  node = FrameListener()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()


if __name__ == '__main__':
  exit( main())

