from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch.actions
import launch_ros.actions.node
from launch.actions import LogInfo

import lifecycle_msgs.msg

PACKAGE_NAME = 'WW_ProjectName_WW'
NODE_EXECUTABLE = PACKAGE_NAME
NODE_NAMESPACE = '' 
NODE_PARAM='WW_ProjectName_WW_params.yaml'

def generate_launch_description():
  return LaunchDescription([
    Node(
      name = PACKAGE_NAME,
      package = PACKAGE_NAME,
      executable = NODE_EXECUTABLE,
      namespace = NODE_NAMESPACE,
      parameters=[
        os.path.join(
          get_package_share_directory('WW_ProjectName_WW'),
          'params', NODE_PARAM
          )
        ],
      output = 'screen',
      )
    ])
