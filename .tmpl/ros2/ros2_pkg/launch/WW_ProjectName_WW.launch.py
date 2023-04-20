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
def generate_launch_description():
  ld = LaunchDescription([
         LogInfo(msg = ['Launch ;'+PACKAGE_NAME]),
  ])

  node = launch_ros.actions.LifecycleNode(
          name = PACKAGE_NAME,
          package = PACKAGE_NAME,
          executable = NODE_EXECUTABLE,
          namespace = NODE_NAMESPACE,
          parameters=[
            os.path.join(
              get_package_share_directory('WW_ProjectName_WW'),
              'params', 'WW_ProjectName_WW_params.yaml'
              )
            ],
          output = 'screen',
          )
  # When the node reaches the 'inactive' state, make it to the 'activate'
  evt_hwnd = launch.actions.RegisterEventHandler(
      launch_ros.event_handlers.OnStateTransition(
        target_lifecycle_node = node, goal_state='inactive',
        entities=[
            launch.actions.LogInfo(
              msg = PACKAGE_NAME + 
                    " reached the 'inactive' state, 'activating'."),
            launch.actions.EmitEvent(
              event=launch_ros.events.lifecycle.ChangeState(
              lifecycle_node_matcher=launch.events.matches_action(node),
              transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )),
        ],
      )
    )

  # Make the node take the 'configure' transition.
  emit_configure_transition = launch.actions.EmitEvent(
      event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch.events.matches_action(node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
      )
    )

  ld.add_action(evt_hwnd)
  ld.add_action(node)
  ld.add_action(emit_configure_transition)

  return ld
