. set.sh
echo transmit the action for testing
ros2 action send_goal /test_mission test_action_interface/action/TestMission "\
  {  pose: \
       {  pose : {  position : {  x: 10, y: 10, z: 0 } } } \
  }"
