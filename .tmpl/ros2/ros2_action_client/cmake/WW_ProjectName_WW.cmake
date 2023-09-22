find_package(example_interfaces REQUIRED)
find_package(test_action_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


include_directories(
  include
  )
add_executable(WW_ProjectName_WW
  src/WW_ProjectName_WW_main.cpp
  )
ament_target_dependencies(WW_ProjectName_WW
  "rclcpp"
  "rclcpp_action"
  "test_action_interface"
  )


target_link_libraries(WW_ProjectName_WW
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

install(TARGETS
  WW_ProjectName_WW
  DESTINATION lib/${PROJECT_NAME}
  )


