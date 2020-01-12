include_directories(
  include
  )
add_executable(WW_ProjectName_WW
  src/WW_NodeClassName_WW.cpp
  src/WW_ProjectName_WW_main.cpp
  )
target_link_libraries(WW_ProjectName_WW
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

install(TARGETS
  WW_ProjectName_WW
  DESTINATION lib/${PROJECT_NAME}
  )


