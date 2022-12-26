# find dependencies                       
find_package(ament_cmake REQUIRED)        
find_package(geometry_msgs REQUIRED)      
find_package(rclcpp REQUIRED)             
find_package(tf2 REQUIRED)                
find_package(tf2_ros REQUIRED)            
find_package(tf2_geometry_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)        

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  )
add_executable(WW_ProjectName_WW
  src/WW_ProjectName_WW_main.cpp
  )
target_link_libraries(WW_ProjectName_WW
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${sensor_msgs_LIBRARIES}

)
ament_target_dependencies(WW_ProjectName_WW 
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  )


install(TARGETS
  WW_ProjectName_WW
  DESTINATION lib/${PROJECT_NAME}
  )


