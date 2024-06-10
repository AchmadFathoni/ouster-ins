cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(pcl_filter src/pcl_filter_node.cpp)
ament_target_dependencies(pcl_filter
  rclcpp 
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pcl_filter ${PCL_LIBRARIES})

add_executable(save_pointcloud src/save_pointcloud.cpp)
ament_target_dependencies(save_pointcloud
  rclcpp 
  sensor_msgs
  std_srvs
  pcl_conversions
)
target_link_libraries(save_pointcloud ${PCL_LIBRARIES})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS pcl_filter save_pointcloud DESTINATION lib/${PROJECT_NAME})
ament_package()
