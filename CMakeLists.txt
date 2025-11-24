cmake_minimum_required(VERSION 3.8)
project(ut_start_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(plotjuggler REQUIRED)

add_library(${PROJECT_NAME} 
  src/bag_recorder.cpp
)

# Add include directory
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  px4_msgs
  audio_common_msgs
  rosbag2_cpp
)

# Add executables
add_executable(recorder src/main.cpp)
target_link_libraries(recorder ${PROJECT_NAME})
ament_target_dependencies(recorder 
  std_msgs
  px4_msgs
  audio_common_msgs
  rosbag2_cpp
)


install(TARGETS 
  ${PROJECT_NAME}
  recorder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)  

ament_package()
