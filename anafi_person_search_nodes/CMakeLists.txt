cmake_minimum_required(VERSION 3.5)
project(anafi_person_search_nodes)

# Use ccache to speed up rebuilds
include(cmake/ccache.cmake)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CMake Options
option(ENABLE_BENCHMARKS "Benchmark the preprocessing, inference, and post processing" OFF)
if (ENABLE_BENCHMARKS)
    add_compile_definitions(ENABLE_BENCHMARKS)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(anafi_person_search_interfaces REQUIRED)
find_package(anafi_ros_interfaces REQUIRED)

# Include Cpp "include" directory
include_directories(
  include 
  ${OpenCV_INCLUDE_DIRS}
)

# TODO: Specify the path to TensorRT root dir
set(TensorRT_DIR /usr/src/tensorrt/)

# Build the TensorRT inference engine library
# TensorRT is found and linked by the tensorrt-cpp-api
add_subdirectory(libs/tensorrt-cpp-api)

# Build the YoloV8 library
add_library(YoloV8_TRT SHARED src/yolov8.cpp)
set_target_properties(YoloV8_TRT PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY /home/carlos/ros2_ws/install/anafi_person_search_nodes/lib)
target_link_libraries(YoloV8_TRT PUBLIC tensorrt_cpp_api ${OpenCV_LIBS})
target_include_directories(YoloV8_TRT PUBLIC libs/tensorrt-cpp-api/src)

add_library(RTSPcam SHARED src/RTSPcam.cpp)
set_target_properties(RTSPcam PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY /home/carlos/ros2_ws/install/anafi_person_search_nodes/lib)
target_link_libraries(RTSPcam PUBLIC ${OpenCV_LIBS})

# Build and link the executables
add_executable(anafi_rtsp_stream_video src/anafi_rtsp_stream_video.cpp)
ament_target_dependencies(anafi_rtsp_stream_video rclcpp cv_bridge std_msgs sensor_msgs anafi_person_search_interfaces anafi_ros_interfaces)
target_link_libraries(anafi_rtsp_stream_video RTSPcam ${OpenCV_LIBS})

add_executable(test_anafi_flightplan_action src/test_anafi_flightplan_action.cpp)
ament_target_dependencies(test_anafi_flightplan_action rclcpp std_msgs std_srvs anafi_ros_interfaces anafi_person_search_interfaces)

add_executable(anafi_flightplan_action src/anafi_flightplan_action.cpp)
ament_target_dependencies(anafi_flightplan_action rclcpp std_msgs std_srvs anafi_ros_interfaces anafi_person_search_interfaces)

add_executable(anafi_person_detect src/anafi_person_detect.cpp)
ament_target_dependencies(anafi_person_detect rclcpp sensor_msgs std_msgs cv_bridge anafi_ros_interfaces anafi_person_search_interfaces)
target_link_libraries(anafi_person_detect YoloV8_TRT ${OpenCV_LIBS})

add_executable(anafi_face_reubication src/anafi_face_reubication.cpp)
ament_target_dependencies(anafi_face_reubication rclcpp std_msgs anafi_person_search_interfaces anafi_ros_interfaces)

# Install Cpp executables
install(TARGETS
  anafi_rtsp_stream_video
  test_anafi_flightplan_action
  anafi_flightplan_action
  anafi_person_detect
  anafi_face_reubication
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/pytorch2onnx.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
