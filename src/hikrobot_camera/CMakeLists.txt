cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MVS_ROOT "" CACHE PATH "Root directory of the Hikrobot MVS SDK")
find_path(MVS_INCLUDE_DIR MvCameraControl.h
  HINTS ${MVS_ROOT} $ENV{MVCAM_SDK_PATH}
  PATH_SUFFIXES include include/64
)
find_library(MVS_LIBRARY MvCameraControl
  HINTS ${MVS_ROOT} $ENV{MVCAM_SDK_PATH}
  PATH_SUFFIXES lib lib/64 lib64
)

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)
target_include_directories(camera_node PRIVATE include)
ament_target_dependencies(camera_node rclcpp rcl_interfaces sensor_msgs std_msgs)

if(MVS_INCLUDE_DIR AND MVS_LIBRARY)
  target_compile_definitions(camera_node PRIVATE HIKROBOT_CAMERA_HAS_MVS=1)
  target_include_directories(camera_node PRIVATE ${MVS_INCLUDE_DIR})
  target_link_libraries(camera_node ${MVS_LIBRARY})
  message(STATUS "Hikrobot MVS SDK enabled: ${MVS_LIBRARY}")
else()
  message(WARNING "Hikrobot MVS SDK not found; camera_node will build but cannot acquire images")
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
