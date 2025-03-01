cmake_minimum_required(VERSION 3.10.0)
project(gongxun_v)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 4.11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# CUDA
find_package(CUDA REQUIRED)
message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include /usr/include/aarch-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)
message(STATUS "TensorRT Libs: \n\n${TensorRT_LIBRARIES}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
  DEPENDS OpenCV Eigen3 nlohmann_json
)

include_directories(
  ${TensorRT_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  /usr/local/include
  /usr/local/include/opencv4
  /usr/include
)

set(LIBS 
  ${CUDA_LIBRARIES}
  ${CUDA_LIB_DIR}
  ${TensorRT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBRARIES}
  ${nlohmann_json_LIBRARIES}
)

# Node for helloworld
add_executable(helloworld
  src/helloworld.cpp
  src/parameters.cpp
)
target_link_libraries(helloworld
  ${LIBS}
)

# Node for camera
add_executable(camera
  src/camera.cpp
  src/parameters.cpp
)
target_link_libraries(camera
  ${LIBS}
)

# Node for testbridge
add_executable(testbridge
  src/testbridge.cpp
  src/yolov8.cpp
)
target_link_libraries(testbridge
  ${LIBS}
  nvinfer nvinfer_plugin cudart
)
