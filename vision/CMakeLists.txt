cmake_minimum_required(VERSION 3.16)

project(edgegaze_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EDGEGAZE_WITH_OPENCV "Build OpenCV-backed camera and video-file frame sources" ON)
option(EDGEGAZE_WITH_BPU "Build the RDK X5 BPU body detector" OFF)

if(EDGEGAZE_WITH_OPENCV)
  find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs videoio)
endif()

add_library(edgegaze_runtime runtime/frame_runtime.cpp)
target_include_directories(edgegaze_runtime PUBLIC runtime)
target_compile_features(edgegaze_runtime PUBLIC cxx_std_17)

add_library(edgegaze_detector detector/body_detector.cpp)
target_include_directories(edgegaze_detector PUBLIC detector runtime)
target_link_libraries(edgegaze_detector PUBLIC edgegaze_runtime)
target_compile_features(edgegaze_detector PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(edgegaze_runtime PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(edgegaze_detector PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(OpenCV_FOUND)
  target_compile_definitions(edgegaze_runtime PRIVATE EDGEGAZE_WITH_OPENCV=1)
  target_link_libraries(edgegaze_runtime PRIVATE ${OpenCV_LIBS})
  target_include_directories(edgegaze_runtime PRIVATE ${OpenCV_INCLUDE_DIRS})

  target_compile_definitions(edgegaze_detector PRIVATE EDGEGAZE_WITH_OPENCV=1)
  target_link_libraries(edgegaze_detector PRIVATE ${OpenCV_LIBS})
  target_include_directories(edgegaze_detector PRIVATE ${OpenCV_INCLUDE_DIRS})

  add_executable(edgegaze_detect_video apps/detect_video.cpp)
  target_link_libraries(edgegaze_detect_video PRIVATE edgegaze_detector edgegaze_runtime ${OpenCV_LIBS})
  target_include_directories(edgegaze_detect_video PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_compile_features(edgegaze_detect_video PRIVATE cxx_std_17)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(edgegaze_detect_video PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

if(EDGEGAZE_WITH_BPU)
  find_path(EDGEGAZE_HB_DNN_INCLUDE_DIR dnn/hb_dnn.h)
  find_library(EDGEGAZE_HB_DNN_LIBRARY NAMES dnn)
  if(NOT EDGEGAZE_HB_DNN_INCLUDE_DIR OR NOT EDGEGAZE_HB_DNN_LIBRARY)
    message(FATAL_ERROR "EDGEGAZE_WITH_BPU=ON requires dnn/hb_dnn.h and libdnn (RDK X5)")
  endif()
  target_compile_definitions(edgegaze_detector PRIVATE EDGEGAZE_WITH_BPU=1)
  target_include_directories(edgegaze_detector PRIVATE ${EDGEGAZE_HB_DNN_INCLUDE_DIR})
  target_link_libraries(edgegaze_detector PRIVATE ${EDGEGAZE_HB_DNN_LIBRARY})
endif()
