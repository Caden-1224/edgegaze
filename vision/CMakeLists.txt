cmake_minimum_required(VERSION 3.16)

project(edgegaze_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EDGEGAZE_WITH_OPENCV "Build OpenCV-backed camera and video-file frame sources" ON)

if(EDGEGAZE_WITH_OPENCV)
  find_package(OpenCV QUIET COMPONENTS core imgproc videoio)
endif()

add_library(edgegaze_runtime runtime/frame_runtime.cpp)
target_include_directories(edgegaze_runtime PUBLIC runtime)
target_compile_features(edgegaze_runtime PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(edgegaze_runtime PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(OpenCV_FOUND)
  target_compile_definitions(edgegaze_runtime PRIVATE EDGEGAZE_WITH_OPENCV=1)
  target_link_libraries(edgegaze_runtime PRIVATE ${OpenCV_LIBS})
  target_include_directories(edgegaze_runtime PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()
