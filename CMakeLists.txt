cmake_minimum_required(VERSION 3.20.0)
project(ovinf)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/dknt/Library/openvino-2025.0.0/runtime/cmake")

find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(ovinf SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ovinf/ovinf_humanoid.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ovinf/ovinf_humanoid_stand.cc
)
target_include_directories(ovinf PUBLIC
  ${OpenVINO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)
target_link_libraries(ovinf
  openvino::runtime
  yaml-cpp
)

set(BUILD_TEST OFF CACHE BOOL "Build test executables")
if (BUILD_TEST) 
  add_subdirectory(test)
endif()

