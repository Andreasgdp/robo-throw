cmake_minimum_required(VERSION 3.5)
project(app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


File(GLOB_RECURSE SOURCE_FILES */*.cpp)
File(GLOB_RECURSE HEADER_FILES */*.h)

# Opencv
find_package(OpenCV REQUIRED)

#include_directories(${OpenCV_INCLUDE_DIRS})

# pylon
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon)
include_directories(${Pylon_INCLUDE_DIRS})




# UR-RTDE
find_package(ur_rtde REQUIRED)
add_executable(app main.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(app ur_rtde::rtde ${OpenCV_LIBS} ${Pylon_LIBRARIES})





