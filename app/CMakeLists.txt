cmake_minimum_required(VERSION 3.5)
project(app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


File(GLOB_RECURSE SOURCE_FILES */*.cpp)
File(GLOB_RECURSE HEADER_FILES */*.h)

# Opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#Robotics Library (gripper). HAL is the subpart of the library where WeissWsg50 is located.
find_package(RL COMPONENTS HAL REQUIRED)

# pylon
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon)
include_directories(${Pylon_INCLUDE_DIRS})

# UR-RTDE
find_package(ur_rtde REQUIRED)

# SQL
find_package(Qt6 COMPONENTS Sql REQUIRED)

# Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Adds executables
add_executable(app main.cpp ${SOURCE_FILES} ${HEADER_FILES})

# Link libraries
target_link_libraries(app PRIVATE ur_rtde::rtde ${OpenCV_LIBS} ${Pylon_LIBRARIES} ${RL_LIBRARIES} Eigen3::Eigen Qt6::Sql)
