cmake_minimum_required(VERSION 3.5)

project(app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(api)
add_subdirectory(frontend)
add_subdirectory(gripperHandling)
add_subdirectory(imageProcessing)
add_subdirectory(robotConnection)
add_subdirectory(simulation)
add_subdirectory(jointPoseGetter)

add_executable(app main.cpp)
