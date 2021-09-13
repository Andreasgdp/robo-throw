cmake_minimum_required (VERSION 3.8)
 
project(app)
 
set(SOURCE_FILES "test.cpp")
set(HEADER_FILES "test.h")
 
# We declare the project as a static library and add all the source code files to it.
add_library(frontendScr ${HEADER_FILES} ${SOURCE_FILES})
