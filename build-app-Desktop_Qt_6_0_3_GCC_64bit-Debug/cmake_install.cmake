# Install script for directory: /git/GitHub/robo-throw/app

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/api/cmake_install.cmake")
  include("/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/frontend/cmake_install.cmake")
  include("/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/gripperHandling/cmake_install.cmake")
  include("/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/imageProcessing/cmake_install.cmake")
  include("/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/robotConnection/cmake_install.cmake")
  include("/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/simulation/cmake_install.cmake")
  include("/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/jointPoseGetter/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/git/GitHub/robo-throw/build-app-Desktop_Qt_6_0_3_GCC_64bit-Debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
