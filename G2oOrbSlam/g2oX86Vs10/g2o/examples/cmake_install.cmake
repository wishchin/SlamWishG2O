# Install script for directory: D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/g2o")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/data_fitting/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/sphere/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/tutorial_slam2d/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/icp/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/calibration_odom_laser/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/simple_optimize/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/plane_slam/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/sba/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/bal/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/data_convert/cmake_install.cmake")
  include("D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2oX86Vs10/g2o/examples/interactive_slam/cmake_install.cmake")

endif()

