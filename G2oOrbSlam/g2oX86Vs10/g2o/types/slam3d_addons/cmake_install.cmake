# Install script for directory: D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons

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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_types_slam3d_addons_d.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_types_slam3d_addons_d.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_types_slam3d_addons.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_types_slam3d_addons.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_types_slam3d_addons_s.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_types_slam3d_addons_s.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_types_slam3d_addons_rd.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_types_slam3d_addons_rd.lib")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_types_slam3d_addons_d.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_types_slam3d_addons_d.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_types_slam3d_addons.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_types_slam3d_addons.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_types_slam3d_addons_s.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_types_slam3d_addons_s.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_types_slam3d_addons_rd.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_types_slam3d_addons_rd.dll")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/edge_line3d.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/edge_plane.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/edge_se3_calib.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/edge_se3_euler.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/edge_se3_line.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/edge_se3_plane_calib.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/line3d.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/plane3d.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/types_slam3d_addons.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/vertex_line3d.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/vertex_plane.h;C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons/vertex_se3_euler.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/include/g2o/types/slam3d_addons" TYPE FILE FILES
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/edge_line3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/edge_plane.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/edge_se3_calib.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/edge_se3_euler.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/edge_se3_line.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/edge_se3_plane_calib.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/line3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/plane3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/types_slam3d_addons.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/vertex_line3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/vertex_plane.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/types/slam3d_addons/vertex_se3_euler.h"
    )
endif()

