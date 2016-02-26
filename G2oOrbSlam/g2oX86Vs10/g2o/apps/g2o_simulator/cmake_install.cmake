# Install script for directory: D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator

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
     "C:/Program Files (x86)/g2o/lib/g2o_simulator_d.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_simulator_d.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_simulator.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_simulator.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_simulator_s.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_simulator_s.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_simulator_rd.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_simulator_rd.lib")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator_d.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_simulator_d.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_simulator.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator_s.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_simulator_s.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator_rd.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_simulator_rd.dll")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator2d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_simulator2d.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator2d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_simulator2d.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator2d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_simulator2d.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator2d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_simulator2d.exe")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator3d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_simulator3d.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator3d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_simulator3d.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator3d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_simulator3d.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_simulator3d.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_simulator3d.exe")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_anonymize_observations.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_anonymize_observations.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_anonymize_observations.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_anonymize_observations.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_anonymize_observations.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_anonymize_observations.exe")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_anonymize_observations.exe")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE EXECUTABLE FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_anonymize_observations.exe")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/g2o_simulator_api.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/pointsensorparameters.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_line3d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_odometry.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_odometry2d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_odometry3d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pointxy.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pointxyz.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pointxyz_depth.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pointxyz_disparity.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pointxy_bearing.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pointxy_offset.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pose2d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pose3d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_pose3d_offset.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_se3_prior.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_segment2d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_segment2d_line.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/sensor_segment2d_pointline.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/simulator.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/simulator2d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/simulator2d_base.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/simulator3d.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/simulator3d_base.h;C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator/simutils.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/include/g2o/apps/g2o_simulator" TYPE FILE FILES
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/g2o_simulator_api.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/pointsensorparameters.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_line3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_odometry.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_odometry2d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_odometry3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pointxy.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pointxyz.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pointxyz_depth.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pointxyz_disparity.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pointxy_bearing.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pointxy_offset.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pose2d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pose3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_pose3d_offset.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_se3_prior.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_segment2d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_segment2d_line.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/sensor_segment2d_pointline.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/simulator.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/simulator2d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/simulator2d_base.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/simulator3d.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/simulator3d_base.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/apps/g2o_simulator/simutils.h"
    )
endif()

