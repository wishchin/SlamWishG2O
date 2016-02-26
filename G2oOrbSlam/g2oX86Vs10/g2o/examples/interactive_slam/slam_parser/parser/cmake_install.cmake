# Install script for directory: D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser

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
     "C:/Program Files (x86)/g2o/lib/g2o_parser_d.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_parser_d.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_parser.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_parser.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_parser_s.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_parser_s.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_parser_rd.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_parser_rd.lib")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/g2o/include/slam_parser/parser/bison_parser.h;C:/Program Files (x86)/g2o/include/slam_parser/parser/commands.h;C:/Program Files (x86)/g2o/include/slam_parser/parser/driver.h;C:/Program Files (x86)/g2o/include/slam_parser/parser/FlexLexer.h;C:/Program Files (x86)/g2o/include/slam_parser/parser/scanner.h;C:/Program Files (x86)/g2o/include/slam_parser/parser/slam_context.h;C:/Program Files (x86)/g2o/include/slam_parser/parser/location.hh;C:/Program Files (x86)/g2o/include/slam_parser/parser/position.hh;C:/Program Files (x86)/g2o/include/slam_parser/parser/stack.hh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/include/slam_parser/parser" TYPE FILE FILES
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/bison_parser.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/commands.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/driver.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/FlexLexer.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/scanner.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/slam_context.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/location.hh"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/position.hh"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/examples/interactive_slam/slam_parser/parser/stack.hh"
    )
endif()

