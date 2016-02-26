# Install script for directory: D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core

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
     "C:/Program Files (x86)/g2o/lib/g2o_core_d.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_core_d.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_core.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_core.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_core_s.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_core_s.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/lib/g2o_core_rd.lib")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_core_rd.lib")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_core_d.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Debug/g2o_core_d.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_core.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/Release/g2o_core.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_core_s.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/MinSizeRel/g2o_core_s.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "C:/Program Files (x86)/g2o/bin/g2o_core_rd.dll")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/bin" TYPE SHARED_LIBRARY FILES "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/bin/RelWithDebInfo/g2o_core_rd.dll")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files (x86)/g2o/include/g2o/core/base_binary_edge.h;C:/Program Files (x86)/g2o/include/g2o/core/base_edge.h;C:/Program Files (x86)/g2o/include/g2o/core/base_multi_edge.h;C:/Program Files (x86)/g2o/include/g2o/core/base_unary_edge.h;C:/Program Files (x86)/g2o/include/g2o/core/base_vertex.h;C:/Program Files (x86)/g2o/include/g2o/core/batch_stats.h;C:/Program Files (x86)/g2o/include/g2o/core/block_solver.h;C:/Program Files (x86)/g2o/include/g2o/core/cache.h;C:/Program Files (x86)/g2o/include/g2o/core/creators.h;C:/Program Files (x86)/g2o/include/g2o/core/eigen_types.h;C:/Program Files (x86)/g2o/include/g2o/core/estimate_propagator.h;C:/Program Files (x86)/g2o/include/g2o/core/factory.h;C:/Program Files (x86)/g2o/include/g2o/core/g2o_core_api.h;C:/Program Files (x86)/g2o/include/g2o/core/hyper_dijkstra.h;C:/Program Files (x86)/g2o/include/g2o/core/hyper_graph.h;C:/Program Files (x86)/g2o/include/g2o/core/hyper_graph_action.h;C:/Program Files (x86)/g2o/include/g2o/core/jacobian_workspace.h;C:/Program Files (x86)/g2o/include/g2o/core/linear_solver.h;C:/Program Files (x86)/g2o/include/g2o/core/marginal_covariance_cholesky.h;C:/Program Files (x86)/g2o/include/g2o/core/matrix_operations.h;C:/Program Files (x86)/g2o/include/g2o/core/matrix_structure.h;C:/Program Files (x86)/g2o/include/g2o/core/openmp_mutex.h;C:/Program Files (x86)/g2o/include/g2o/core/optimizable_graph.h;C:/Program Files (x86)/g2o/include/g2o/core/optimization_algorithm.h;C:/Program Files (x86)/g2o/include/g2o/core/optimization_algorithm_dogleg.h;C:/Program Files (x86)/g2o/include/g2o/core/optimization_algorithm_factory.h;C:/Program Files (x86)/g2o/include/g2o/core/optimization_algorithm_gauss_newton.h;C:/Program Files (x86)/g2o/include/g2o/core/optimization_algorithm_levenberg.h;C:/Program Files (x86)/g2o/include/g2o/core/optimization_algorithm_property.h;C:/Program Files (x86)/g2o/include/g2o/core/optimization_algorithm_with_hessian.h;C:/Program Files (x86)/g2o/include/g2o/core/parameter.h;C:/Program Files (x86)/g2o/include/g2o/core/parameter_container.h;C:/Program Files (x86)/g2o/include/g2o/core/robust_kernel.h;C:/Program Files (x86)/g2o/include/g2o/core/robust_kernel_factory.h;C:/Program Files (x86)/g2o/include/g2o/core/robust_kernel_impl.h;C:/Program Files (x86)/g2o/include/g2o/core/solver.h;C:/Program Files (x86)/g2o/include/g2o/core/sparse_block_matrix.h;C:/Program Files (x86)/g2o/include/g2o/core/sparse_block_matrix_ccs.h;C:/Program Files (x86)/g2o/include/g2o/core/sparse_block_matrix_diagonal.h;C:/Program Files (x86)/g2o/include/g2o/core/sparse_optimizer.h;C:/Program Files (x86)/g2o/include/g2o/core/sparse_optimizer_terminate_action.h;C:/Program Files (x86)/g2o/include/g2o/core/base_binary_edge.hpp;C:/Program Files (x86)/g2o/include/g2o/core/base_multi_edge.hpp;C:/Program Files (x86)/g2o/include/g2o/core/base_unary_edge.hpp;C:/Program Files (x86)/g2o/include/g2o/core/base_vertex.hpp;C:/Program Files (x86)/g2o/include/g2o/core/block_solver.hpp;C:/Program Files (x86)/g2o/include/g2o/core/sparse_block_matrix.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files (x86)/g2o/include/g2o/core" TYPE FILE FILES
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_binary_edge.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_edge.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_multi_edge.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_unary_edge.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_vertex.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/batch_stats.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/block_solver.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/cache.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/creators.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/eigen_types.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/estimate_propagator.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/factory.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/g2o_core_api.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/hyper_dijkstra.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/hyper_graph.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/hyper_graph_action.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/jacobian_workspace.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/linear_solver.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/marginal_covariance_cholesky.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/matrix_operations.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/matrix_structure.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/openmp_mutex.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimizable_graph.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimization_algorithm.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimization_algorithm_dogleg.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimization_algorithm_factory.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimization_algorithm_levenberg.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimization_algorithm_property.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/optimization_algorithm_with_hessian.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/parameter.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/parameter_container.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/robust_kernel.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/robust_kernel_factory.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/robust_kernel_impl.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/solver.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/sparse_block_matrix.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/sparse_block_matrix_ccs.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/sparse_block_matrix_diagonal.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/sparse_optimizer.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/sparse_optimizer_terminate_action.h"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_binary_edge.hpp"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_multi_edge.hpp"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_unary_edge.hpp"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/base_vertex.hpp"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/block_solver.hpp"
    "D:/CodeBase/IisuVR/ORB_SLAM_vs2010_x86/g2o/g2o/core/sparse_block_matrix.hpp"
    )
endif()

