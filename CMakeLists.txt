# Project name
PROJECT(Hanoi)

# CMake required version must be >= 2.6
cmake_minimum_required(VERSION 2.6)

# Find DisCODe installation
FIND_PACKAGE(DisCODe REQUIRED)

# Use CMake macros provided with DisCODe
set(CMAKE_MODULE_PATH ${DisCODe_DIR} ${CMAKE_MODULE_PATH})
include(DisCODeMacros)


function (USE_ROS)
    if (NOT DEFINED ENV{ROS_DISTRO})
        MESSAGE(FATAL_ERROR "ROS not sourced")
    else()
        SET(ROS_DISTRO $ENV{ROS_DISTRO})
        MESSAGE(STATUS "ROS distro: ${ROS_DISTRO}")
    endif()

    # Cannot use ARGN directly with list() command.
    # Copy to a variable first.
    set (extra_macro_args ${ARGN})

    # Did we get any optional args?
    list(LENGTH extra_macro_args num_extra_args)
    if (${num_extra_args} GREATER 0)
        MESSAGE(STATUS "Using ROS packages:")
        foreach(loop_var ${extra_macro_args})
            message(STATUS "${loop_var}")
            FIND_PACKAGE(${loop_var})
            set(INT_ROS_LIBRARIES ${INT_ROS_LIBRARIES} ${${loop_var}_LIBRARIES})
        endforeach()
    endif()

    set(ROS_LIBRARIES ${ROS_LIBRARIES} ${INT_ROS_LIBRARIES} PARENT_SCOPE)
endfunction ()

# Set default install prefix to dist folder
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
      ${CMAKE_SOURCE_DIR}/dist CACHE PATH "${CMAKE_PROJECT_NAME} install prefix" FORCE
  )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

ADD_SUBDIRECTORY(src)

REBUILD_DCL_CACHE()
