# Install script for directory: /home/yik/sdu_ws/SDU-Math-Common

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
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/yik/sdu_ws/lib/libsdu_math.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/yik/sdu_ws/lib/libsdu_math.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/yik/sdu_ws/lib/libsdu_math.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yik/sdu_ws/lib/libsdu_math.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yik/sdu_ws/lib" TYPE SHARED_LIBRARY FILES "/home/yik/sdu_ws/SDU-Math-Common/build/libsdu_math.so")
  if(EXISTS "$ENV{DESTDIR}/home/yik/sdu_ws/lib/libsdu_math.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/yik/sdu_ws/lib/libsdu_math.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/yik/sdu_ws/lib/libsdu_math.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xsdu_mathx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yik/sdu_ws/include/sdu_math/control_function.h;/home/yik/sdu_ws/include/sdu_math/kinematics.h;/home/yik/sdu_ws/include/sdu_math/statistics_math.h;/home/yik/sdu_ws/include/sdu_math/end_point_to_rad_cal.h;/home/yik/sdu_ws/include/sdu_math/fifth_order_trajectory_generate.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yik/sdu_ws/include/sdu_math" TYPE FILE FILES
    "/home/yik/sdu_ws/SDU-Math-Common/sdu_math/include/sdu_math/control_function.h"
    "/home/yik/sdu_ws/SDU-Math-Common/sdu_math/include/sdu_math/kinematics.h"
    "/home/yik/sdu_ws/SDU-Math-Common/sdu_math/include/sdu_math/statistics_math.h"
    "/home/yik/sdu_ws/SDU-Math-Common/sdu_math/include/sdu_math/end_point_to_rad_cal.h"
    "/home/yik/sdu_ws/SDU-Math-Common/sdu_math/include/sdu_math/fifth_order_trajectory_generate.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/yik/sdu_ws/SDU-Math-Common/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
