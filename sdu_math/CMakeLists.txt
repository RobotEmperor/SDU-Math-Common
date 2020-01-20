cmake_minimum_required(VERSION 2.8.3)

# 프로젝트 이름 및 버전
PROJECT ( "sdu_math" )


LINK_DIRECTORIES (/usr/lib)
 


add_compile_options(-std=c++11)
 

find_package(Eigen3 REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

## Declare a C++ library
 add_library(${PROJECT_NAME} SHARED
   src/kinematics.cpp
   src/statics_math.cpp
 )


target_link_libraries(sdu_math ${catkin_LIBRARIES})
