CMAKE_MINIMUM_REQUIRED ( VERSION 3.0 )

PROJECT ( sdu_math )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Eigen3
)

set(LIB_HEADER_FILES
   include/sdu_math/control_function.h
   include/sdu_math/kinematics.h
   include/sdu_math/statistics_math.h
   include/sdu_math/end_point_to_rad_cal.h
   include/sdu_math/fifth_order_trajectory_generate.h
   )

set( sdu_math_SOURCE_FILES
 src/control_function.cpp
 src/kinematics.cpp
 src/statistics_math.cpp
 src/end_point_to_rad_cal.cpp
 src/fifth_order_trajectory_generate.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS}) # To satisfy catkin_lint

add_library(sdu_math SHARED ${sdu_math_SOURCE_FILES} ${LIB_HEADER_FILES})
target_include_directories(sdu_math PUBLIC include ${EIGEN3_INCLUDE_DIR})

if (CMAKE_COMPILER_IS_GNUCXX)
  # Give error in Linux if library is not fully linked (like Windows)
  set_target_properties(sdu_math PROPERTIES LINK_FLAGS -Wl,--no-undefined)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

